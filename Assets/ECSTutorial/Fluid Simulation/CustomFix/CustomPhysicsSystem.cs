
//public class CustomPhysicsSystem : MonoBehaviour

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;
using Unity.Jobs;
using Unity.Burst;
//using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using Unity.Transforms;
using Unity.Collections;
using Unity.Burst.Intrinsics;
using Samples.Boids;

[UpdateInGroup(typeof(SimulationSystemGroup))]
[UpdateBefore(typeof(TransformSystemGroup))]
[BurstCompile]
public partial class CustomPhysicsSystem : SystemBase
{
    private EntityQuery SPHCharacterGroup;
    private EntityQuery SPHColliderGroup;

    private JobHandle collidersToNativeArrayJobHandle;
    //private NativeArray<SPHColliderComponent> colliders;

    private Transform cameraTransform;

    private List<SPHParticleComponent> uniqueTypes = new List<SPHParticleComponent>(10);
    private List<PreviousParticle> previousParticles = new List<PreviousParticle>();

    private static readonly int[] cellOffsetTable =
    {
        1, 1, 1, 1, 1, 0, 1, 1, -1, 1, 0, 1, 1, 0, 0, 1, 0, -1, 1, -1, 1, 1, -1, 0, 1, -1, -1,
        0, 1, 1, 0, 1, 0, 0, 1, -1, 0, 0, 1, 0, 0, 0, 0, 0, -1, 0, -1, 1, 0, -1, 0, 0, -1, -1,
        -1, 1, 1, -1, 1, 0, -1, 1, -1, -1, 0, 1, -1, 0, 0, -1, 0, -1, -1, -1, 1, -1, -1, 0, -1, -1, -1
    };
    private struct PreviousParticle
    {
        public NativeParallelMultiHashMap<int, int> hashMap;
        public NativeArray<LocalTransform> particlesPosition;
        public NativeArray<SPHVelocityComponent> particlesVelocity;
        public NativeArray<float3> particlesForces;
        public NativeArray<float> particlesPressure;
        public NativeArray<float> particlesDensity;
        public NativeArray<int> particleIndices;

        public NativeArray<int> cellOffsetTable;
    }

    #region  Job -------------------

    [BurstCompile]
    private partial struct HashPositions : IJobEntity
    {
        //#pragma warning disable 0649
        [ReadOnly] public float cellRadius;

        public NativeArray<LocalTransform> positions;
        public NativeParallelMultiHashMap<int, int>.ParallelWriter hashMap;
        //#pragma warning restore 0649

        public void Execute(Entity e, [EntityIndexInQuery] int index, in SPHParticleComponent particle)
        {
            float3 position = positions[index].Position;

            int hash = GridHash.Hash(position, cellRadius);
            hashMap.Add(hash, index);

            positions[index] = new LocalTransform
            {
                Position = position,
                Rotation = quaternion.identity,
                Scale = 1f
            };
        }
    }
    [BurstCompile]
    private struct MergeParticles : IJobNativeMultiHashMapMergedSharedKeyIndices
    {
        public NativeArray<int> particleIndices;
        //#pragma warning restore 0649



        public void ExecuteFirst(int index)
        {
            particleIndices[index] = index;
        }


        public void ExecuteNext(int cellIndex, int index)
        {
            particleIndices[index] = cellIndex;
        }
    }

    [BurstCompile]
    private partial struct ComputeDensityPressure : IJobEntity
    {
        //#pragma warning disable 0649
        [ReadOnly] public NativeParallelMultiHashMap<int, int> hashMap;
        [ReadOnly] public NativeArray<int> cellOffsetTable;
        [ReadOnly] public NativeArray<LocalTransform> particlesPosition;
        [ReadOnly] public SPHParticleComponent settings;

        public NativeArray<float> densities;
        public NativeArray<float> pressures;
        //#pragma warning restore 0649

        private const float PI = 3.14159274F;
        private const float GAS_CONST = 2000.0f;

        //***** settings 대신에 particle 쓰니까 프레임 나락

        public void Execute(Entity e, [EntityIndexInQuery] int index, in SPHParticleComponent particle)
        {
            // Cache
            int particleCount = particlesPosition.Length;
            float3 position = particlesPosition[index].Position;
            float density = 0.0f;
            int i, hash, j;
            int3 gridOffset;
            int3 gridPosition = GridHash.Quantize(position, settings.radius);
            bool found;

            // Find neighbors
            for (int oi = 0; oi < 27; oi++)
            {
                i = oi * 3;
                gridOffset = new int3(cellOffsetTable[i], cellOffsetTable[i + 1], cellOffsetTable[i + 2]);
                hash = GridHash.Hash(gridPosition + gridOffset);
                NativeParallelMultiHashMapIterator<int> iterator;
                found = hashMap.TryGetFirstValue(hash, out j, out iterator);
                while (found)
                {
                    // Neighbor found, get density
                    float3 rij = particlesPosition[j].Position - position;
                    float r2 = math.lengthsq(rij);

                    if (r2 < settings.smoothingRadiusSq)
                    {
                        density += settings.mass * (315.0f / (64.0f * PI * math.pow(settings.smoothingRadius, 9.0f))) * math.pow(settings.smoothingRadiusSq - r2, 3.0f);
                    }

                    // Next neighbor
                    found = hashMap.TryGetNextValue(out j, ref iterator);
                }
            }

            // Apply density and compute/apply pressure
            densities[index] = density;
            pressures[index] = GAS_CONST * (density - settings.restDensity);

            //===>DirectionToHash GridHash.Hash(GridHash.Quantize(position, settings.radius) + 'Direction(ex : float3(0,1,0))');
            //      found = hashMap.TryGetFirstValue(hash, out j, out iterator); / j : ParticleUnitIndex
        }
    }

    [BurstCompile]
    private partial struct ComputeForces : IJobEntity
    {
        //#pragma warning disable 0649
        [ReadOnly] public NativeParallelMultiHashMap<int, int> hashMap;
        [ReadOnly] public NativeArray<int> cellOffsetTable;
        [ReadOnly] public NativeArray<LocalTransform> particlesPosition;
        [ReadOnly] public NativeArray<SPHVelocityComponent> particlesVelocity;
        [ReadOnly] public NativeArray<float> particlesPressure;
        [ReadOnly] public NativeArray<float> particlesDensity;
        [ReadOnly] public SPHParticleComponent settings;

        public NativeArray<float3> particlesForces;
        //#pragma warning restore 0649

        private const float PI = 3.14159274F;



        public void Execute(Entity e, [EntityIndexInQuery] int index, in SPHParticleComponent particle)
        {
            // Cache
            int particleCount = particlesPosition.Length;
            float3 position = particlesPosition[index].Position;
            float3 velocity = particlesVelocity[index].value;
            float pressure = particlesPressure[index];
            float density = particlesDensity[index];
            float3 forcePressure = new float3(0, 0, 0);
            float3 forceViscosity = new float3(0, 0, 0);
            int i, hash, j;
            int3 gridOffset;
            int3 gridPosition = GridHash.Quantize(position, settings.radius);
            bool found;

            // Physics
            // Find neighbors
            for (int oi = 0; oi < 27; oi++)
            {
                i = oi * 3;
                gridOffset = new int3(cellOffsetTable[i], cellOffsetTable[i + 1], cellOffsetTable[i + 2]);
                hash = GridHash.Hash(gridPosition + gridOffset);
                NativeParallelMultiHashMapIterator<int> iterator;
                found = hashMap.TryGetFirstValue(hash, out j, out iterator);
                while (found)
                {
                    // Neighbor found, get density
                    if (index == j)
                    {
                        found = hashMap.TryGetNextValue(out j, ref iterator);
                        continue;
                    }

                    float3 rij = particlesPosition[j].Position - position;
                    float r2 = math.lengthsq(rij);
                    float r = math.sqrt(r2);

                    if (r < settings.smoothingRadius)
                    {
                        forcePressure += -math.normalize(rij) * settings.mass * (2.0f * pressure) / (2.0f * density) * (-45.0f / (PI * math.pow(settings.smoothingRadius, 6.0f))) * math.pow(settings.smoothingRadius - r, 2.0f);
                        //압력^ , 점성v
                        forceViscosity += settings.viscosity * settings.mass * (particlesVelocity[j].value - velocity) / density * (45.0f / (PI * math.pow(settings.smoothingRadius, 6.0f))) * (settings.smoothingRadius - r);
                    }

                    // Next neighbor
                    found = hashMap.TryGetNextValue(out j, ref iterator);
                }
            }

            //========== SPHParticleComponent에 충돌각(Dot)을 전달해서 중력 영향을... 음수 => 미접촉 , 0 ~ 180 

            // Gravity
            //float3 forceGravity = new float3(0.0f, -9.81f, 0.0f) * density * settings.gravityMult;

            // Apply
            particlesForces[index] = forcePressure + forceViscosity;// + forceGravity;
        }
    }// 유닛간의 충돌구현부분
    [BurstCompile]
    private partial struct Integrate : IJobEntity
    {
        //#pragma warning disable 0649
        [ReadOnly] public NativeArray<float3> particlesForces;
        [ReadOnly] public NativeArray<float> particlesDensity;

        public NativeArray<LocalTransform> particlesPosition;
        public NativeArray<SPHVelocityComponent> particlesVelocity;
        //#pragma warning restore 0649

        private const float DT = 0.0008f;



        public void Execute(Entity e, [EntityIndexInQuery] int index, in SPHParticleComponent particle)
        {
            // Cache
            float3 velocity = particlesVelocity[index].value;
            float3 position = particlesPosition[index].Position;

            // Process
            velocity += DT * particlesForces[index] / particlesDensity[index];
            position += DT * velocity;

            // Apply
            particlesVelocity[index] = new SPHVelocityComponent { value = velocity };
            particlesPosition[index] = new LocalTransform
            {
                Position = position,
                Rotation = quaternion.identity,
                Scale = 1f
            };
        }
    }
    [BurstCompile]
    private partial struct ComputeColliders : IJobEntity
    {
        //#pragma warning disable 0649
        [ReadOnly] public SPHParticleComponent settings;
        [ReadOnly] public NativeArray<SPHColliderComponent> copyColliders;

        public NativeArray<LocalTransform> particlesPosition;
        public NativeArray<SPHVelocityComponent> particlesVelocity;
        //#pragma warning restore 0649



        private static bool Intersect(SPHColliderComponent collider, float3 position, float radius, out float3 penetrationNormal, out float3 penetrationPosition, out float penetrationLength)
        {
            float3 colliderProjection = collider.position - position;

            penetrationNormal = math.cross(collider.right, collider.up);
            penetrationLength = math.abs(math.dot(colliderProjection, penetrationNormal)) - (radius / 2.0f);
            penetrationPosition = collider.position - colliderProjection;

            return penetrationLength < 0.0f
                && math.abs(math.dot(colliderProjection, collider.right)) < collider.scale.x
                && math.abs(math.dot(colliderProjection, collider.up)) < collider.scale.y;
        }



        private static Vector3 DampVelocity(SPHColliderComponent collider, float3 velocity, float3 penetrationNormal, float drag, float bound)
        {
            float3 newVelocity = math.dot(velocity, penetrationNormal) * penetrationNormal * bound
                                + math.dot(velocity, collider.right) * collider.right * drag
                                + math.dot(velocity, collider.up) * collider.up * drag;
            newVelocity = math.dot(newVelocity, new float3(0, 0, 1)) * new float3(0, 0, 1)
                        + math.dot(newVelocity, new float3(1, 0, 0)) * new float3(1, 0, 0)
                        + math.dot(newVelocity, new float3(0, 1, 0)) * new float3(0, 1, 0);
            return newVelocity;
        }



        public void Execute(Entity e, [EntityIndexInQuery] int index, in SPHParticleComponent particle)
        {
            // Cache
            int colliderCount = copyColliders.Length;
            float3 position = particlesPosition[index].Position;
            float3 velocity = particlesVelocity[index].value;

            // Process
            for (int i = 0; i < colliderCount; i++)
            {
                float3 penetrationNormal;
                float3 penetrationPosition;
                float penetrationLength;

                //velocity += (new float3(0, -9.8f, 0) * particle.gravityMult) * 1f;// 여기서 적용이 안되는데??

                if (Intersect(copyColliders[i], position, settings.radius, out penetrationNormal, out penetrationPosition, out penetrationLength))
                {
                    velocity = DampVelocity(copyColliders[i], velocity, penetrationNormal, 1.0f - settings.drag, 1 - particle.bound);
                    position = penetrationPosition - penetrationNormal * math.abs(penetrationLength);

                    {
                        //float3 colliderProjection = math.normalize(copyColliders[i].position - position);
                        //(1 - math.clamp((math.dot(colliderProjection, penetrationNormal) / (Mathf.Deg2Rad * 90)), 0, 1))


                    }
                }
            }
            //============================================================== 충돌노말과 속력노말의 Dot 으로 중력 비례?
            // Apply
            particlesVelocity[index] = new SPHVelocityComponent { value = velocity };
            particlesPosition[index] = new LocalTransform
            {
                Position = position,
                Rotation = quaternion.identity,
                Scale = 1f
            };
        }
    }//==== Disable

    [BurstCompile]
    private partial struct ApplyTranslations : IJobEntity
    {
        [ReadOnly] public NativeArray<LocalTransform> particlesPosition;
        [ReadOnly] public NativeArray<SPHVelocityComponent> particlesVelocity;

        public void Execute(Entity entity, [EntityIndexInQuery] int index, ref LocalTransform translation, ref SPHVelocityComponent sphCollider)
        {
            translation.Position = particlesPosition[index].Position;
            sphCollider.value = particlesVelocity[index].value;

        }

    }

    [BurstCompile]
    private partial struct TempCollisionFloor : IJobEntity
    {
        [ReadOnly] public NativeParallelMultiHashMap<int, int> hashMap;
        [ReadOnly] public NativeArray<LocalTransform> particlesPosition;
        public NativeArray<SPHVelocityComponent> particlesVelocity;

        [ReadOnly] public SPHParticleComponent particle;


        private static Vector3 DampVelocity( float3 velocity, float3 penetrationNormal, float drag, float bound)
        {
            float3 newVelocity = math.dot(velocity, penetrationNormal) * penetrationNormal * bound
                                + math.dot(velocity, new float3(1,0,0)) * new float3(1, 0, 0) * drag
                                + math.dot(velocity, new float3(0, 1, 0)) * new float3(0, 1, 0) * drag;
            newVelocity = math.dot(newVelocity, new float3(0, 0, 1)) * new float3(0, 0, 1)
                        + math.dot(newVelocity, new float3(1, 0, 0)) * new float3(1, 0, 0)
                        + math.dot(newVelocity, new float3(0, 1, 0)) * new float3(0, 1, 0);
            return newVelocity;
        }

        public void Execute(Entity entity, [EntityIndexInQuery] int index, in SPHVelocityComponent sphCollider, in SPHParticleComponent particleCom)
        {
            //translation.Position = particlesPosition[index].Position;
            //sphCollider.value = particlesVelocity[index].value;
            if (particlesPosition[index].Position.y < particle.radius * 0.5f)
            {
                particlesVelocity[index] = new SPHVelocityComponent
                {
                    //value = DampVelocity(sphCollider.value, math.normalizesafe(sphCollider.value), 1 - particleCom.drag, particleCom.bound * -1)

                    value = math.reflect(sphCollider.value, new float3(0, 1, 0)) * (1 - particle.drag) * particle.bound

                    //value = new float3(0, 1, 0) * math.length(sphCollider.value) * math.clamp((1 - particle.drag), 0, 1) * math.clamp((1 - particle.bound), 0, 1)

                    //================== 왜 가라앉지??????? , 서로 밀다보니까?
                };
            }else
            {

                {
                    /*
                    var hashed = GridHash.Hash(GridHash.Quantize(particlesPosition[index].Position, particle.radius) + new int3(0, -1, 0));
                    bool found = hashMap.TryGetFirstValue(hashed, out int j, out var iterator);
                    found = (math.lengthsq(particlesPosition[index].Position - particlesPosition[j].Position) <= particle.smoothingRadius * 2);
                    var moveDot = math.dot(math.normalize((particlesPosition[j].Position - particlesPosition[index].Position)), math.normalize(new float3(0, -9.8f, 0)));
                    float viscosityRate = math.clamp((moveDot / 90), 0, 1);
                    */
                }//접촉 ... 잘못 계산

                #region Disable - 주변접촉하는거에 따라 중력감소하는데 , 덩어리 지면 안떨어짐
                /*
                int i, hash, j;
                int3 gridOffset;
                int3 gridPosition = GridHash.Quantize(particlesPosition[index].Position, particle.radius);
                bool found;
                float grvityRate = 0;

                // Find neighbors
                for (int oi = 0; oi < 27; oi++)
                {
                    i = oi * 3;
                    gridOffset = new int3(cellOffsetTable[i], cellOffsetTable[i + 1], cellOffsetTable[i + 2]);
                    hash = GridHash.Hash(gridPosition + gridOffset);
                    NativeMultiHashMapIterator<int> iterator;
                    found = hashMap.TryGetFirstValue(hash, out j, out iterator);

                    while(found)
                    {
                        // Neighbor found, get density
                        if (index == j)
                        {
                            found = hashMap.TryGetNextValue(out j, ref iterator);
                            continue;
                        }

                        //====================== 거리비교를 해서 접촉중인 유닛들의 ((Dot(아래 , 대상 - 현재유닛위치))의 합 / 90) = 중력비율
                        if (math.lengthsq(particlesPosition[index].Position - particlesPosition[j].Position) <= particle.smoothingRadius * 2)
                        {
                            float moveDot = math.dot(math.normalize((particlesPosition[j].Position - particlesPosition[index].Position)), math.normalize(new float3(0, -9.8f, 0)));
                            grvityRate += (math.clamp(moveDot, 0, 90) / 90);
                        }
                            // Next neighbor
                        found = hashMap.TryGetNextValue(out j, ref iterator);
                    }

                }
                */
                #endregion

                float gravityRate = 0;

                var hashed = GridHash.Hash(GridHash.Quantize(particlesPosition[index].Position, particle.radius) + new int3(0, -1, 0));
                if (hashMap.TryGetFirstValue(hashed, out int j, out var iterator))
                {
                    if ((math.lengthsq(particlesPosition[index].Position - particlesPosition[j].Position) <= particle.smoothingRadius * 2))
                    {
                        var moveDot = math.dot(math.normalize((particlesPosition[j].Position - particlesPosition[index].Position)), math.normalize(new float3(0, -9.8f, 0)));
                        gravityRate += math.clamp((moveDot / 90), 0, 1);
                    }
                }
                hashed = GridHash.Hash(GridHash.Quantize(particlesPosition[index].Position, particle.radius));
                if (hashMap.TryGetFirstValue(hashed, out j, out iterator))
                {
                    if (math.lengthsq(particlesPosition[index].Position - particlesPosition[j].Position) <= (particle.smoothingRadius * particle.smoothingRadius))
                    {
                        var moveDot = math.dot(math.normalize((particlesPosition[j].Position - particlesPosition[index].Position)), math.normalize(new float3(0, -9.8f, 0)));
                        gravityRate += math.clamp(((moveDot * Mathf.Rad2Deg) / 90), 0, 1);
                    }
                }


                //=========== particleCom 쓸때 기다리는것 때문에 프레임 나락감 , 추가안해도 포함 / 주변에 접촉해있는 각도가 ((90 - Dot) 의 합 / 90 ) => viscosityRate

                //+ (new float3(0, -9.8f, 0) * particleCom.gravityMult)
                particlesVelocity[index] = new SPHVelocityComponent
                {
                    value = particlesVelocity[index].value * (1 - particle.drag) + (new float3(0, -9.8f, 0) * particleCom.gravityMult * math.clamp(gravityRate, 0, 1))

                    //value = particlesVelocity[index].value * (1 - particle.drag) + (new float3(0, -9.8f, 0) * particle.gravityMult * 1)//Legacy
                    //충돌감지후 중력에 가중치 부여 (쌓여있을수 있게) / (found ? 0 : 1)
                };
            }

            //ComputeForces의 충돌 계산을 가져와서 진행방향만 충돌 검색해서 멈추게하면 테스트 되니까
            //      가장 가까운 지점이 일정거리면 암튼 ㅇㅇ

            //      position = particlesPosition[index].Position
            //===>DirectionToHash : GridHash.Hash(GridHash.Quantize(position, particleCom.radius) + 'Direction(ex : float3(0,1,0))');
            //      found = hashMap.TryGetFirstValue(hash, out j, out iterator); / j : ParticleUnitIndex


        }

    }

    [BurstCompile]
    private partial struct AddTranslations : IJobEntity
    {
        public float delta;

        public void Execute(Entity entity, [EntityIndexInQuery] int index, ref LocalTransform translation, ref SPHVelocityComponent sphCollider, in SPHParticleComponent particle)
        {
            translation.Position += sphCollider.value * delta;
            //sphCollider.value = particlesVelocity[index].value;
            sphCollider.value -= sphCollider.value * particle.drag;

        }

    }

    #endregion

    //위에 있는건 위치를 지정 하는거 여서 / 속력기반 예상위치를 매프레임 , 위에있는 job은 충돌적용 위치

    protected override void OnCreate()
    {
        //this.Enabled = false;
        //<SPHParticleComponent, SPHVelocityComponent>
        SPHCharacterGroup = GetEntityQuery
        (
            ComponentType.ReadOnly(typeof(SPHParticleComponent)),
            typeof(LocalTransform), typeof(SPHVelocityComponent)
        );
        SPHColliderGroup = GetEntityQuery(ComponentType.ReadOnly(typeof(SPHColliderComponent)));
    }
    protected override void OnStartRunning()
    {
        // Get the colliders
        //colliders = SPHColliderGroup.ToComponentDataArray<SPHColliderComponent>(Allocator.Persistent);
        // out collidersToNativeArrayJobHandle

        if (SystemAPI.HasSingleton<SwitchPhysicsComponent>() == false)
        {
            Enabled = false;
            return;
        }
        Enabled = (SystemAPI.GetSingleton<SwitchPhysicsComponent>().switchType == SwitchSPHtype.customFix);
    }
    protected override void OnUpdate()
    {
        //Schedule 에서 ScheduleParallel 으로 바꾸니 만개 유닛 일때 15 ~ 20 프레임 올라감

        EntityManager.GetAllUniqueSharedComponentsManaged(uniqueTypes);

        //DelayCount += SystemAPI.Time.DeltaTime;

        for (int typeIndex = 1; typeIndex < uniqueTypes.Count; typeIndex++)
        {
            if (true)// 매번 N초 마다 실행 + 값 초기화는 for 외부에서
            {
                // Get the current chunk setting
                SPHParticleComponent settings = uniqueTypes[typeIndex];
                //SPHCharacterGroup.SetFilter(settings);
                //SPHCharacterGroup.AddSharedComponentFilterManaged(settings);

                #region Cache the data
                //JobHandle particlesPositionJobHandle;

                NativeArray<LocalTransform> particlesPosition = SPHCharacterGroup.ToComponentDataArray<LocalTransform>(Allocator.TempJob);//, out particlesPositionJobHandle
                                                                                                                                          //JobHandle particlesVelocityJobHandle;
                NativeArray<SPHVelocityComponent> particlesVelocity = SPHCharacterGroup.ToComponentDataArray<SPHVelocityComponent>(Allocator.TempJob);//, out particlesVelocityJobHandle

                int cacheIndex = typeIndex - 1;
                int particleCount = particlesPosition.Length;

                NativeParallelMultiHashMap<int, int> hashMap = new NativeParallelMultiHashMap<int, int>(particleCount, Allocator.TempJob);

                NativeArray<float3> particlesForces = new NativeArray<float3>(particleCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
                NativeArray<float> particlesPressure = new NativeArray<float>(particleCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
                NativeArray<float> particlesDensity = new NativeArray<float>(particleCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
                NativeArray<int> particleIndices = new NativeArray<int>(particleCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);

                NativeArray<int> cellOffsetTableNative = new NativeArray<int>(cellOffsetTable, Allocator.TempJob);
                #endregion

                #region  Add new or dispose previous particle chunks / 쓰는거 맞아?
                PreviousParticle nextParticles = new PreviousParticle
                {
                    hashMap = hashMap,
                    particlesPosition = particlesPosition,
                    particlesVelocity = particlesVelocity,
                    particlesForces = particlesForces,
                    particlesPressure = particlesPressure,
                    particlesDensity = particlesDensity,
                    particleIndices = particleIndices,
                    cellOffsetTable = cellOffsetTableNative
                };

                if (cacheIndex > previousParticles.Count - 1)
                {
                    previousParticles.Add(nextParticles);
                }
                else
                {
                    previousParticles[cacheIndex].hashMap.Dispose();
                    previousParticles[cacheIndex].particlesPosition.Dispose();
                    previousParticles[cacheIndex].particlesVelocity.Dispose();
                    previousParticles[cacheIndex].particlesForces.Dispose();
                    previousParticles[cacheIndex].particlesPressure.Dispose();
                    previousParticles[cacheIndex].particlesDensity.Dispose();
                    previousParticles[cacheIndex].particleIndices.Dispose();
                    previousParticles[cacheIndex].cellOffsetTable.Dispose();
                }
                previousParticles[cacheIndex] = nextParticles;
                #endregion

                #region  (Job Process 1) Initialize the empty arrays with a default value
                MemsetNativeArray<float> particlesPressureJob = new MemsetNativeArray<float> { Source = particlesPressure, Value = 0.0f };
                JobHandle particlesPressureJobHandle = particlesPressureJob.Schedule(particleCount, 64, Dependency);

                MemsetNativeArray<float> particlesDensityJob = new MemsetNativeArray<float> { Source = particlesDensity, Value = 0.0f };
                JobHandle particlesDensityJobHandle = particlesDensityJob.Schedule(particleCount, 64, Dependency);

                MemsetNativeArray<int> particleIndicesJob = new MemsetNativeArray<int> { Source = particleIndices, Value = 0 };
                JobHandle particleIndicesJobHandle = particleIndicesJob.Schedule(particleCount, 64, Dependency);

                MemsetNativeArray<float3> particlesForcesJob = new MemsetNativeArray<float3> { Source = particlesForces, Value = new float3(0, 0, 0) };
                JobHandle particlesForcesJobHandle = particlesForcesJob.Schedule(particleCount, 64, Dependency);


                #endregion

                #region  (Job Process 2) Put positions into a hashMap
                // hashPositionsJob에서 해쉬맵을 만들고 , particlesPosition에 위치값 넣기
                // mergeParticlesJob은 유닛갯수 크기만큼 particleIndices를 만드는데 , 일정영역 안에 있는건 처음꺼인 해쉬키값으로
                HashPositions hashPositionsJob = new HashPositions
                {
                    positions = particlesPosition,
                    hashMap = hashMap.AsParallelWriter(),
                    cellRadius = settings.radius
                };
                //JobHandle hashPositionsJobHandle = hashPositionsJob.Schedule(particleCount, 64, Dependency);//particlesPositionJobHandle
                JobHandle hashPositionsJobHandle = hashPositionsJob.ScheduleParallel(Dependency);

                JobHandle mergedPositionIndicesJobHandle = JobHandle.CombineDependencies(hashPositionsJobHandle, particleIndicesJobHandle);

                MergeParticles mergeParticlesJob = new MergeParticles
                {
                    particleIndices = particleIndices
                };
                JobHandle mergeParticlesJobHandle = mergeParticlesJob.Schedule(hashMap, 64, mergedPositionIndicesJobHandle);

                JobHandle mergedMergedParticlesDensityPressure = JobHandle.CombineDependencies(mergeParticlesJobHandle, particlesPressureJobHandle, particlesDensityJobHandle);
                #endregion

                #region  (Job Process 3) Compute density pressure
                // 주변 영역중 , 일정범위 안에 있는것들을 질량을 합함  ... 어마?
                ComputeDensityPressure computeDensityPressureJob = new ComputeDensityPressure
                {
                    particlesPosition = particlesPosition,
                    densities = particlesDensity,
                    pressures = particlesPressure,
                    hashMap = hashMap,
                    cellOffsetTable = cellOffsetTableNative,
                    settings = settings
                };
                //JobHandle computeDensityPressureJobHandle = computeDensityPressureJob.Schedule(particleCount, 64, mergedMergedParticlesDensityPressure);
                JobHandle computeDensityPressureJobHandle = computeDensityPressureJob.ScheduleParallel(mergedMergedParticlesDensityPressure);

                JobHandle mergeComputeDensityPressureVelocityForces = JobHandle.CombineDependencies(computeDensityPressureJobHandle, particlesForcesJobHandle);//particlesVelocityJobHandle
                #endregion

                #region  (Job Process 4) Compute forces
                // 주변 영역중 , 일정범위 안에 있는것들을 가속도(속도 / 질량)를 더함
                ComputeForces computeForcesJob = new ComputeForces
                {
                    particlesPosition = particlesPosition,
                    particlesVelocity = particlesVelocity,
                    particlesForces = particlesForces,
                    particlesPressure = particlesPressure,
                    particlesDensity = particlesDensity,
                    cellOffsetTable = cellOffsetTableNative,
                    hashMap = hashMap,
                    settings = settings
                };
                //JobHandle computeForcesJobHandle = computeForcesJob.Schedule(particleCount, 64, mergeComputeDensityPressureVelocityForces);
                JobHandle computeForcesJobHandle = computeForcesJob.ScheduleParallel(mergeComputeDensityPressureVelocityForces);
                #endregion

                #region  (Job Process 5) Integrate
                // 위치, 속력값 적용
                Integrate integrateJob = new Integrate
                {
                    particlesPosition = particlesPosition,
                    particlesVelocity = particlesVelocity,
                    particlesDensity = particlesDensity,
                    particlesForces = particlesForces
                };
                //JobHandle integrateJobHandle = integrateJob.Schedule(particleCount, 64, computeForcesJobHandle);
                JobHandle integrateJobHandle = integrateJob.ScheduleParallel(computeForcesJobHandle);

                JobHandle mergedIntegrateCollider = JobHandle.CombineDependencies(integrateJobHandle, collidersToNativeArrayJobHandle);
                #endregion

                // (Job Process 6) Compute Colliders
                /*
                ComputeColliders computeCollidersJob = new ComputeColliders
                {
                    particlesPosition = particlesPosition,
                    particlesVelocity = particlesVelocity,
                    copyColliders = colliders,
                    settings = settings
                };
                JobHandle computeCollidersJobHandle = computeCollidersJob.Schedule(particleCount, 64, mergedIntegrateCollider);
                */

                var TempColision = new TempCollisionFloor
                {
                    hashMap = hashMap,
                    particlesPosition = particlesPosition,
                    particlesVelocity = particlesVelocity,
                    particle = settings
                };
                JobHandle TempCollisionHandle = TempColision.ScheduleParallel(mergedIntegrateCollider);

                // (Job Process 7) Apply translations and velocities
                ApplyTranslations applyTranslationsJob = new ApplyTranslations
                {
                    particlesPosition = particlesPosition,
                    particlesVelocity = particlesVelocity
                };
                JobHandle applyTranslationsJobHandle = applyTranslationsJob.ScheduleParallel(TempCollisionHandle);//computeCollidersJobHandle
                applyTranslationsJobHandle.Complete();//LocalTransForm을 쓰는거여서 Complete() 필요

                this.Dependency = applyTranslationsJobHandle;
            }
            else
            {
                AddTranslations addTranslationsJob = new AddTranslations
                {
                    delta = SystemAPI.Time.DeltaTime
                };
                JobHandle addTranslationsHandle = addTranslationsJob.ScheduleParallel(Dependency);
                addTranslationsHandle.Complete();
            }
            
        }

        //Note - 매프레임마다 실행되어서 비용증가 , 대부분을 기다리는데 사용 그다음이 'ComputeDesityPressure'
        //      ++ 영역 단위로 뭉쳐서 계산하면 더 빨라짐 / 만개 기준 200FPS 이상 가능할듯
        //      

        //if (DelayCount > Delay)
        {
            //DelayCount = 0;
        }
        //=============== Integrate에서 particlesVelocity 으로 위치 계산 하던데
        //=============== 이걸 이용해서 계산만 주기적으로 실행(매번X)

        uniqueTypes.Clear();
    }

    protected override void OnStopRunning()
    {
        //EntityManager.CompleteAllJobs();
        EntityManager.CompleteAllTrackedJobs();

        for (int i = 0; i < previousParticles.Count; i++)
        {
            previousParticles[i].hashMap.Dispose();
            previousParticles[i].particlesPosition.Dispose();
            previousParticles[i].particlesVelocity.Dispose();
            previousParticles[i].particlesForces.Dispose();
            previousParticles[i].particlesPressure.Dispose();
            previousParticles[i].particlesDensity.Dispose();
            previousParticles[i].particleIndices.Dispose();
            previousParticles[i].cellOffsetTable.Dispose();
        }

        //colliders.Dispose();

        previousParticles.Clear();
    }
}
