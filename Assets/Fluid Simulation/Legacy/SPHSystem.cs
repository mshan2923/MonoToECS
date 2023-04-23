using System.Collections.Generic;
using UnityEngine;
using Unity.Collections;
using Unity.Transforms;
using Unity.Mathematics;
using Unity.Entities;
using Unity.Burst;
using Unity.Jobs;
using Samples.Boids;

[UpdateBefore(typeof(TransformSystemGroup))]
public partial class SPHSystem : SystemBase
{
    private EntityQuery SPHCharacterGroup;
    private EntityQuery SPHColliderGroup;

    private JobHandle collidersToNativeArrayJobHandle;
    private NativeArray<SPHColliderComponent> colliders;

    private Transform cameraTransform;

    private List<SPHParticleComponent> uniqueTypes = new List<SPHParticleComponent>(10);
    private List<PreviousParticle> previousParticles = new List<PreviousParticle>();// 어디에.. 쓰는거지?

    private static readonly int[] cellOffsetTable =
    {
        1, 1, 1, 1, 1, 0, 1, 1, -1, 1, 0, 1, 1, 0, 0, 1, 0, -1, 1, -1, 1, 1, -1, 0, 1, -1, -1,
        0, 1, 1, 0, 1, 0, 0, 1, -1, 0, 0, 1, 0, 0, 0, 0, 0, -1, 0, -1, 1, 0, -1, 0, 0, -1, -1,
        -1, 1, 1, -1, 1, 0, -1, 1, -1, -1, 0, 1, -1, 0, 0, -1, 0, -1, -1, -1, 1, -1, -1, 0, -1, -1, -1
    };

    private struct PreviousParticle
    {
        //#pragma warning disable 0649
        public NativeParallelMultiHashMap<int, int> hashMap;//NativeMultiHashMap<int, int> hashMap;
        public NativeArray<LocalTransform> particlesPosition;
        public NativeArray<SPHVelocityComponent> particlesVelocity;
        public NativeArray<float3> particlesForces;
        public NativeArray<float> particlesPressure;
        public NativeArray<float> particlesDensity;
        public NativeArray<int> particleIndices;

        public NativeArray<int> cellOffsetTable;
        //#pragma warning restore 0649
    }
    // 초기화 / hashMap 초기화 --> <GridHash.Hash(), cellRadius>
    [BurstCompile]
    private struct HashPositions : IJobParallelFor
    {
        //#pragma warning disable 0649
        [ReadOnly] public float cellRadius;

        public NativeArray<LocalTransform> positions;
        public NativeParallelMultiHashMap<int, int>.ParallelWriter hashMap;//NativeMultiHashMap<int, int>.ParallelWriter hashMap;
        
        
        //#pragma warning restore 0649

        public void Execute(int index)
        {
            float3 position = positions[index].Position;

            int hash = GridHash.Hash(position, cellRadius);
            hashMap.Add(hash, index);

            positions[index] = new LocalTransform { Position = position , Rotation = quaternion.identity, Scale = 1};
        }
    }

    //Merge : 병합 
    [BurstCompile]
    private struct MergeParticles : Samples.Boids.IJobNativeMultiHashMapMergedSharedKeyIndices
    {
        public NativeArray<int> particleIndices;

        // 키가 생길때
        public void ExecuteFirst(int index)
        {
            particleIndices[index] = index;
        }
        // 키가 있을때 , cellIndex == firstIndex
        public void ExecuteNext(int cellIndex, int index)
        {
            particleIndices[index] = cellIndex;
        }

        //#pragma warning restore 0649
        //FIXME - 
    }//==============엄....없는디?

    // 밀도 계산? , 밀도 높아지면 서로 밀치는건가?
      [BurstCompile]
    private struct ComputeDensityPressure : IJobParallelFor
    {
        //#pragma warning disable 0649
        [ReadOnly] public NativeParallelMultiHashMap<int, int> hashMap;//NativeMultiHashMap<int, int> hashMap;
        [ReadOnly] public NativeArray<int> cellOffsetTable;
        [ReadOnly] public NativeArray<LocalTransform> particlesPosition;
        [ReadOnly] public SPHParticleComponent settings;

        public NativeArray<float> densities;
        public NativeArray<float> pressures;
        //#pragma warning restore 0649

        private const float PI = 3.14159274F;
        private const float GAS_CONST = 2000.0f;

        

        public void Execute(int index)
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

                NativeParallelMultiHashMapIterator<int> iterator;//NativeMultiHashMapIterator<int> iterator;
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
        }
    }
    
    // Force 계산후 적용
     [BurstCompile]
    private struct ComputeForces : IJobParallelFor
    {
        //#pragma warning disable 0649
        [ReadOnly] public NativeParallelMultiHashMap<int, int> hashMap;//NativeMultiHashMap<int, int> hashMap;
        [ReadOnly] public NativeArray<int> cellOffsetTable;
        [ReadOnly] public NativeArray<LocalTransform> particlesPosition;
        [ReadOnly] public NativeArray<SPHVelocityComponent> particlesVelocity;
        [ReadOnly] public NativeArray<float> particlesPressure;
        [ReadOnly] public NativeArray<float> particlesDensity;
        [ReadOnly] public SPHParticleComponent settings;

        public NativeArray<float3> particlesForces;
        //#pragma warning restore 0649

        private const float PI = 3.14159274F;



        public void Execute(int index)
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
                NativeParallelMultiHashMapIterator<int> iterator;//NativeMultiHashMapIterator<int> iterator;
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

                        forceViscosity += settings.viscosity * settings.mass * (particlesVelocity[j].value - velocity) / density * (45.0f / (PI * math.pow(settings.smoothingRadius, 6.0f))) * (settings.smoothingRadius - r);
                    }

                    // Next neighbor
                    found = hashMap.TryGetNextValue(out j, ref iterator);
                }
            }

            // Gravity
            float3 forceGravity = new float3(0.0f, -9.81f, 0.0f) * density * settings.gravityMult;

            // Apply
            particlesForces[index] = forcePressure + forceViscosity + forceGravity;
        }
    }

    //위치와 속력 계산후 적용
    // 속력 : 힘 / 질량 , 위치 += 속력
     [BurstCompile]
    private struct Integrate : IJobParallelFor//Integrate : 통합하다
    {
        #pragma warning disable 0649
        [ReadOnly] public NativeArray<float3> particlesForces;
        [ReadOnly] public NativeArray<float> particlesDensity;

        public NativeArray<LocalTransform> particlesPosition;
        public NativeArray<SPHVelocityComponent> particlesVelocity;
        #pragma warning restore 0649

        private const float DT = 0.0008f;



        public void Execute(int index)
        {
            // Cache
            float3 velocity = particlesVelocity[index].value;
            float3 position = particlesPosition[index].Position;

            // Process
            velocity += DT * particlesForces[index] / particlesDensity[index];
            position += DT * velocity;

            // Apply
            particlesVelocity[index] = new SPHVelocityComponent { value = velocity };
            particlesPosition[index] = new LocalTransform { Position = position, Rotation = quaternion.identity, Scale = 1 };
        }
    }

    //위치와 속력 계산후 적용
    [BurstCompile]
    private struct ComputeColliders : IJobParallelFor
    {
        #pragma warning disable 0649
        [ReadOnly] public SPHParticleComponent settings;
        [ReadOnly] public NativeArray<SPHColliderComponent> copyColliders;

        public NativeArray<LocalTransform> particlesPosition;
        public NativeArray<SPHVelocityComponent> particlesVelocity;
        #pragma warning restore 0649

        private const float BOUND_DAMPING = -0.5f;



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



        private static Vector3 DampVelocity(SPHColliderComponent collider, float3 velocity, float3 penetrationNormal, float drag)
        {
            float3 newVelocity = math.dot(velocity, penetrationNormal) * penetrationNormal * BOUND_DAMPING
                                + math.dot(velocity, collider.right) * collider.right * drag
                                + math.dot(velocity, collider.up) * collider.up * drag;
            newVelocity = math.dot(newVelocity, new float3(0, 0, 1)) * new float3(0, 0, 1)
                        + math.dot(newVelocity, new float3(1, 0, 0)) * new float3(1, 0, 0)
                        + math.dot(newVelocity, new float3(0, 1, 0)) * new float3(0, 1, 0);
            return newVelocity;
        }



        public void Execute(int index)
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
                if (Intersect(copyColliders[i], position, settings.radius, out penetrationNormal, out penetrationPosition, out penetrationLength))
                {
                    velocity = DampVelocity(copyColliders[i], velocity, penetrationNormal, 1.0f - settings.drag);
                    position = penetrationPosition - penetrationNormal * math.abs(penetrationLength);
                }
            }

            // Apply
            particlesVelocity[index] = new SPHVelocityComponent { value = velocity };
            particlesPosition[index] = new LocalTransform { Position = position, Rotation = quaternion.identity, Scale = 1 };
        }
    }
    [BurstCompile]
    private struct ApplyTranslations : IJobParallelFor//IJobForEachWithEntity<LocalTransform, SPHVelocityComponent>
    {
        [ReadOnly] public NativeArray<LocalTransform> particlesPosition;
        [ReadOnly] public NativeArray<SPHVelocityComponent> particlesVelocity;

        LocalTransform trans;

        public void Execute(Entity entity, int index, ref LocalTransform translation, ref SPHVelocityComponent sphCollider)
        {
            translation = new LocalTransform { Position = particlesPosition[index].Position };
            sphCollider = new SPHVelocityComponent { value = particlesVelocity[index].value };
            //trans = particlesPosition[index];
        }

        public void Execute(int index)
        {
            throw new System.NotImplementedException();
        }
    }//========= 
    //FIXME - ApplyTranslations

    protected override void OnCreate()
    {
        //base.OnCreate();
        //Import
        SPHCharacterGroup = GetEntityQuery(
            ComponentType.ReadOnly(typeof(SPHParticleComponent)),
            typeof(LocalTransform), typeof(SPHVelocityComponent));
        SPHColliderGroup = GetEntityQuery(ComponentType.ReadOnly(typeof(SPHColliderComponent)));

        this.Enabled = false;
    }
    protected override void OnStartRunning()
    {
        //base.OnStartRunning();
        // Get the colliders
        //        colliders = SPHColliderGroup.ToComponentDataArray<SPHCollider>(Allocator.Persistent, out collidersToNativeArrayJobHandle);
        colliders = SPHColliderGroup.ToComponentDataListAsync<SPHColliderComponent>(Allocator.Persistent, out collidersToNativeArrayJobHandle).AsArray();
    }
    protected override void OnUpdate()
    {
            if (cameraTransform == null)
            cameraTransform = GameObject.Find("Main Camera").transform;//이건.. 왜??

            //공유 구성 요소 유형의 모든 고유 인스턴스 목록을 가져옵니다.
            EntityManager.GetAllUniqueSharedComponentsManaged(uniqueTypes);

            for (int typeIndex = 1; typeIndex < uniqueTypes.Count; typeIndex++)
            {
                //먼저 이전에 생성한 모든 NativeArray를 구조에 넣은 다음 목록에 넣습니다.
                // 이를 통해 고유한 공유 구성 요소가 여러 개 있는 경우 이전의 Native Array 세트를 폐기할 수 있습니다.
                // Get the current chunk setting
                SPHParticleComponent settings = uniqueTypes[typeIndex];
                //SPHCharacterGroup.SetFilter(settings);
                SPHCharacterGroup.SetSharedComponentFilterManaged(settings);
                

                // Cache the data / ToComponentDataArray를 대신... 
                JobHandle particlesPositionJobHandle;
                NativeArray<LocalTransform> particlesPosition = SPHCharacterGroup.ToComponentDataListAsync<LocalTransform>(Allocator.TempJob, out particlesPositionJobHandle).AsArray();
                JobHandle particlesVelocityJobHandle;
                NativeArray<SPHVelocityComponent> particlesVelocity = SPHCharacterGroup.ToComponentDataListAsync<SPHVelocityComponent>(Allocator.TempJob, out particlesVelocityJobHandle).AsArray();

                        int cacheIndex = typeIndex - 1;
                int particleCount = particlesPosition.Length;

                NativeParallelMultiHashMap<int, int> hashMap = new NativeParallelMultiHashMap<int, int>(particleCount, Allocator.TempJob);
                //  NativeMultiHashMap<int, int> hashMap = new NativeMultiHashMap<int, int>(particleCount, Allocator.TempJob);

            NativeArray<float3> particlesForces = new NativeArray<float3>(particleCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
                NativeArray<float> particlesPressure = new NativeArray<float>(particleCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
                NativeArray<float> particlesDensity = new NativeArray<float>(particleCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
                NativeArray<int> particleIndices = new NativeArray<int>(particleCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);

                NativeArray<int> cellOffsetTableNative = new NativeArray<int>(cellOffsetTable, Allocator.TempJob);

                // Add new or dispose previous particle chunks
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
                
#region  Job
                // Initialize the empty arrays with a default value , 값 덮어쓰기
                MemsetNativeArray<float> particlesPressureJob = new MemsetNativeArray<float> { Source = particlesPressure, Value = 0.0f };
                JobHandle particlesPressureJobHandle = particlesPressureJob.Schedule(particleCount, 64);//inputDeps

                MemsetNativeArray<float> particlesDensityJob = new MemsetNativeArray<float> { Source = particlesDensity, Value = 0.0f };
                JobHandle particlesDensityJobHandle = particlesDensityJob.Schedule(particleCount, 64);

                MemsetNativeArray<int> particleIndicesJob = new MemsetNativeArray<int> { Source = particleIndices, Value = 0 };
                JobHandle particleIndicesJobHandle = particleIndicesJob.Schedule(particleCount, 64);

                MemsetNativeArray<float3> particlesForcesJob = new MemsetNativeArray<float3> { Source = particlesForces, Value = new float3(0, 0, 0) };
                JobHandle particlesForcesJobHandle = particlesForcesJob.Schedule(particleCount, 64);

                // Put positions into a hashMap
                HashPositions hashPositionsJob = new HashPositions
                {
                    positions = particlesPosition,
                    hashMap = hashMap.AsParallelWriter(),
                    cellRadius = settings.radius
                };
                //particlePosition 이 완료되고 실행
                JobHandle hashPositionsJobHandle = hashPositionsJob.Schedule(particleCount, 64, particlesPositionJobHandle);
                //이걸쓰는 job이 hashPositionJob 과 particleIndicesJob 끝나야 실행되게
                JobHandle mergedPositionIndicesJobHandle = JobHandle.CombineDependencies(hashPositionsJobHandle, particleIndicesJobHandle);

                MergeParticles mergeParticlesJob = new MergeParticles
                {
                    particleIndices = particleIndices
                };

                //이 작업의 목적은 각 입자에 hashMap 버킷의 ID를 부여하는 것입니다.
                JobHandle mergeParticlesJobHandle = mergeParticlesJob.Schedule(hashMap, 64, mergedPositionIndicesJobHandle);
                //  MergeParticles 가 HashMap만들게 / using Samples.Boids 사용
                JobHandle mergedMergedParticlesDensityPressure = JobHandle.CombineDependencies(mergeParticlesJobHandle, particlesPressureJobHandle, particlesDensityJobHandle);
                // 입자간 충돌 사전 작업완료

                // Compute density pressure
                ComputeDensityPressure computeDensityPressureJob = new ComputeDensityPressure
                {
                    particlesPosition = particlesPosition,
                    densities = particlesDensity,
                    pressures = particlesPressure,
                    hashMap = hashMap,
                    cellOffsetTable = cellOffsetTableNative,
                    settings = settings
                };
                JobHandle computeDensityPressureJobHandle = computeDensityPressureJob.Schedule(particleCount, 64, mergedMergedParticlesDensityPressure);
                JobHandle mergeComputeDensityPressureVelocityForces = JobHandle.CombineDependencies(computeDensityPressureJobHandle, particlesForcesJobHandle, particlesVelocityJobHandle);

                // Compute forces
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
                JobHandle computeForcesJobHandle = computeForcesJob.Schedule(particleCount, 64, mergeComputeDensityPressureVelocityForces);

                // Integrate
                Integrate integrateJob = new Integrate
                {
                    particlesPosition = particlesPosition,
                    particlesVelocity = particlesVelocity,
                    particlesDensity = particlesDensity,
                    particlesForces = particlesForces
                };
                JobHandle integrateJobHandle = integrateJob.Schedule(particleCount, 64, computeForcesJobHandle);
                //벽 충돌을 해결하고 입자 위치를 구성 요소 위치로 다시 적용하는 것으로 작업 스케줄링을 마칩니다. 그런 다음 루프가 계속됩니다. 

                JobHandle mergedIntegrateCollider = JobHandle.CombineDependencies(integrateJobHandle, collidersToNativeArrayJobHandle);

                // Compute Colliders
                ComputeColliders computeCollidersJob = new ComputeColliders
                {
                    particlesPosition = particlesPosition,
                    particlesVelocity = particlesVelocity,
                    copyColliders = colliders,
                    settings = settings
                };
                JobHandle computeCollidersJobHandle = computeCollidersJob.Schedule(particleCount, 64, mergedIntegrateCollider);

                            // Apply translations and velocities
                ApplyTranslations applyTranslationsJob = new ApplyTranslations
                {
                    particlesPosition = particlesPosition,
                    particlesVelocity = particlesVelocity                    
                };
                //FIXME - JobHandle applyTranslationsJobHandle = applyTranslationsJob.Schedule(this, computeCollidersJobHandle);
                //JobHandle applyTranslationsJobHandle = applyTranslationsJob.Schedule(particlesPosition.Length, 1, computeCollidersJobHandle);
                
                //FIXME - inputDeps = applyTranslationsJobHandle;
                //Dependency = applyTranslationsJobHandle;
                Dependency = computeCollidersJobHandle;

                //작업 구조는 단일 스레드 워크플로우에서 호출한 방법과 크게 다르지 않습니다.
                // 작업을 예약하면 Unity가 각 파티클에 대해 Execute(index) 메서드를 호출합니다. 컴퓨팅 셰이더와 매우 유사합니다.
                // 하지만 몇 가지 짚고 넘어가야 할 것들이 있다. 처음에 [BurstCompile]을 추가하는 것을 잊지 마십시오.
                // 작업 실행 속도가 최대 10배 빨라집니다. 읽기 전용 값을 [ReadOnly](읽기 전용)으로 표시했습니다(필요한 경우 [WriteOnly](쓰기 전용)도 있습니다).
#endregion      
                
            }

        uniqueTypes.Clear();
        //return inputDeps;
    }

        protected override void OnStopRunning()
    {
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

        colliders.Dispose();

        previousParticles.Clear();
    }
}
