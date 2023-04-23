using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;
using Random = Unity.Mathematics.Random;

namespace FluidSimulate
{
    [UpdateAfter(typeof(SPHManagerSystem))]
    public partial class FluidSimlationSystem : SystemBase
    {
        #region Job

        [BurstCompile]
        partial struct PositionSetup : IJobEntity
        {
            //public NativeArray<FluidSimlationComponent> particleData;
            public void Execute([EntityIndexInQuery]int index, in LocalTransform transform, ref FluidSimlationComponent data)
            {
                //var temp = particleData[index];
                //temp.position = transform.Position;

                //particleData[index] = temp;

                data.position = transform.Position;
                //data.position = new Vector3(transform.Position.x, transform.Position.y, transform.Position.z);
            }
        }//처음에 스폰된 위치 적용

        [BurstCompile]
        partial struct ResetAcc : IJobEntity
        {
            [WriteOnly] public NativeArray<FluidSimlationComponent> particleData;
            public ParticleParameterComponent parameter;
            public Vector3 AccVaule;

            public void Execute([EntityIndexInQuery]int index, in FluidSimlationComponent data)
            {
                var temp = data;
                temp.acc = parameter.Gravity + AccVaule;

                particleData[index] = temp;
                //FluidSimlationComponent값을 계산 끝나고 적용되니
            }
        }//Acc 초기화

        [BurstCompile]
        struct ComputePressure : IJobParallelFor
        {

            [ReadOnly] public NativeArray<FluidSimlationComponent> particleData;
            public NativeArray<Vector3> pressureDir;
            public NativeArray<float> moveRes;
            public float Amount;

            public ParticleParameterComponent parameter;

            //============== Particleparameter(반지름, 접촉 감쇄량 등)

            public void Execute(int index)
            {
                for (int j = 0; j < Amount; j++)
                {
                    var ij = particleData[j].position - particleData[index].position;//particleData[index].position - particleData[j].position;
                    var rad = parameter.ParticleRadius + parameter.SmoothRadius;
                    if (ij.sqrMagnitude <= rad * rad)
                    {
                        pressureDir[index] += ij;
                        moveRes[index] += Mathf.Clamp01(Vector3.Dot(-ij.normalized, -(particleData[index].velocity + particleData[index].acc * parameter.DT)));
                    }
                }
            }
        }//압력방향 , 이동저항력 계산

        [BurstCompile]
        struct ComputeFloorCollision : IJobParallelFor
        {
            public NativeArray<FluidSimlationComponent> particleData;

            public ParticleParameterComponent parameter;

            public void Execute(int index)
            {
                var temp = particleData[index];

                if (particleData[index].position.y <= parameter.ParticleRadius * 0.5f)
                {
                    if (particleData[index].isGround == false)
                    {
                        temp.velocity = Vector3.Reflect(temp.velocity, Vector3.up)
                            * (1 - parameter.ParticleViscosity);
                    }
                    var AccSpeed = temp.acc.magnitude;
                    temp.acc.y = 0;
                    temp.acc = temp.acc.normalized * AccSpeed;

                    temp.isGround = true;
                    // 아래로 내려가지 않도록 y 방향 제거
                }else
                {
                    temp.isGround = false;
                }

                particleData[index] = temp;
            }
        }

        [BurstCompile]
        struct ComputeCollision : IJobParallelFor
        {
            public NativeArray<FluidSimlationComponent> particleData;
            public NativeArray<Vector3> pressureDir;
            public NativeArray<float> moveRes;
            public float Amount;

            public ParticleParameterComponent parameter;

            public void Execute(int index)
            {
                {
                    var temp = particleData[index];

                    if (Mathf.Approximately(pressureDir[index].sqrMagnitude, 0))
                    {
                        if (particleData[index].isGround)
                        {
                            temp.velocity *= 1 - (parameter.ParticleDrag * parameter.DT);
                        }
                    }
                    else
                    {
                        float CollisionAcc = Mathf.Max(1 - parameter.ParticleViscosity, 0);
                        if (particleData[index].isGround)
                        {
                            if (moveRes[index] >= 0)
                            {
                                var reflected = Vector3.Reflect(temp.velocity, pressureDir[index].normalized) * CollisionAcc;
                                reflected.y = Mathf.Max(reflected.y, 0);

                                temp.velocity = reflected.normalized * reflected.magnitude;
                            }
                            else
                            {
                                var reflected = Vector3.Reflect(-temp.velocity, pressureDir[index].normalized) * CollisionAcc;
                                reflected.y = Mathf.Max(reflected.y, 0);

                                temp.velocity = reflected.normalized * reflected.magnitude;
                            }
                        }
                        else
                        {
                            var CollisionRate = (parameter.ParticleRadius - pressureDir[index].magnitude) / parameter.ParticleRadius;
                            temp.velocity += parameter.Evaluate(CollisionRate) *
                                 parameter.CollisionPush * pressureDir[index].normalized;

                            if (Mathf.Abs(moveRes[index]) > 0.1f)
                            {
                                //var reflectVel = temp.velocity * (1 - parameter.ParticleViscosity);// Legacy
                                var reflectVel = temp.velocity * (1 - parameter.ParticleViscosity);
                                //      (parameter.DT * CollisionAcc * temp.velocity);

                                if (moveRes[index] >= 0)
                                {
                                    temp.velocity = Vector3.Reflect((reflectVel + temp.acc * parameter.DT), pressureDir[index].normalized);
                                }
                                else
                                {
                                    temp.velocity = Vector3.Reflect((-reflectVel + temp.acc * parameter.DT), pressureDir[index].normalized);
                                }
                            }
                        }
                    }

                    particleData[index] = temp;
                    
                }//
            }
        }

        [BurstCompile]
        partial struct AddPosition : IJobEntity
        {
            public NativeArray<FluidSimlationComponent> particleData;
            public ParticleParameterComponent parameter;

            public void Execute([EntityIndexInQuery] int index, ref FluidSimlationComponent data)//, in LocalTransform transform
            {
                var acc = particleData[index].acc;

                if (particleData[index].isGround)
                {
                    //data.acc -= parameter.Gravity; //=========== 계속 Acc가 쌓임
                    acc -= parameter.Gravity;
                }
                data.velocity = particleData[index].velocity + acc * parameter.DT;
                //if ()
                
                if (float.IsNaN(particleData[index].velocity.x) || float.IsNaN(particleData[index].velocity.y) || float.IsNaN(particleData[index].velocity.z))
                {
                    //이동을 파업했어....
                }else
                {
                    data.position += particleData[index].velocity * parameter.DT;
                }

                data.acc = Vector3.zero;
            }
        }
        [BurstCompile]
        partial struct ApplyPosition : IJobEntity
        {

            public void Execute([EntityIndexInQuery] int index, ref LocalTransform transform, in FluidSimlationComponent data)
            {
                transform.Position = data.position;
            }
        }

        #endregion

        private EntityQuery ParticleGroup;
        //NativeArray<FluidSimlationComponent> particle;

        ParticleParameterComponent Parameter;
        //float DT = 0.0083333f;
        //static Vector3 GRAVITY = new Vector3(0.0f, -9.81f, 0.0f);
        JobHandle PositionSetupHandle;
        bool isReady = false;
        float timer = 0;

        protected override void OnCreate()
        {
            ParticleGroup = GetEntityQuery(typeof(FluidSimlationComponent), typeof(LocalTransform));

        }
        protected override void OnStartRunning()
        {

            //====--------------> particleData의 위치 설정
            if (SystemAPI.HasSingleton<ParticleParameterComponent>())
                Parameter = SystemAPI.GetSingleton<ParticleParameterComponent>();
            else
                Enabled = false;

            //particle = ParticleGroup.ToComponentDataListAsync<>(Allocator.Persistent)
            // -------------------------- 나중에 장애물 추가 구현
            isReady = false;

            if (Parameter.simulationType != SimulationType.ECS)
            {
                Enabled = false;
            }
        }
        protected override void OnUpdate()
        {
            if (isReady == false)
            {
                PositionSetup PositionSetupJob = new PositionSetup
                {
                    //particleData = ParticleGroup.ToComponentDataArray<FluidSimlationComponent>(Allocator.TempJob)
                };
                PositionSetupHandle = PositionSetupJob.ScheduleParallel(ParticleGroup, Dependency);
                PositionSetupHandle.Complete();

                var tempData = ParticleGroup.ToComponentDataArray<FluidSimlationComponent>(Allocator.Temp);
                if (tempData.Length <= 0)
                {
                    tempData.Dispose();
                    return;
                }

                isReady = true;


                tempData.Dispose();
                return;
            }

            if (timer > Parameter.DT)
            {
                timer = 0;
                return;
            }else
            {
                timer += SystemAPI.Time.DeltaTime;
            }

            NativeArray<FluidSimlationComponent> particleData =
                ParticleGroup.ToComponentDataArray<FluidSimlationComponent>(Allocator.TempJob);

            int particleCount = particleData.Length;

            var particleDir = new NativeArray<Vector3>(particleCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            var particleMoveRes = new NativeArray<float>(particleCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);

            //-------

            var particleDirJob = new MemsetNativeArray<Vector3> { Source = particleDir, Value = Vector3.zero};
            JobHandle particleDirJobHandle = particleDirJob.Schedule(particleCount, 64);
            var particleMoveResJob = new MemsetNativeArray<float> { Source = particleMoveRes, Value = 0 };
            JobHandle particleMoveResJobHandle = particleMoveResJob.Schedule(particleCount, 64);

            JobHandle SetupMergedHandle = JobHandle.CombineDependencies(PositionSetupHandle, particleDirJobHandle, particleMoveResJobHandle);
            //------

            ResetAcc ResetAccJob = new ResetAcc
            {
                particleData = particleData,
                parameter = Parameter,
                AccVaule = Vector3.zero
            };
            JobHandle ResetAccHandle = ResetAccJob.ScheduleParallel(ParticleGroup, SetupMergedHandle);

            //ResetAccHandle.Complete();//------------- 추가하니 왜 프레임 증가? / 이젠 이것때문에 프레임 감소 (기다리는것 때문)
            Debugging(particleData, particleDir, particleMoveRes, "Reset");

            ComputePressure computePressureJob = new ComputePressure
            {
                particleData = particleData,
                pressureDir = particleDir,
                moveRes = particleMoveRes,
                Amount = particleCount,
                parameter = Parameter
            };
            JobHandle computePressureHandle = computePressureJob.Schedule(particleCount, 64, ResetAccHandle);

            ComputeFloorCollision FloorCollisionJob = new ComputeFloorCollision
            {
                particleData = particleData,
                parameter = Parameter
            };
            JobHandle FloorCollisionHandle = FloorCollisionJob.Schedule(particleCount, 64, computePressureHandle);
            
            ComputeCollision ComputeCollisionJob = new ComputeCollision
            {
                particleData = particleData,
                pressureDir = particleDir,
                moveRes = particleMoveRes,
                Amount = particleCount,
                parameter = Parameter
            };
            JobHandle ComputeCollisionHandle = ComputeCollisionJob.Schedule(particleCount, 64, FloorCollisionHandle);

            ComputeCollisionHandle.Complete();
            Debugging(particleData, particleDir, particleMoveRes, "Before AddPos");
            

            AddPosition AddPositionJob = new AddPosition
            {
                particleData = particleData,
                parameter = Parameter
            };
            JobHandle AddPositionHandle = AddPositionJob.ScheduleParallel(ParticleGroup, ComputeCollisionHandle);
            AddPositionHandle.Complete();// ------ 없으면 에러

            ApplyPosition ApplyPositionJob = new() { };
            //JobHandle ApplyPositionHandle = ApplyPositionJob.ScheduleParallel(ParticleGroup, AddPositionHandle);
            ApplyPositionJob.ScheduleParallel(ParticleGroup);

            //------
            //ApplyPositionHandle.Complete();

            //Dependency = ApplyPositionHandle;

            Debugging(particleData, particleDir, particleMoveRes,"End");

            particleData.Dispose();
            particleDir.Dispose();
            particleMoveRes.Dispose();
        }

        void Debugging(NativeArray<FluidSimlationComponent> particleData, NativeArray<Vector3> dir, NativeArray<float> res, string vaule = "")
        {
            /*
            if (particleData.Length > 0)
            {
                var tempData = ParticleGroup.ToComponentDataArray<FluidSimlationComponent>(Allocator.TempJob);
                var temp = tempData.ToArray()[0];
                var temp2 = particleData.ToArray()[0];
                var IsNan = (float.IsNaN(temp2.velocity.x) || float.IsNaN(temp2.velocity.y) || float.IsNaN(temp2.velocity.z));
                Debug.Log(vaule + $" | Component  Pos : {temp.position} / Vel : {temp.velocity} / Acc : {temp.acc} " +
                    $"\nData  Pos : {temp2.position} / Vel :{temp2.velocity} / Acc : {temp2.acc}" +
                    $"\n Dir : {dir.ToArray()[0]} / Move Res : {res.ToArray()[0]} / Data Vel is : {(IsNan ? "Nan" : "Enable")}");

                //(float.IsNaN(data.velocity.x) || float.IsNaN(data.velocity.y) || float.IsNaN(data.velocity.z))
            }
            else
            {
                Debug.Log("particleData is null");
            }*/
        }
        //Disabled
    }
}
