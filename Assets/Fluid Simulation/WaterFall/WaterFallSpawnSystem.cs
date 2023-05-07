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

namespace FluidSimulate.WaterFall
{
    public partial class WaterFallSpawnSystem : SystemBase
    {
        BeginInitializationEntityCommandBufferSystem IntiECB;
        EntityQuery ParticlesQuery;

        float Timer = 0f;
        NativeArray<Entity> entities;

        enum SpawnWorkType { Inti , Reset, Spawning};
        SpawnWorkType SpawnWork = SpawnWorkType.Inti;

        NativeParallelMultiHashMap<int, Entity> DisabledEntity;

        #region Job

        partial struct SetEnableAll : IJobEntity
        {
            public EntityCommandBuffer.ParallelWriter ECB;
            public NativeArray<Entity> entities;
            public bool Vaule;
            public void Execute([EntityIndexInQuery] int index)
            {
                ECB.SetEnabled(index, entities[index], Vaule);
            }
        }//������ ��� ��Ȱ��ȭ �Ҷ� ���

        partial struct GetDisableEntity : IJobParallelFor
        {
            [Unity.Collections.LowLevel.Unsafe.NativeDisableUnsafePtrRestriction]
            public EntityQuery entityQuery;
            public NativeArray<Entity> entities;
            public NativeParallelMultiHashMap<int, Entity>.ParallelWriter DisabledEntity;

            public void Execute(int index)
            {
                if (entityQuery.Matches(entities[index]))
                {

                }else
                {
                    DisabledEntity.Add(index, entities[index]);
                }
            }
        }//����Ǯ�� ���鶧 ���

        #endregion

        #region Disabled Job

        partial struct TempTimer : IJobParallelFor
        {
            public EntityCommandBuffer.ParallelWriter ECB;
            [Unity.Collections.LowLevel.Unsafe.NativeDisableUnsafePtrRestriction]
            public EntityQuery entityQuery;

            public NativeArray<Entity> entities;
            public float Timer;
            public float SpawnInterval;

            public NativeParallelMultiHashMap<int, Entity>.ParallelWriter SpawnedEntity;// �б�� ���⸦ ���ÿ� ���� , ��� ����ó�� ����
            //NativeHashMap - �ȵ�

            //public EntityManager manager;//---------------------- �ȵ�
            // EntityManager, SystemAPI �� �ȵǳ�

            public void Execute(int index)// , in FluidSimlationComponent manager)
            {
                //if ((Timer / 10f) > (index / entities.Length))

                {
                    if (entities.Length > index)
                    {
                        if (entityQuery.Matches(entities[index]))
                        {
                            //Debug.Log("--" + index);
                            //SpawnedEntity[index] = false;
                        }
                        else
                        {
                            if (Timer > index * SpawnInterval)
                            {
                                ECB.SetEnabled(index, entities[index], true);

                                var temp = new FluidSimlationComponent();
                                temp.velocity = Vector3.forward;
                                //ECB.SetComponent(index, entities[index], temp);// �̰ž��� �Ѱ��� ����

                                //SpawnedEntity[index] = true;
                                SpawnedEntity.Add(index, entities[index]);
                            }
                        }
                    }
                }
            }
        }
        partial struct StartVelocity : IJobEntity
        {
            public EntityCommandBuffer.ParallelWriter ECB;
            public NativeArray<Entity> entities;
            public Vector3 velocity;
            [ReadOnly] public NativeParallelMultiHashMap<int, Entity> SpawnedEntity;

            public void Execute([EntityIndexInQuery] int index, in LocalTransform trans, in SpawnedTag tag)
            {
                //Debug.Log(index + " : " + SpawnedEntity[index]);

                if (SpawnedEntity.TryGetFirstValue(index, out Entity vaule, out var it))//(SpawnedEntity[index])
                {
                    var temp = new FluidSimlationComponent();
                    temp.position = trans.Position;
                    temp.velocity = velocity;
                    ECB.SetComponent(index, vaule, temp);// ����ȿ�� Ű�� ���� ����� , ���� �״�� �ְ�...

                    var tempPos = trans;
                    tempPos.Scale = 0.25f;
                    Debug.Log("new Spawn");
                    //ECB.SetComponent(index, vaule, tempPos);
                }
                else
                {
                    if (trans.Scale < 1)
                    {
                        var tempPos = trans;
                        tempPos.Scale = 1;

                        //ECB.SetComponent(index, entities[index], tempPos);
                    }
                }

                //SpawnedEntity.ContainsKey(index)
                //SpawnedEntity.TryGetValue(index, out Entity temp);
            }
        }

        #endregion

        protected override void OnCreate()
        {
            IntiECB = World.GetOrCreateSystemManaged<BeginInitializationEntityCommandBufferSystem>();
            RequireForUpdate<WaterFallSpawnComponent>();

            DisabledEntity = new(entities.Length, Allocator.TempJob);
        }
        protected override void OnStopRunning()
        {
            entities.Dispose();
        }

        protected override void OnUpdate()
        {
            //------- ���������� ����
            var ecb = IntiECB.CreateCommandBuffer().AsParallelWriter();//�����۾�      

            /*
             * SpawnWorkType.Inti : ����
             * SpawnWorkType.Reset : ��� ��Ȱ��ȭ��Ű�� , entities �� ����
             * SpawnWorkType.Spawning : ��Ȱ��ȭ�� ��ƼƼ ����� ����� , ���� ������ �ٽ� Ȱ��ȭ ��Ű�� , ������ �ִ°� �ٽ� ȸ��
             */

            switch (SpawnWork)
            {
                case SpawnWorkType.Inti:
                    {
                        Entities
                            //.WithAll<RemakedFluidSpawnComponent>()
                            .WithName("WaterFall")
                            .WithBurst(FloatMode.Default, FloatPrecision.Standard, true)
                            .ForEach((Entity e, int entityInQueryIndex, in WaterFallSpawnComponent manager, in LocalTransform transform) =>
                            {

                                var random = new Random(24825);
                                int size = Mathf.FloorToInt(Mathf.Pow(manager.Amount, 1 / 3f));

                                for (int i = 0; i < manager.Amount; i++)
                                {
                                    var instance = ecb.Instantiate(entityInQueryIndex, manager.particle);

                                    /*
                                    var position = new float3((i % size) * 1.2f + random.NextFloat(-0.1f, 0.1f) * manager.RandomPower,
                                        0 + (i / size / size) * 1.2f,
                                        ((i / size) % size) * 1.2f + random.NextFloat(-0.1f, 0.1f) * manager.RandomPower) + transform.Position;*/

                                    var position = new float3(random.NextFloat(-1f, 1f), random.NextFloat(-1f, 1f), random.NextFloat(-1f, 1f)) * manager.RandomPower + transform.Position;

                                    var Ltrans = new LocalTransform
                                    {
                                        Position = position,
                                        Rotation = quaternion.identity,
                                        Scale = 1
                                    };

                                    ecb.SetComponent(entityInQueryIndex, instance, Ltrans);
                                    ecb.AddComponent(entityInQueryIndex, instance, new SpawnedTag());
                                    //ecb.SetComponent(entityInQueryIndex, instance, new FluidSimlationComponent());
                                    //ecb.SetEnabled(entityInQueryIndex, instance, false);
                                }
                            }).ScheduleParallel();

                        IntiECB.AddJobHandleForProducer(Dependency);
                        ParticlesQuery = GetEntityQuery(typeof(FluidSimlationComponent));

                        SpawnWork = SpawnWorkType.Reset;
                        break;
                    }
                case SpawnWorkType.Reset:
                    {

                        if (entities.Length == 0)
                        {
                            entities = ParticlesQuery.ToEntityArray(Allocator.Persistent);
                            //Manager = SystemAPI.GetSingleton<WaterFallSpawnComponent>();
                        }

                        SetEnableAll setEnableJob = new SetEnableAll
                        {
                            ECB = ecb,
                            entities = entities,
                            Vaule = false
                        };
                        JobHandle SetEnableHandle = setEnableJob.ScheduleParallel(ParticlesQuery, Dependency);
                        SetEnableHandle.Complete();

                        SpawnWork = SpawnWorkType.Spawning;
                        break;
                    }
                case SpawnWorkType.Spawning:
                    {

                        Timer += SystemAPI.Time.DeltaTime;


                        DisabledEntity = new(entities.Length, Allocator.TempJob);
                        GetDisableEntity DisabledEntityJob = new GetDisableEntity
                        {
                            entities = entities,
                            entityQuery = ParticlesQuery,
                            DisabledEntity = DisabledEntity.AsParallelWriter()
                        };
                        var GetDisabledHandle = DisabledEntityJob.Schedule(entities.Length, 64, Dependency);
                        GetDisabledHandle.Complete();


                        var manager = SystemAPI.GetSingleton<WaterFallSpawnComponent>();
                        var VauleArray = DisabledEntity.GetValueArray(Allocator.Temp);

                        var random = new Random(24825 + (uint)(Timer * 10000));//(uint)VauleArray.Length

                        var SpawnerEntity = SystemAPI.GetSingletonEntity<WaterFallSpawnComponent>();
                        var SpawnerManager = SystemAPI.GetSingleton<WaterFallSpawnComponent>();
                        var SpawnerTrans = SystemAPI.GetComponent<LocalTransform>(SpawnerEntity);

                        if (manager.SpawnInterval < Timer && manager.SpawnAmountToOnece <= VauleArray.Length )
                        {
                            for(int i = 0; i < manager.SpawnAmountToOnece; i++)
                            {
                                IntiECB.CreateCommandBuffer().SetEnabled(VauleArray[i], true);
                                var Ltrans = SystemAPI.GetComponent<LocalTransform>(VauleArray[i]);
                                var Ldata = new FluidSimlationComponent();
                                Ldata.position = //Ltrans.Position;
                                    SpawnerTrans.Position 
                                    + math.normalize(new float3(random.NextFloat(-1, 1), random.NextFloat(-1, 1), random.NextFloat(-1, 1))) * SpawnerManager.SpawnRadius;
                                Ldata.velocity = SpawnerManager.IntiVelocity;
                                IntiECB.CreateCommandBuffer().SetComponent(VauleArray[i], Ldata);
                            }
                            

                            Timer = 0;
                        }// Get Pool
                        VauleArray.Dispose();

                        {
                            var ReturnPoolHandle = Entities
                                .WithName("WaterFall_ReturnPool")
                                .WithBurst(FloatMode.Default, FloatPrecision.Standard, true)
                                .ForEach((Entity e, int entityInQueryIndex, in FluidSimlationComponent fluid) =>
                                {
                                    if (fluid.isGround && fluid.velocity.sqrMagnitude < (1))
                                        ecb.SetEnabled(entityInQueryIndex, e, false);
                                }).ScheduleParallel(GetDisabledHandle);
                            ReturnPoolHandle.Complete();
                        }// Retrun Pool

                        DisabledEntity.Dispose();//�Ҵ� ���� �ϴϱ� ������  �þ�µ�?
                        break;
                    }

            }//End Switch
        }
    }
}
