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

public partial class SPHManagerSystem : SystemBase
{
    BeginInitializationEntityCommandBufferSystem entityCommandBufferSystem;

    protected override void OnCreate()
    {
        base.OnCreate();
        entityCommandBufferSystem = World.GetOrCreateSystemManaged<BeginInitializationEntityCommandBufferSystem>();
    }

    protected override void OnUpdate()
    {
        var commandBuffer = entityCommandBufferSystem.CreateCommandBuffer().AsParallelWriter();

        Entities
            .WithName("SPHManager")
            .WithBurst(FloatMode.Default, FloatPrecision.Standard, true)
            .ForEach((Entity entity, int entityInQueryIndex, in SPHManagerComponent manager, in LocalTransform trans)=>
            {
                Debug.Log("Spawning");

                var random = new Random(1);
                int size = Mathf.FloorToInt(Mathf.Pow(manager.Amount, 1/3f));

                for (int i = 0; i < manager.Amount; i++)
                {
                        // entity 생성 예약
                        var instance = commandBuffer.Instantiate(entityInQueryIndex, manager.Particle);
                        
                        //var position = new float3((i % 16) * 1.2f + random.NextFloat(-0.1f, 0.1f),
                        //     2 + (i / 16 / 16) * 1.1f,
                        //      ((i / 16) % 16) * 1.2f + random.NextFloat(-0.1f, 0.1f)) + trans.Position;
                    var position = new float3((i % size) * 1.2f + random.NextFloat(-0.1f, 0.1f),
                                2 + (i / size / size) * 1.1f,
                                ((i / size) % size) * 1.2f + random.NextFloat(-0.1f, 0.1f)) + trans.Position;

                    
                    commandBuffer
                        .SetComponent(entityInQueryIndex, instance,
                            new LocalTransform {Position = position, Rotation = quaternion.identity, Scale = 1});

                        //========= (SHPManager.AddCollider부분)월드에 모든 SPHCollider를 가진 블럭마다 컨포넌트 추가
                        //========= 하는걸로 추정중 , 전에 컴퓨트 셰이더로 물리효과 내는것 처럼
                        //  ==> 파티클이 스폰되기전에 월드에 있는 SPHCollider 태크를 가진 오브젝트를 모아서
                        //   => SPHColliderComponent를 추가 , 더 자세한건 SHPSystem 연구하고
                }

                // 스폰 Entity를 제거한다. 제거하지 않으면 onUpdate에 의해 Cube가 계속 생성된다.
                commandBuffer.DestroyEntity(entityInQueryIndex, entity);
            }).ScheduleParallel();

        // 위의 작업이 완료되면
        // 예약한 명령을 다음 프레임에서 BeginInitializationEntityCommandBufferSystem 호출될때 처리하라고 등록한다.
        entityCommandBufferSystem.AddJobHandleForProducer(Dependency);
    }
}
