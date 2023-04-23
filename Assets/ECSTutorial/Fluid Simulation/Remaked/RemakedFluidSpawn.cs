using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;

namespace FluidSimulate
{
    public class RemakedFluidSpawn : MonoBehaviour
    {
        public GameObject particleObj;
        public int Amount;
        public float RandomPower;

        public void Start()
        {
            
        }//�̰� �߰��ϸ� �ν����Ϳ��� ��Ȱ��ȭ ����
    }
    public struct RemakedFluidSpawnComponent : IComponentData
    {
        public Entity particle;
        public int Amount;
        public float RandomPower;
    }
    public class RemakedFluidSpawnBake : Baker<RemakedFluidSpawn>
    {
        public override void Bake(RemakedFluidSpawn authoring)
        {
            AddComponent(new RemakedFluidSpawnComponent
            {
                particle = GetEntity(authoring.particleObj),//authoring.Gameobject....�� �Ἥ ��� �ȵȰſ���...
                Amount = authoring.Amount,
                RandomPower = authoring.RandomPower
            });
        }
    }

}