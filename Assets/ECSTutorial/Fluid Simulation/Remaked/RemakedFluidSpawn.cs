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
            
        }//이거 추가하면 인스팩터에서 비활성화 가능
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
                particle = GetEntity(authoring.particleObj),//authoring.Gameobject....를 써서 계속 안된거였어...
                Amount = authoring.Amount,
                RandomPower = authoring.RandomPower
            });
        }
    }

}