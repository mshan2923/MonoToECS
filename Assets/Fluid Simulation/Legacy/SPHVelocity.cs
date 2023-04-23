using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

[System.Serializable]
public class SPHVelocity : MonoBehaviour
{
    public float3 Value;
}

public struct SPHVelocityComponent : IComponentData
{
    public float3 value;
}

public class SPHVelocityBaker : Baker<SPHVelocity>
{
    public override void Bake(SPHVelocity authoring)
    {
        AddComponent(new SPHVelocityComponent{value = authoring.Value});
    }
}