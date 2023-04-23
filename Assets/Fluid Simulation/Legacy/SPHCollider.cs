using System.Collections;
using System.Collections.Generic;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

public class SPHCollider : MonoBehaviour
{
    public float3 position;
    public float3 right;
    public float3 up;
    public float2 scale;
}
public struct SPHColliderComponent : IComponentData
{
    public float3 position;
    public float3 right;
    public float3 up;
    public float2 scale;
}

public class SPHColliderBaker : Baker<SPHCollider>
{
    public override void Bake(SPHCollider authoring)
    {
        AddComponent(new SPHColliderComponent
        {
            position = authoring.position,
            right = authoring.right,
            up = authoring.up,
            scale = authoring.scale
        });
    }
}