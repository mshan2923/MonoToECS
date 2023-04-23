using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using Unity.Collections;

public class SPHManager : MonoBehaviour
{
    [Header("Import")]
    public GameObject sphParticlePrefab = null;
    public GameObject sphColliderPrefab = null;
    private EntityManager manager;

        // Properties
    [Header("Properties")]
    public int amount = 5000;

    void Start()
    {
        
    }
    //System에서 4번 예제 처럼 스폰
}
public struct SPHManagerComponent : IComponentData
{
    public Entity Particle;
    public Entity Collider;
    public int Amount;
}

public class SPHManagerBaker : Baker<SPHManager>
{
    public override void Bake(SPHManager authoring)
    {
        AddComponent(new SPHManagerComponent
        {
            Particle = GetEntity(authoring.sphParticlePrefab),
            Collider = GetEntity(authoring.sphColliderPrefab),
            Amount = authoring.amount
        });
    }
}
