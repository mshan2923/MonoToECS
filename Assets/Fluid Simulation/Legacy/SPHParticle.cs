using Unity.Entities;
using UnityEngine;

public class SPHParticle : MonoBehaviour
{
    public float radius;
    public float smoothingRadius;
    public float smoothingRadiusSq;

    public float mass;

    public float restDensity;
    public float viscosity;
    public float gravityMult;

    public float drag;
    public float bound = 0.5f;
}

public struct SPHParticleComponent : ISharedComponentData
{
    public float radius;
    public float smoothingRadius;
    public float smoothingRadiusSq;

    public float mass;

    public float restDensity;
    public float viscosity;
    public float gravityMult;

    public float drag;
    public float bound;
}
public class SPHParticleBaker : Baker<SPHParticle>
{
    public override void Bake(SPHParticle authoring)
    {
        AddSharedComponent(new SPHParticleComponent
        {
            radius = authoring.radius,
            smoothingRadius = authoring.smoothingRadius,
            smoothingRadiusSq = authoring.smoothingRadiusSq,
            mass = authoring.mass,
            restDensity = authoring.restDensity,
            viscosity = authoring.viscosity,
            gravityMult = authoring.gravityMult,
            drag = authoring.drag,
            bound = authoring.bound
        });
    }
}