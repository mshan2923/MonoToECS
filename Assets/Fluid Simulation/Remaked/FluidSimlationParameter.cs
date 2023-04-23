using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;

namespace FluidSimulate
{
    public class FluidSimlationParameter : MonoBehaviour
    {
        public Vector3 Position;

        public Vector3 Velocity;
        public Vector3 Force;
        public Vector3 Acc;//가속

        public bool IsGround;
    }
    public struct FluidSimlationComponent : IComponentData
    {
        public Vector3 position;

        public Vector3 velocity;
        public Vector3 force;
        public Vector3 acc;//가속

        public bool isGround;
    }
    public class FluidSimlationBaker : Baker<FluidSimlationParameter>
    {
        public override void Bake(FluidSimlationParameter authoring)
        {
            AddComponent(new FluidSimlationComponent
            {
                position = authoring.Position,
                velocity = authoring.Velocity,
                force = authoring.Force,
                acc = authoring.Force,
                isGround = authoring.IsGround
            });
        }
    }
}