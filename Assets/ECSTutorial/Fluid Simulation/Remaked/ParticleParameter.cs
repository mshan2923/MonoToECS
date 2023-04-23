using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;

namespace FluidSimulate
{
    public enum SimulationType { ECS , HashedECS};
    public class ParticleParameter : MonoBehaviour
    {
        public float particleRadius = 1f;
        public float smoothRadius = 0f;
        //public float restDensity;
        public Vector3 gravity = new Vector3(0, -9.81f, 0);
        //public float particleMass;
        public float particleViscosity = 0.25f;
        public float particleDrag = 0.2f;

        public int MoveFPS = 120;

        public AnimationCurve collisionPushMultiply = new AnimationCurve(new Keyframe(0, 1), new Keyframe(1, 10));
        public float collisionPush = 0.1f;
        [Space(10)]
        public SimulationType simulationType;

        public void Start()
        {
            
        }
    }

    public struct ParticleParameterComponent : IComponentData
    {
        public float ParticleRadius;
        public float SmoothRadius;
        //public float restDensity;
        public Vector3 Gravity;
        //public float particleMass;
        public float ParticleViscosity;
        public float ParticleDrag;
        public float DT;

        public float CollisionPush;
        public Keyframe CollisionPushStart;
        public Keyframe CollisionPushEnd;

        public SimulationType simulationType;

        public static float Evaluate(float t, Keyframe keyframe0, Keyframe keyframe1)
        {
            float dt = keyframe1.time - keyframe0.time;

            float m0 = keyframe0.outTangent * dt;
            float m1 = keyframe1.inTangent * dt;

            float t2 = t * t;
            float t3 = t2 * t;

            float a = 2 * t3 - 3 * t2 + 1;
            float b = t3 - 2 * t2 + t;
            float c = t3 - t2;
            float d = -2 * t3 + 3 * t2;

            return a * keyframe0.value + b * m0 + c * m1 + d * keyframe1.value;
        }

        public float Evaluate(float t)
        {
            float dt = CollisionPushEnd.time - CollisionPushStart.time;

            float m0 = CollisionPushStart.outTangent * dt;
            float m1 = CollisionPushEnd.inTangent * dt;

            float t2 = t * t;
            float t3 = t2 * t;

            float a = 2 * t3 - 3 * t2 + 1;
            float b = t3 - 2 * t2 + t;
            float c = t3 - t2;
            float d = -2 * t3 + 3 * t2;

            return a * CollisionPushStart.value + b * m0 + c * m1 + d * CollisionPushEnd.value;
        }
    }
    public class ParticleParameterBake : Baker<ParticleParameter>
    {
        public override void Bake(ParticleParameter authoring)
        {
            AddComponent(new ParticleParameterComponent
            {
                ParticleRadius = authoring.particleRadius,
                SmoothRadius = authoring.smoothRadius,
                Gravity = authoring.gravity,
                ParticleViscosity = authoring.particleViscosity,
                ParticleDrag = authoring.particleDrag,
                DT = 1f / authoring.MoveFPS,

                CollisionPush = authoring.collisionPush,
                CollisionPushStart =
                    authoring.collisionPushMultiply.keys.Length >= 2 ? authoring.collisionPushMultiply.keys[0] : default,
                CollisionPushEnd =
                    authoring.collisionPushMultiply.keys.Length >= 2 ? authoring.collisionPushMultiply.keys[1] : default,

                simulationType = authoring.simulationType
            });;
        }
    }
}