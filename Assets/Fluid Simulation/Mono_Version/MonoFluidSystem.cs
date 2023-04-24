using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Jobs;
using Unity.Collections;
using Unity.Mathematics;

public class MonoFluidSystem : MonoBehaviour
{
    //우선 해슁 없이 , 접촉(일정 범위안)하고 있는거 충돌 반사각을 평균내서 이동

    private struct SPHParticle
    {
        public Vector3 position;

        public Vector3 velocity;
        public Vector3 Force;
        public Vector3 Acc;//가속

        public bool IsGround;

        public void Init(Vector3 _position)
        {
            position = _position;
            //parameterID = _parameterID;
            //go = _go;

            velocity = Vector3.zero;

            Force = Vector3.zero;
            Acc = Vector3.zero;
            //Penetration = Vector3.zero;
            IsGround = false;
        }
    }
    [System.Serializable]
    private struct SPHParameters
    {
        public float particleRadius;
        public float smoothingRadius;
        public float restDensity;
        public float gravityMult;
        public float particleMass;
        public float particleViscosity;
        public float particleDrag;
    }

    #region Job
    struct HashPosition : IJobParallelFor
    {
        public NativeArray<SPHParticle> datas;
        public float Radius;
        public NativeParallelMultiHashMap<int, int>.ParallelWriter hashMap;//NativeMultiHashMap<int, int>.ParallelWriter hashMap;

        public void Execute(int index)
        {
            int hash = GridHash.Hash(datas[index].position, Radius);
            hashMap.Add(hash, index);
        }
    }
    struct FindNeighbors : IJobParallelFor
    {
        [ReadOnly] public NativeArray<SPHParticle> datas;
        [ReadOnly] public NativeParallelMultiHashMap<int, int> hashMap;//NativeMultiHashMap<int, int> hashMap;
        [ReadOnly] public NativeArray<int> cellOffsetTable;
        [WriteOnly] public NativeArray<float3> Forces;
        [WriteOnly] public NativeArray<float3> Penetration;

        public float Radius;

        public void Execute(int index)
        {
            float3 pos = datas[index].position;
            int3 gridOffset;
            int3 gridPosition = GridHash.Quantize(pos, Radius);
            int i, j, hash;
            float3 dir = float3.zero;
            float3 penetration = float3.zero;
            bool found;

            for (int oi = 0; oi < 27; oi++)
            {
                i = oi * 3;
                gridOffset = new int3(cellOffsetTable[i], cellOffsetTable[i + 1], cellOffsetTable[i + 2]);
                hash = GridHash.Hash(gridPosition + gridOffset);
                NativeParallelMultiHashMapIterator<int> iterator;//NativeMultiHashMapIterator<int> iterator;
                found = hashMap.TryGetFirstValue(hash, out j, out iterator);
                while(found)
                {
                    float3 rij = datas[j].position;
                    rij -= pos;
                    float r = math.length(rij);

                    if (r < Radius)
                    {
                        dir += math.normalizesafe(-rij) * Mathf.Pow(10, Mathf.Lerp(2, 1, r / (Radius * 0.5f)));
                        penetration += math.normalizesafe(-rij) * (Radius - r);
                    }

                    // Next neighbor
                    found = hashMap.TryGetNextValue(out j, ref iterator);
                }
            }

            //if (found)
            {
                //SPHParticle temp = datas[index];
                //temp.Force = new Vector3(dir.x, dir.y, dir.z);
                //datas[index] = temp;
            }//읽기와 쓰기가 같이 있어 문제 생기나봄
            Forces[index] = dir;
            Penetration[index] = penetration;
        }
    }
    struct SetForce : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float3> Forces;
        [ReadOnly] public NativeArray<float3> Penetration;

        public NativeArray<SPHParticle> datas;
        void IJobParallelFor.Execute(int index)
        {
            var temp = datas[index];
            temp.Force = Forces[index];
            //temp.Penetration = Penetration[index];
            datas[index] = temp;
        }
    }
    #endregion

    #region 변수
    // Consts
    private static Vector3 GRAVITY = new Vector3(0.0f, -9.81f, 0.0f);//프레임단 -0.08166 이동
    private const float DT = 0.008333f;//0.0163333f;

    // Properties
    [Header("Import")]
    [SerializeField] private GameObject character0Prefab = null;
    public float SpawnOffset = 0;
    [SerializeField] private float RandomPower = 1f;

    [Header("Parameters")]
    [SerializeField] private int parameterID = 0;
    [SerializeField] private SPHParameters[] parameters = null;
    [SerializeField] AnimationCurve CollisionPushMultiply = new AnimationCurve(new Keyframe(0, 1), new Keyframe(1, 10));
    [SerializeField] float CollsionPush = 0.1f;

    [Header("Properties")]
    [SerializeField] private int amount = 250;
    [SerializeField] private int rowSize = 16;
    [SerializeField] private int _debugIndex = 0;
    
    private int DebugIndex
    {
        get 
        {
            if (_debugIndex < 0)
                return 0;
            else if (_debugIndex > amount)
                return amount - 1;
            else
                return _debugIndex;
        }
    }

    // Data
    private SPHParticle[] particles;
    GameObject[] particleObj;
    #endregion

    void Start()
    {
        InitSPH();
    }

    // Update is called once per frame
    void Update()
    {
        {
            for(int i = 0; i < particles.Length; i++)
            {
                particles[i].Acc = GRAVITY;
                Vector3 dir = Vector3.zero;
                float MoveResistance = 0;//이동 저항력 - 파티클간의 충돌

                for (int j = 0; j < particles.Length; j++)
                {
                    var ij = particles[j].position - particles[i].position;//particles[i].position - particles[j].position;
                    var rad = parameters[parameterID].particleRadius + parameters[parameterID].smoothingRadius;
                    if ((ij).sqrMagnitude <= rad * rad)
                    {
                        dir += ij;

                        MoveResistance += Mathf.Clamp01(Vector3.Dot(-ij.normalized, -(particles[i].velocity + particles[i].Acc * DT).normalized));
                    }
                }// dir , MoveResistance 계산 -> 충돌 노멀값 , 이동 저항값 계산

                {
                    if (particles[i].position.y <= parameters[parameterID].particleRadius * 0.5f)
                    {
                        if (particles[i].IsGround == false)
                        {
                            particles[i].velocity = Vector3.Reflect(particles[i].velocity, Vector3.up)
                                * (1 - parameters[parameterID].particleViscosity);
                            //Ndir == 0
                        }
                        var tempVec = particles[i].Acc;
                        tempVec.y = 0;

                        particles[i].Acc = tempVec;
                        particles[i].IsGround = true;
                    }
                    else
                    {
                        particles[i].IsGround = false;
                    }
                }//바닥과 충돌

                {
                    if (Mathf.Approximately(dir.sqrMagnitude, 0))
                    {
                        if (particles[i].IsGround)
                        {
                            particles[i].velocity *= 1 - (parameters[parameterID].particleDrag * DT);
                        }
                    }
                    else
                    {
                        float CollisionAcc = Mathf.Max(1 - parameters[parameterID].particleViscosity, 0);
                        if (particles[i].IsGround)
                        {
                            if (MoveResistance >= 0)
                            {
                                var temp = Vector3.Reflect(particles[i].velocity, dir.normalized) * CollisionAcc;
                                temp.y = 0;

                                particles[i].velocity = temp.normalized * temp.magnitude;
                            }
                            else
                            {
                                var temp = Vector3.Reflect(-particles[i].velocity, dir.normalized) * CollisionAcc;
                                temp.y = 0;

                                particles[i].velocity = temp.normalized * temp.magnitude;
                            }

                        }
                        else
                        {
                            var CollisionRate = (parameters[parameterID].particleRadius - dir.magnitude) / parameters[parameterID].particleRadius;

                            particles[i].velocity += CollisionPushMultiply.Evaluate(CollisionRate) * CollsionPush * dir.normalized;//당김

                            if (Mathf.Abs(MoveResistance) > 0.1f)
                            {
                                var reflecVel = particles[i].velocity * (1 - parameters[parameterID].particleViscosity);

                                if (MoveResistance >= 0)
                                {
                                    particles[i].velocity = Vector3.Reflect((reflecVel + particles[i].Acc * DT), dir.normalized);//충돌
                                }
                                else
                                {
                                    particles[i].velocity = Vector3.Reflect((-reflecVel + particles[i].Acc * DT), dir.normalized);
                                }
                            }
                        }
                    }

                }//파티클간 충돌 -> 가라앉는거 방지

                {
                    if (particles[i].IsGround)
                    {
                        particles[i].Acc -= GRAVITY;
                    }
                    particles[i].velocity += particles[i].Acc * DT;

                    particles[i].position += particles[i].velocity * DT;
                    particleObj[i].transform.position += particles[i].velocity * DT;
                }//값 적용

                if (i == DebugIndex)
                {
                    print($"Velocity : {particles[i].velocity} / Acc : {particles[i].Acc} / Dir : {dir} / Ndir : {MoveResistance} / MoveResistance : { MoveResistance}" +
                            $"\n Pos : {particles[i].position} / Is Grand : {particles[i].IsGround} / Is Sleep : {MoveResistance > 0} / dir Length : {dir.magnitude}");
                }

            }
        }        
    }

    private void InitSPH()
    {
        particles = new SPHParticle[amount];
        particleObj = new GameObject[amount];

        for (int i = 0; i < amount; i++)
        {
            float jitter = (UnityEngine.Random.value * 2f - 1f) * parameters[parameterID].particleRadius * 0.1f * RandomPower;
            float x = (i % rowSize) * (1 + SpawnOffset) + UnityEngine.Random.Range(-0.1f, 0.1f) * RandomPower;
            float y = (i / rowSize) / rowSize * 1.1f * (1 + SpawnOffset);
            float z = ((i / rowSize) % rowSize) * (1 + SpawnOffset) + UnityEngine.Random.Range(-0.1f, 0.1f) * RandomPower;

            GameObject go = Instantiate(character0Prefab, gameObject.transform);
            go.transform.localScale = Vector3.one * parameters[parameterID].particleRadius;
            go.transform.position = new Vector3(x + jitter, y, z + jitter) + gameObject.transform.position;
            go.name = "char" + i.ToString();

            particles[i].Init(new Vector3(x, y, z) + gameObject.transform.position);
            particleObj[i] = go;
        }
    }

}
