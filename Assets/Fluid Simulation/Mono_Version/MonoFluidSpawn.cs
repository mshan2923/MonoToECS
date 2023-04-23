using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

public class MonoFluidSpawn : MonoBehaviour
{
    public int Amount = 1000;
    public GameObject prefab;

    void Start()
    {
        var random = new Unity.Mathematics.Random(1);
        var trans = gameObject.transform;

        for(int i = 0; i < Amount; i++)
        {
            var position = new float3(i % 16 + random.NextFloat(-0.1f, 0.1f) + trans.position.x,
                2 + (i / 16 / 16) * 1.1f + trans.position.y,
                (i / 16) % 16) + random.NextFloat(-0.1f, 0.1f) + trans.position.z;

            GameObject.Instantiate(prefab, position, Quaternion.identity);
        }
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
