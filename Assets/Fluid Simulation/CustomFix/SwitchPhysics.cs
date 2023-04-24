using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;

public enum SwitchSPHtype { convert , customFix, Optify}
public class SwitchPhysics : MonoBehaviour
{
    public SwitchSPHtype SwitchType;
}
public struct SwitchPhysicsComponent : IComponentData
{
    public SwitchSPHtype switchType;
}
public class SwitchSPHtypeBaker : Baker<SwitchPhysics>
{
    public override void Bake(SwitchPhysics authoring)
    {
        
        AddComponent(new SwitchPhysicsComponent { switchType = authoring.SwitchType });
    }
}
