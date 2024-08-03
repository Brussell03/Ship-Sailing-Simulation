using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[CreateAssetMenu(fileName = "Data", menuName = "ScriptableObjects/HullShape", order = 1)]
public class HullShape : ScriptableObject
{
    public string prefabName;

    public List<Vector2> hullPoints = new List<Vector2>();
    public float hullWidth;
}
