using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[CreateAssetMenu(fileName = "Data", menuName = "ScriptableObjects/Hull", order = 1)]
public class HullShape : ScriptableObject
{
	public string prefabName;

	public List<HullSection> hullSections = new List<HullSection>();
}

[System.Serializable]
public class HullSection
{
	public float zPos, bottomY, topY, normalizedSectionLength, normalizedSectionWidth, normalizedSectionHeight;
	public HullSectionShape hullShape;

} 