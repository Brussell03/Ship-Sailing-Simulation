using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[CreateAssetMenu(fileName = "Data", menuName = "ScriptableObjects/Hull", order = 1)]
public class HullShape : ScriptableObject
{
	public string prefabName;

	public List<HullSection> hullSections = new List<HullSection>();
	public Rudder rudder = new Rudder();

	public Vector2[] GetUnnormalizedHullPoints(int sectionIndex, float height, float width) {
		Vector2[] unnormalizedPoints = new Vector2[hullSections[sectionIndex].hullShape.hullPoints.Count];

		float yOffset = hullSections[sectionIndex].topY * height - hullSections[sectionIndex].hullShape.hullPoints[unnormalizedPoints.Length - 1].y * hullSections[sectionIndex].normalizedSectionHeight * height;
		for (int i = 0; i < unnormalizedPoints.Length; i++) {
			unnormalizedPoints[i] = new Vector2(hullSections[sectionIndex].hullShape.hullPoints[i].x * hullSections[sectionIndex].normalizedSectionWidth * width, hullSections[sectionIndex].hullShape.hullPoints[i].y * hullSections[sectionIndex].normalizedSectionHeight * height + yOffset);
		}

		return unnormalizedPoints;
	}

	public static Vector2[] GetUnnormalizedHullPoints(HullSection hullSection, float height, float width) {
		Vector2[] unnormalizedPoints = new Vector2[hullSection.hullShape.hullPoints.Count];

		float yOffset = hullSection.topY * height - hullSection.hullShape.hullPoints[unnormalizedPoints.Length - 1].y * hullSection.normalizedSectionHeight * height;
		for (int i = 0; i < unnormalizedPoints.Length; i++) {
			unnormalizedPoints[i] = new Vector2(hullSection.hullShape.hullPoints[i].x * hullSection.normalizedSectionWidth * width, hullSection.hullShape.hullPoints[i].y * hullSection.normalizedSectionHeight * height + yOffset);
		}

		return unnormalizedPoints;
	}

	public Vector2[] GetLeftHullPoints(int sectionIndex) {
		Vector2[] flippedPoints = new Vector2[hullSections[sectionIndex].hullShape.hullPoints.Count];

		// Flipping about the y-axis
		for (int i = 0; i < flippedPoints.Length; i++) {
			flippedPoints[i] = (new Vector2(-hullSections[sectionIndex].hullShape.hullPoints[i].x, hullSections[sectionIndex].hullShape.hullPoints[i].y));
		}

		return flippedPoints;
	}

	public Vector2[] GetUnnormalizedLeftHullPoints(int sectionIndex, float height, float width) {
		Vector2[] points = GetUnnormalizedHullPoints(sectionIndex, height, width);
		Vector2[] flippedPoints = new Vector2[points.Length];

		// Flipping about the y-axis
		for (int i = 0; i < flippedPoints.Length; i++) {
			flippedPoints[i] = (new Vector2(-points[i].x, points[i].y));
		}

		return flippedPoints;
	}

	public Vector2[] GetMirroredHullPoints(Vector2[] points) {
		Vector2[] flippedPoints = new Vector2[points.Length];

		// Flipping about the y-axis
		for (int i = 0; i < flippedPoints.Length; i++) {
			flippedPoints[i] = (new Vector2(-points[i].x, points[i].y));
		}

		return flippedPoints;
	}
}

[System.Serializable]
public class HullSection
{
	public float zPos, bottomY, topY, normalizedSectionLength, normalizedSectionWidth, normalizedSectionHeight;
	public HullSectionShape hullShape;

}

[System.Serializable]
public class Rudder {
	public Vector3 pivot = Vector3.zero;
	public List<Vector2> rudderPoints = new List<Vector2>();

	public Vector3[] GetUnnormalizedPoints(float width, float height, float length) {
		Vector3[] points = new Vector3[rudderPoints.Count];
		for (int i = 0; i < rudderPoints.Count; i++) {
			points[i] = new Vector3(0, rudderPoints[i].y * height, rudderPoints[i].x * length) + new Vector3(pivot.x * width, pivot.y * height, pivot.z * length);
		}

		return points;
	}

	public Vector3[] GetUnnormalizedRotatedPoints(float width, float height, float length, float rudderAngle) {
		Vector3[] points = new Vector3[rudderPoints.Count];

		for (int i = 0; i < rudderPoints.Count; i++) {
			points[i] = Quaternion.AngleAxis(rudderAngle, Vector3.up) * new Vector3(0, rudderPoints[i].y * height, rudderPoints[i].x * length) + new Vector3(pivot.x * width, pivot.y * height, pivot.z * length);
		}

		return points;
	}

	public Vector3 GetCentroid(float width, float height, float length, float rudderAngle) {
		Vector3 centroid = Vector3.zero;
		for (int i = 0; i < rudderPoints.Count; i++) {
			centroid += Quaternion.AngleAxis(rudderAngle, Vector3.up) * new Vector3(0, rudderPoints[i].y * height, rudderPoints[i].x * length) + new Vector3(pivot.x * width, pivot.y * height, pivot.z * length);
		}

		return centroid / rudderPoints.Count;
	}

	public Vector2[] GetUnnormalizedOutline(float height, float length) {
		Vector2[] points = new Vector2[rudderPoints.Count];
		for (int i = 0; i < rudderPoints.Count; i++) {
			points[i] = new Vector2(rudderPoints[i].y * height, rudderPoints[i].x * length);
		}

		return points;
	}

	public Vector3 GetUnnormalizedPivot(float width, float height, float length) {
		return new Vector3(pivot.x * width, pivot.y * height, pivot.z * length);
	}
}