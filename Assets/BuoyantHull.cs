using System.Collections;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;
using static UnityEngine.Rendering.ProbeTouchupVolume;

[RequireComponent(typeof(Rigidbody))]
public class BuoyantHull : MonoBehaviour
{
    //public HullSectionShape hull;
	public HullShape hull;
	public WaterSurface waterSurface;
	[SerializeField] private float hullLength = 1f; // m
	[SerializeField] private float hullWidth = 1f; // m
	[SerializeField] private float hullHeight = 1f; // m

	//[SerializeField] private float shipMass = 10f; // kg
	//[SerializeField] private float shipLength = 10f; // m
	[SerializeField] private float shipRoll = 0f; // Deg
	[SerializeField] private float shipPitch = 0f; // Deg
	private float gravity = -9.81f; // m / s^2
    private float waterDensity = 1000f; // kg / m^3
	//private int longitudinalBuoyantForceSplit = 6;
	private int lateralBuoyantForceSplit = 2;
	//private int lowestSectionIndex = 0;
	//private float totalArea = 0f;
	//private Vector2 submergedCenterOfBuoyancy = Vector2.zero;

	private Rigidbody rb;
	private WaterSurfaceHeightSearch waterSurfaceHeightSearch;

    // Start is called before the first frame update
    void Start()
    {
		rb = GetComponent<Rigidbody>();

		// Looks in scene for existing water surface if it's not set
		if (waterSurface == null) {
			waterSurface = FindObjectsByType<WaterSurface>(FindObjectsSortMode.None)[0];
		}

		waterSurfaceHeightSearch = new WaterSurfaceHeightSearch(hull.hullSections.Count, lateralBuoyantForceSplit, waterSurface);

		foreach (HullSection hullSection in hull.hullSections)
		{
			hullSection.totalArea = GetTotalArea(hullSection);
			hullSection.submergedCenterOfBuoyancy = GetSubmergedCenterOfBuoyancy(hullSection);
		}
		//totalArea = GetTotalArea();
		//submergedCenterOfBuoyancy = GetSubmergedCenterOfBuoyancy();
	}

    // Update is called once per frame
    void Update()
    {
		// Recenter ship roll from 0 to 360 to -180 to 180
		if (transform.rotation.eulerAngles.z >= 0 && transform.rotation.eulerAngles.z <= 90) {
			shipRoll = transform.rotation.eulerAngles.z;
		} else if (transform.rotation.eulerAngles.z > 90 && transform.rotation.eulerAngles.z < 270) {
			// Ship unrecoverable
		} else if (transform.rotation.eulerAngles.z >= 270 && transform.rotation.eulerAngles.z < 360) {
			shipRoll = transform.rotation.eulerAngles.z - 360;
		}

		// Recenter ship pitch from 0 to 360 to -180 to 180
		if (transform.rotation.eulerAngles.x >= 0 && transform.rotation.eulerAngles.x <= 90) {
			shipPitch = transform.rotation.eulerAngles.x;
		} else if (transform.rotation.eulerAngles.x > 90 && transform.rotation.eulerAngles.x < 270) {
			// Ship unrecoverable
		} else if (transform.rotation.eulerAngles.x >= 270 && transform.rotation.eulerAngles.x < 360) {
			shipPitch = transform.rotation.eulerAngles.x - 360;
		}

		//float buoyantForce = -waterDensity * gravity * GetSubmergedArea() * shipLength;
		//float total = 0f;

		float[] sectionWaterLines = new float[hull.hullSections.Count];
		float[] sectionWaterRollAngles = new float[hull.hullSections.Count];
		float[] sectionWaterPitchAngles = new float[hull.hullSections.Count];
		float overallMinRotatedY = -hullHeight / 2f;

		if(!UpdateWaterValues(out sectionWaterLines, out sectionWaterRollAngles, out sectionWaterPitchAngles, out overallMinRotatedY)) {
			return;
		}

		float largestSubmergedHullSectionArea = 0f;
		Vector2 largestSubmergedHullSectionCentroid = Vector2.zero;
		ApplyBuoyancy(sectionWaterLines, sectionWaterRollAngles, sectionWaterPitchAngles, overallMinRotatedY, out largestSubmergedHullSectionArea, out largestSubmergedHullSectionCentroid);

		ApplyWaterDrag(sectionWaterLines, largestSubmergedHullSectionArea, largestSubmergedHullSectionCentroid);
	}

	private bool UpdateWaterValues(out float[] sectionWaterLines, out float[] sectionWaterRollAngles, out float[] sectionWaterPitchAngles, out float overallMinRotatedY) {

		sectionWaterLines = new float[hull.hullSections.Count];
		sectionWaterRollAngles = new float[hull.hullSections.Count];
		sectionWaterPitchAngles = new float[hull.hullSections.Count];
		overallMinRotatedY = -hullHeight / 2f;

		Vector3[] waterSamplePositions = new Vector3[hull.hullSections.Count * lateralBuoyantForceSplit];
		for (int x = 0; x < hull.hullSections.Count; x++) {
			for (int y = 0; y < lateralBuoyantForceSplit; y++) {
				//waterSamplePositions[x * lateralBuoyantForceSplit + y] = transform.TransformPoint(new Vector3(((hullWidth / lateralBuoyantForceSplit * y + hullWidth / (lateralBuoyantForceSplit * 2f)) - (hullWidth / 2f)), 0, hull.hullSections[x].zPos * hullLength));
				waterSamplePositions[x * lateralBuoyantForceSplit + y] = transform.TransformPoint(new Vector3(-0.8f + y * 1.6f, 0, hull.hullSections[x].zPos * hullLength));

			}
		}

		float[] waterHeights = waterSurfaceHeightSearch.GetWaterHeights(waterSamplePositions);

		for (int x = 0; x < hull.hullSections.Count; x++) {

			Vector2[] sectionWaterHeightPoints = new Vector2[lateralBuoyantForceSplit];
			float avgSectionWaterHeight = 0f;

			for (int i = x * lateralBuoyantForceSplit; i < (x + 1) * lateralBuoyantForceSplit; i++) {
				try {
					avgSectionWaterHeight += waterHeights[i];
				} catch {
					return false; // Height values failed to be found
				}
				sectionWaterHeightPoints[i - x * lateralBuoyantForceSplit] = new Vector2(waterSamplePositions[i].x, waterHeights[i]);
			}
			avgSectionWaterHeight /= lateralBuoyantForceSplit;

			sectionWaterLines[x] = avgSectionWaterHeight - transform.TransformPoint(new Vector3(0, 0, hull.hullSections[x].zPos * hullLength)).y + (hullHeight / 2f);

			float sectionWaterRollSlope;
			float sectionWaterB;
			FindLinearLeastSquaresFit(sectionWaterHeightPoints, out sectionWaterRollSlope, out sectionWaterB);
			sectionWaterRollAngles[x] = Mathf.Atan(sectionWaterRollSlope) * Mathf.Rad2Deg;

			if (x == 0) { // Calculate water pitch angle based on back-end and adjacent water height values

				Vector2[] lengthwiseHeightPoints = new Vector2[2];
				lengthwiseHeightPoints[0] = new Vector2(waterSamplePositions[x * lateralBuoyantForceSplit].z, avgSectionWaterHeight);
				lengthwiseHeightPoints[1] = new Vector2(waterSamplePositions[(x + 1) * lateralBuoyantForceSplit].z, (waterHeights[(x + 1) * lateralBuoyantForceSplit] + waterHeights[(x + 1) * lateralBuoyantForceSplit + 1]) / 2f);


				float sectionWaterPitchSlope;
				FindLinearLeastSquaresFit(lengthwiseHeightPoints, out sectionWaterPitchSlope, out sectionWaterB);

				sectionWaterPitchAngles[x] = Mathf.Atan(sectionWaterPitchSlope) * Mathf.Rad2Deg;

			} else if (x == hull.hullSections.Count - 1) { // Calculate water pitch angle based on front-end and adjacent water height values

				Vector2[] lengthwiseHeightPoints = new Vector2[2];
				lengthwiseHeightPoints[0] = new Vector2(waterSamplePositions[(x - 1) * lateralBuoyantForceSplit].z, (waterHeights[(x - 1) * lateralBuoyantForceSplit] + waterHeights[(x - 1) * lateralBuoyantForceSplit + 1]) / 2f);
				lengthwiseHeightPoints[1] = new Vector2(waterSamplePositions[x * lateralBuoyantForceSplit].z, avgSectionWaterHeight);

				float sectionWaterPitchSlope;
				FindLinearLeastSquaresFit(lengthwiseHeightPoints, out sectionWaterPitchSlope, out sectionWaterB);

				sectionWaterPitchAngles[x] = Mathf.Atan(sectionWaterPitchSlope) * Mathf.Rad2Deg;

			} else { // Caluclate water pitch angle based on current section water height and 2 adjacent sides water heights

				Vector2[] lengthwiseHeightPoints = new Vector2[3];
				lengthwiseHeightPoints[0] = new Vector2(waterSamplePositions[(x - 1) * lateralBuoyantForceSplit].z, (waterHeights[(x - 1) * lateralBuoyantForceSplit] + waterHeights[(x - 1) * lateralBuoyantForceSplit + 1]) / 2f);
				lengthwiseHeightPoints[1] = new Vector2(waterSamplePositions[x * lateralBuoyantForceSplit].z, avgSectionWaterHeight);
				lengthwiseHeightPoints[2] = new Vector2(waterSamplePositions[(x + 1) * lateralBuoyantForceSplit].z, (waterHeights[(x + 1) * lateralBuoyantForceSplit] + waterHeights[(x + 1) * lateralBuoyantForceSplit + 1]) / 2f);

				float sectionWaterPitchSlope;
				FindLinearLeastSquaresFit(lengthwiseHeightPoints, out sectionWaterPitchSlope, out sectionWaterB);

				sectionWaterPitchAngles[x] = Mathf.Atan(sectionWaterPitchSlope) * Mathf.Rad2Deg;

			}

			float sectionMinY = GetSectionMinRotatedY(hull.hullSections[x], shipRoll + sectionWaterRollAngles[x]);
			if (sectionMinY < overallMinRotatedY) {
				overallMinRotatedY = sectionMinY;
				//lowestSectionIndex = x;
			}
		}

		return true; // Height values found successfully
	}

	private void ApplyBuoyancy(float[] sectionWaterLines, float[] sectionWaterRollAngles, float[] sectionWaterPitchAngles, float overallMinRotatedY, out float largestSubmergedHullSectionArea, out Vector2 largestSubmergedHullSectionCentroid) {
		float[] submergedAreas = new float[hull.hullSections.Count];
		largestSubmergedHullSectionArea = 0f;
		largestSubmergedHullSectionCentroid = Vector2.zero;

		for (int x = 0; x < hull.hullSections.Count; x++) {

			Vector2 centerOfBuoyancy = Vector2.zero;
			submergedAreas[x] = GetSubmergedArea(sectionWaterLines[x], shipRoll + sectionWaterRollAngles[x], hull.hullSections[x], overallMinRotatedY, out centerOfBuoyancy);
			float sectionBuoyantForce = -waterDensity * gravity * submergedAreas[x] * (hullLength * hull.hullSections[x].normalizedSectionLength);

			Vector3 buoyantForcePos = transform.TransformPoint(new Vector3(centerOfBuoyancy.x, centerOfBuoyancy.y, hull.hullSections[x].zPos * hullLength));

			if (sectionBuoyantForce > 0) {
				rb.AddForceAtPosition(Quaternion.AngleAxis(sectionWaterRollAngles[x], transform.forward) * Quaternion.AngleAxis(sectionWaterPitchAngles[x], -transform.right) * Vector3.up * sectionBuoyantForce, buoyantForcePos, ForceMode.Force);
				Debug.DrawRay(buoyantForcePos, Quaternion.AngleAxis(sectionWaterRollAngles[x], transform.forward) * Quaternion.AngleAxis(sectionWaterPitchAngles[x], -transform.right) * Vector3.up * sectionBuoyantForce / 1000000, UnityEngine.Color.red, Time.deltaTime * 5);
				Debug.DrawRay(transform.TransformPoint(new Vector3(0, 0, hull.hullSections[x].zPos * hullLength)) - transform.up * (hullHeight / 2f), -transform.right * 8, UnityEngine.Color.green, Time.deltaTime * 5);
			} else {
				Debug.DrawRay(buoyantForcePos, Vector3.up * 20, UnityEngine.Color.blue, Time.deltaTime * 10);
			}

			if (submergedAreas[x] > largestSubmergedHullSectionArea) {
				largestSubmergedHullSectionArea = submergedAreas[x];
				largestSubmergedHullSectionCentroid = centerOfBuoyancy;
			}
		}
	}

	private void ApplyWaterDrag(float[] sectionWaterLines, float largestSubmergedHullSectionArea, Vector2 largestSubmergedHullSectionCentroid) {

		bool allAboveWater = true;
		for (int i = 0; i < sectionWaterLines.Length; i++) {
			if (sectionWaterLines[i] > 0) { // In water
				allAboveWater = false;
				break;
			}
		}

		if (allAboveWater) {
			return;
		}

		//Vector2[] sideOutline = new Vector2[hull.hullSections.Count * 2];
		List<Vector2> sideOutline = new List<Vector2>();
		int numInWater = 0;
		for (int i = 0; i < hull.hullSections.Count; i++) {
			//sideOutline[i] = new Vector2(hull.hullSections[i].zPos * hullLength, hull.hullSections[i].bottomY * hullHeight);
			//sideOutline[sideOutline.Length * 2 - 1 - i] = new Vector2(hull.hullSections[i].zPos * hullLength, hull.hullSections[i].topY * hullHeight);
			float sectionWaterLineCentered = sectionWaterLines[i] - hullHeight / 2f;
			if (hull.hullSections[i].bottomY * hullHeight < sectionWaterLineCentered) { // Section is in the water
				//print(i);
				sideOutline.Insert(numInWater, new Vector2(hull.hullSections[i].zPos * hullLength, hull.hullSections[i].bottomY * hullHeight));
				if (hull.hullSections[i].topY * hullHeight <= sectionWaterLineCentered) { // Section is fully submerged

					sideOutline.Insert(sideOutline.Count - numInWater, new Vector2(hull.hullSections[i].zPos * hullLength, hull.hullSections[i].topY * hullHeight));
				} else { // Section is partially submerged

					sideOutline.Insert(sideOutline.Count - numInWater, new Vector2(hull.hullSections[i].zPos * hullLength, sectionWaterLineCentered));
				}

				numInWater++;
			}
		}

		//Vector2[] rotatedSideOutline = GetRotatedHullPoints(sideOutline, shipPitch);
		float sideArea = GetPolygonArea(sideOutline.ToArray());
		Vector2 sideCentroid = GetPolygonCentroid(sideOutline.ToArray(), sideArea);



		//List<Vector2> forwardRightHullPoints = GetRotatedHullPoints(GetUnnormalizedHullPoints(hull.hullSections[lowestSectionIndex]), waterAngleWithRoll).ToList();
		//List<Vector2> forwardLeftHullPoints = GetRotatedHullPoints(GetLeftHullPoints(GetUnnormalizedHullPoints(hull.hullSections[lowestSectionIndex])), waterAngleWithRoll).ToList();

		// Add waterline deck point
		/*Vector2 deckPoint = new Vector2(0, forwardLeftHullPoints[forwardLeftHullPoints.Count - 1].y + ((forwardRightHullPoints[forwardRightHullPoints.Count - 1].y - forwardLeftHullPoints[forwardLeftHullPoints.Count - 1].y) / (forwardRightHullPoints[forwardRightHullPoints.Count - 1].x + forwardLeftHullPoints[forwardLeftHullPoints.Count - 1].x) * forwardLeftHullPoints[forwardLeftHullPoints.Count - 1].x));

		if (waterAngleWithRoll > 0 && forwardLeftHullPoints.Count > 1) {
			for (int i = 0; i < forwardRightHullPoints.Count - 1; i++) {
				if (forwardRightHullPoints[i + 1].x < 0f && forwardRightHullPoints[i].x > 0f) { // Hull line crosses y-axis
					float yIntercept = forwardRightHullPoints[i].y - forwardRightHullPoints[i].x * (forwardRightHullPoints[i + 1].y - forwardRightHullPoints[i].y) / (forwardRightHullPoints[i + 1].x - forwardRightHullPoints[i].x);
					deckPoint = new Vector2(-((yIntercept - forwardLeftHullPoints[forwardLeftHullPoints.Count - 1].y) * (forwardRightHullPoints[forwardRightHullPoints.Count - 1].x + forwardLeftHullPoints[forwardLeftHullPoints.Count - 1].x) / (forwardRightHullPoints[forwardRightHullPoints.Count - 1].y - forwardLeftHullPoints[forwardLeftHullPoints.Count - 1].y) - forwardLeftHullPoints[forwardLeftHullPoints.Count - 1].x), yIntercept);
					break;
				}
			}

			forwardLeftHullPoints.Add(deckPoint);
		} else if (waterAngleWithRoll < 0 && forwardRightHullPoints.Count > 1) {
			for (int i = 0; i < forwardLeftHullPoints.Count - 1; i++) {
				if (forwardLeftHullPoints[i + 1].x < 0f && forwardLeftHullPoints[i].x > 0f) { // Hull line crosses y-axis
					float yIntercept = forwardLeftHullPoints[i].y - forwardLeftHullPoints[i].x * (forwardLeftHullPoints[i + 1].y - forwardLeftHullPoints[i].y) / (forwardLeftHullPoints[i + 1].x - forwardLeftHullPoints[i].x);
					deckPoint = new Vector2(((yIntercept - forwardLeftHullPoints[forwardLeftHullPoints.Count - 1].y) * (forwardRightHullPoints[forwardRightHullPoints.Count - 1].x + forwardLeftHullPoints[forwardLeftHullPoints.Count - 1].x) / (forwardRightHullPoints[forwardRightHullPoints.Count - 1].y - forwardLeftHullPoints[forwardLeftHullPoints.Count - 1].y) - forwardLeftHullPoints[forwardLeftHullPoints.Count - 1].x), yIntercept);
					break;
				}
			}

			forwardRightHullPoints.Add(deckPoint);
		}*/

		/*List<Vector2> hullOutline = new List<Vector2>();
		float waterLineCentered = sectionWaterLines[lowestSectionIndex] - hullHeight / 2f;
		for (int i = 0; i < forwardRightHullPoints.Count; i++) {
			if (forwardRightHullPoints[i].y <= waterLineCentered) { // Point is underwater
				hullOutline.Add(forwardRightHullPoints[i]);
			} else if (forwardRightHullPoints[i].y > waterLineCentered && forwardRightHullPoints[i - 1].y < waterLineCentered) { // Waterline between previous and current point, interpolate xPos
				hullOutline.Add(new Vector2((waterLineCentered - forwardRightHullPoints[i - 1].y) * (forwardRightHullPoints[i].x - forwardRightHullPoints[i - 1].x) / (forwardRightHullPoints[i].y - forwardRightHullPoints[i - 1].y) + forwardRightHullPoints[i - 1].x, waterLineCentered));
			}
		}

		for (int i = forwardLeftHullPoints.Count - 1; i >= 0; i--) {
			if (forwardLeftHullPoints[i].y <= waterLineCentered) { // Point is underwater
				hullOutline.Add(forwardLeftHullPoints[i]);
			} else if (forwardLeftHullPoints[i].y > waterLineCentered && forwardLeftHullPoints[i - 1].y < waterLineCentered) { // Waterline between previous and current point, interpolate xPos
				hullOutline.Add(new Vector2((waterLineCentered - forwardLeftHullPoints[i - 1].y) * (forwardLeftHullPoints[i].x - forwardLeftHullPoints[i - 1].x) / (forwardLeftHullPoints[i].y - forwardLeftHullPoints[i - 1].y) + forwardLeftHullPoints[i - 1].x, waterLineCentered));
			}
		}

		float hullArea = GetPolygonArea(hullOutline.ToArray());
		Vector2 hullCentroid = GetPolygonCentroid(hullOutline.ToArray(), hullArea);*/

		//print(waterSurface.largeOrientationValue);
		//print(transform.rotation.eulerAngles.y);
		float relativeCurrentBearing = transform.rotation.eulerAngles.y + waterSurface.largeOrientationValue + 270f;
		if (relativeCurrentBearing > 360) {
			relativeCurrentBearing -= 360;
		} else if (relativeCurrentBearing < 0) {
			relativeCurrentBearing += 360;
		}
		//print(relativeCurrentBearing);
	}

	/*private float CalculateFlatWaterline() {
        float halfDisplacedVolumePerLength = (rb.mass / waterDensity) / shipLength / 2;
        float area = 0f;

        print("Target: " + halfDisplacedVolumePerLength);

        for (int i = 0; i < hull.hullPoints.Count - 1; i++) {

			float sectionArea = (hull.hullPoints[i].x + hull.hullPoints[i + 1].x) / 2 * (hull.hullPoints[i + 1].y - hull.hullPoints[i].y);
            //print(sectionArea);
            //print("Current: " + (area + sectionArea));
            if ((area + sectionArea) > halfDisplacedVolumePerLength) {
                float remainingArea = halfDisplacedVolumePerLength - area;

                float slope = (hull.hullPoints[i + 1].x - hull.hullPoints[i].x) / (hull.hullPoints[i + 1].y - hull.hullPoints[i].y);

                // Quadratic Formula 
                float dx1 = (-hull.hullPoints[i].x + Mathf.Sqrt(hull.hullPoints[i].x * hull.hullPoints[i].x - 2 * slope * -remainingArea)) / (slope);
				float dx2 = (-hull.hullPoints[i].x - Mathf.Sqrt(hull.hullPoints[i].x * hull.hullPoints[i].x - 2 * slope * -remainingArea)) / (slope);

                //print("dx1: " + dx1);
                //print("dx2: " + dx2);

				if (Mathf.Abs(dx1) < Mathf.Abs(dx2) && dx1 > 0) {
                    //print("Returning");
					return hull.hullPoints[i].y + dx1;
				} else if (Mathf.Abs(dx2) < Mathf.Abs(dx1) && dx2 > 0) {
					//print("Returning");
					return hull.hullPoints[i].y + dx2;
				} else if (Mathf.Abs(dx1) == Mathf.Abs(dx2)) {
					return hull.hullPoints[i].y + Mathf.Abs(dx1);
				}

			} else {
                area += sectionArea;
            }
		}

        return -1f;
	}
	*/
	/*private float CalculateRotatedWaterline(float waterlineAngle) {
		float displacedVolumePerLength = (rb.mass / waterDensity) / shipLength;
		float area = 0f;

		print("Target: " + displacedVolumePerLength);

		List<Vector2> rightHullPoints = GetRotatedHullPoints(hull.hullPoints, waterlineAngle);
		List<Vector2> leftHullPoints = GetRotatedHullPoints(GetLeftHullPoints(), waterlineAngle);

		// Find lowest y-position and its index

		float minY = 0f;
		int index = 0;
		for (int i = 0; i < rightHullPoints.Count; i++) {
			if (rightHullPoints[i].y < minY) {
				minY = rightHullPoints[i].y;
				index = i;
			}
		}
		for (int i = 0; i < leftHullPoints.Count; i++) {
			if (leftHullPoints[i].y < minY) {
				minY = leftHullPoints[i].y;
				index = i;
			}
		}
		//PrintList(rightHullPoints);
		//PrintList(leftHullPoints);

		// Redefine hull positions by setting a new origin at lowest point and flipping the left sides x-positions

		List<Vector2> newRightHullPoints = new List<Vector2>();
		List<Vector2> newLeftHullPoints = new List<Vector2>();

		Vector2 pivotOffset = Vector2.zero;

		// CCW Ship Roll is positive
		if (index != 0 && shipRoll >= 0) { // CCW Roll
			pivotOffset = new Vector2(leftHullPoints[index].x, leftHullPoints[index].y);

			leftHullPoints[index] = new Vector2(0f, 0f);

			newRightHullPoints.Add(leftHullPoints[index]);
			for (int i = index - 1; i > 0; i--) {
				newRightHullPoints.Add(new Vector2((leftHullPoints[i].x - pivotOffset.x), leftHullPoints[i].y - pivotOffset.y));
			}
			foreach (Vector2 point in rightHullPoints) {
				newRightHullPoints.Add(new Vector2((point.x - pivotOffset.x), point.y - pivotOffset.y));
			}

			newLeftHullPoints.Add(leftHullPoints[index]);
			for (int i = index + 1; i < leftHullPoints.Count; i++) {
				newLeftHullPoints.Add(new Vector2(-(leftHullPoints[i].x - pivotOffset.x), leftHullPoints[i].y - pivotOffset.y));
			}

		} else if (index != 0 && shipRoll < 0) { // CW Roll
			pivotOffset = new Vector2(rightHullPoints[index].x, rightHullPoints[index].y);

			rightHullPoints[index] = new Vector2(0f, 0f);

			newLeftHullPoints.Add(rightHullPoints[index]);
			for (int i = index - 1; i > 0; i--) {
				newLeftHullPoints.Add(new Vector2(-(rightHullPoints[i].x - pivotOffset.x), rightHullPoints[i].y - pivotOffset.y));
			}
			foreach (Vector2 point in leftHullPoints) {
				newLeftHullPoints.Add(new Vector2(-(point.x - pivotOffset.x), point.y - pivotOffset.y));
			}

			newRightHullPoints.Add(rightHullPoints[index]);
			for (int i = index + 1; i < rightHullPoints.Count; i++) {
				newRightHullPoints.Add(rightHullPoints[i] - pivotOffset);
			}
		} else {
			newRightHullPoints = rightHullPoints;
			foreach (Vector2 point in leftHullPoints) {
				newLeftHullPoints.Add(new Vector2(-point.x, point.y));
			}
		}
		//PrintList(newRightHullPoints);
		//PrintList(newLeftHullPoints);

		// Interpolate points for equal step intervals

		for (int i = 1; i < newLeftHullPoints.Count; i++) {
			for (int j = 1; j < newRightHullPoints.Count; j++) {
				if (newLeftHullPoints[i].y < newRightHullPoints[j].y) {
					newRightHullPoints.Insert(j, new Vector2((newLeftHullPoints[i].y - newRightHullPoints[j-1].y) * (newRightHullPoints[j].x - newRightHullPoints[j-1].x) / (newRightHullPoints[j].y - newRightHullPoints[j-1].y) + newRightHullPoints[j-1].x, newLeftHullPoints[i].y));
					break;
				} else if (newLeftHullPoints[i].y == newRightHullPoints[j].y) {
					break;
				}
			}
		}

		for (int i = 1; i < newRightHullPoints.Count; i++) {
			for (int j = 1; j < newLeftHullPoints.Count; j++) {
				if (newRightHullPoints[i].y < newLeftHullPoints[j].y) {
					newLeftHullPoints.Insert(j, new Vector2((newRightHullPoints[i].y - newLeftHullPoints[j - 1].y) * (newLeftHullPoints[j].x - newLeftHullPoints[j - 1].x) / (newLeftHullPoints[j].y - newLeftHullPoints[j - 1].y) + newLeftHullPoints[j - 1].x, newRightHullPoints[i].y));
					break;
				} else if (newRightHullPoints[i].y == newLeftHullPoints[j].y) {
					break;
				}
			}
		}

		PrintList(newRightHullPoints);
		PrintList(newLeftHullPoints);

		// Get the number of points on the shortest half of hull

		int pointsToTop = newLeftHullPoints.Count;
		if (newRightHullPoints.Count < pointsToTop) { pointsToTop = newRightHullPoints.Count; }

		// Iteratively add section areas to find waterline

		for (int i = 0; i < pointsToTop - 1; i++) {

			float sectionAreaLeft = (newLeftHullPoints[i].x + newLeftHullPoints[i + 1].x) / 2 * (newLeftHullPoints[i + 1].y - newLeftHullPoints[i].y);
			float sectionAreaRight = (newRightHullPoints[i].x + newRightHullPoints[i + 1].x) / 2 * (newRightHullPoints[i + 1].y - newRightHullPoints[i].y);
			
			if ((newLeftHullPoints[i].x >= 0f && newLeftHullPoints[i + 1].x < 0f)) {
				sectionAreaLeft /= 2;
			} else if (newLeftHullPoints[i].x < 0f && newLeftHullPoints[i + 1].x < 0f) {
				sectionAreaLeft = -sectionAreaLeft;
			}

			if ((newRightHullPoints[i].x >= 0f && newRightHullPoints[i + 1].x < 0f)) {
				sectionAreaRight /= 2;
			} else if (newRightHullPoints[i].x < 0f && newRightHullPoints[i + 1].x < 0f) {
				sectionAreaRight = -sectionAreaRight;
			}

			if ((area + sectionAreaLeft + sectionAreaRight) > displacedVolumePerLength) {
				float remainingArea = displacedVolumePerLength - area;

				float rightSlope = (newRightHullPoints[i + 1].x - newRightHullPoints[i].x) / (newRightHullPoints[i + 1].y - newRightHullPoints[i].y);
				float leftSlope = (newLeftHullPoints[i + 1].x - newLeftHullPoints[i].x) / (newLeftHullPoints[i + 1].y - newLeftHullPoints[i].y);

				float a = 0.5f * (rightSlope + leftSlope);
				float b = -newRightHullPoints[i].y * (rightSlope + leftSlope) + newRightHullPoints[i].x + newLeftHullPoints[i].x;
				float c = -newRightHullPoints[i].y * newRightHullPoints[i].x - newLeftHullPoints[i].y * newLeftHullPoints[i].x - remainingArea + 0.5f * newRightHullPoints[i].y * newRightHullPoints[i].y * (rightSlope + leftSlope);

				// Quadratic Formula 
				float dy1 = (-b + Mathf.Sqrt(b * b - 4f * a * c)) / (2f * a);
				//float dy2 = (-b - Mathf.Sqrt(b * b - 4f * a * c)) / (2f * a);

				if (shipRoll >= 0f) {
					float x = newRightHullPoints[i].x + (dy1 - newRightHullPoints[i].y) * (newRightHullPoints[i + 1].x - newRightHullPoints[i].x) / (newRightHullPoints[i + 1].y - newRightHullPoints[i].y);
					print(GetUnrotatedPoint(new Vector2(x, dy1) + pivotOffset, waterlineAngle));
					return GetUnrotatedPoint(new Vector2(x, dy1) + pivotOffset, waterlineAngle).y;
				} else {
					float x = newLeftHullPoints[i].x + (dy1 - newLeftHullPoints[i].y) * (newLeftHullPoints[i + 1].x - newLeftHullPoints[i].x) / (newLeftHullPoints[i + 1].y - newLeftHullPoints[i].y);
					print(GetUnrotatedPoint(new Vector2(-(x - pivotOffset.x), dy1 + pivotOffset.y), waterlineAngle));
					return GetUnrotatedPoint(new Vector2(-(x - pivotOffset.x), dy1 + pivotOffset.y), waterlineAngle).y;
				}

				if (Mathf.Abs(dy1) < Mathf.Abs(dy2) && dy1 > 0) {
					//print("Returning");
					print(GetUnrotatedPoint(new Vector2(-(x - pivotOffset.x), dy1 - pivotOffset.y)));
					return GetUnrotatedPoint(new Vector2(-(x - pivotOffset.x), dy1 - pivotOffset.y)).y;
					//return dy1;
				} else if (Mathf.Abs(dy2) < Mathf.Abs(dy1) && dy2 > 0) {
					//print("Returning");
					//return dy2;
				} else if (Mathf.Abs(dy1) == Mathf.Abs(dy2)) {
					return newRightHullPoints[i].y + Mathf.Abs(dy1);
				}

			} else if ((area + sectionAreaLeft + sectionAreaRight) == displacedVolumePerLength) {
				return newRightHullPoints[i].y;
			} else {
				area += sectionAreaLeft + sectionAreaRight;
			}
		}

		// One side is now submerged

		if (index != 0 && shipRoll >= 0) { // CCW
			for (int i = pointsToTop - 1; i < newRightHullPoints.Count - 1; i++) {

				float sectionArea = (newRightHullPoints[i].x + newRightHullPoints[i + 1].x) / 2 * (newRightHullPoints[i + 1].y - newRightHullPoints[i].y);

				if ((newRightHullPoints[i].x >= 0f && newRightHullPoints[i + 1].x < 0f)) {
					sectionArea /= 2;
				} else if (newRightHullPoints[i].x < 0f && newRightHullPoints[i + 1].x < 0f) {
					sectionArea = -sectionArea;
				}

				if ((area + sectionArea) > displacedVolumePerLength) {
					float remainingArea = displacedVolumePerLength - area;

					float slope = (newRightHullPoints[i + 1].x - newRightHullPoints[i].x) / (newRightHullPoints[i + 1].y - newRightHullPoints[i].y);

					// Quadratic Formula
					float dy1 = (-newRightHullPoints[i].x + Mathf.Sqrt(newRightHullPoints[i].x * newRightHullPoints[i].x - 2 * slope * -remainingArea)) / (slope);
					//float dy2 = (-newRightHullPoints[i].x - Mathf.Sqrt(newRightHullPoints[i].x * newRightHullPoints[i].x - 2 * slope * -remainingArea)) / (slope);

					float x = newRightHullPoints[i].x + (dy1) * (newRightHullPoints[i + 1].x - newRightHullPoints[i].x) / (newRightHullPoints[i + 1].y - newRightHullPoints[i].y);
					print(GetUnrotatedPoint(new Vector2(x, newRightHullPoints[i].y + dy1) + pivotOffset, waterlineAngle));
					return GetUnrotatedPoint(new Vector2(x, newRightHullPoints[i].y + dy1) + pivotOffset, waterlineAngle).y;

					if (Mathf.Abs(dy1) < Mathf.Abs(dy2) && dy1 > 0) {
						//print("Returning");
						print(GetUnrotatedPoint(new Vector2(x, dy1)));
						return GetUnrotatedPoint(new Vector2(x, dy1)).y;
						//return newRightHullPoints[i].y + dy1;
					} else if (Mathf.Abs(dy2) < Mathf.Abs(dy1) && dy2 > 0) {
						//print("Returning");
						//return newRightHullPoints[i].y + dy2;
					} else if (Mathf.Abs(dy1) == Mathf.Abs(dy2)) {
						return newRightHullPoints[i].y + Mathf.Abs(dy1);
					}

				} else {
					area += sectionArea;
				}
			}
		} else if (index != 0 && shipRoll < 0) { // CW
			for (int i = pointsToTop - 1; i < newLeftHullPoints.Count - 1; i++) {

				float sectionArea = (newLeftHullPoints[i].x + newLeftHullPoints[i + 1].x) / 2 * (newLeftHullPoints[i + 1].y - newLeftHullPoints[i].y);

				if ((newLeftHullPoints[i].x >= 0f && newLeftHullPoints[i + 1].x < 0f)) {
					sectionArea /= 2;
				} else if (newLeftHullPoints[i].x < 0f && newLeftHullPoints[i + 1].x < 0f) {
					sectionArea = -sectionArea;
				}

				if ((area + sectionArea) > displacedVolumePerLength) {
					float remainingArea = displacedVolumePerLength - area;

					float slope = (newLeftHullPoints[i + 1].x - newLeftHullPoints[i].x) / (newLeftHullPoints[i + 1].y - newLeftHullPoints[i].y);

					// Quadratic Formula 
					float dy1 = (-newLeftHullPoints[i].x + Mathf.Sqrt(newLeftHullPoints[i].x * newLeftHullPoints[i].x - 2 * slope * -remainingArea)) / (slope);
					//float dy2 = (-newLeftHullPoints[i].x - Mathf.Sqrt(newLeftHullPoints[i].x * newLeftHullPoints[i].x - 2 * slope * -remainingArea)) / (slope);

					float x = newLeftHullPoints[i].x + (dy1) * (newLeftHullPoints[i + 1].x - newLeftHullPoints[i].x) / (newLeftHullPoints[i + 1].y - newLeftHullPoints[i].y);
					print(GetUnrotatedPoint(new Vector2(-(x - pivotOffset.x), newLeftHullPoints[i].y + dy1 + pivotOffset.y), waterlineAngle));
					return GetUnrotatedPoint(new Vector2(-(x - pivotOffset.x), newLeftHullPoints[i].y + dy1 + pivotOffset.y), waterlineAngle).y;

					if (Mathf.Abs(dy1) < Mathf.Abs(dy2) && dy1 > 0) {
						//print("Returning");
						return GetUnrotatedPoint(new Vector2(x, newLeftHullPoints[i].y + dy1)).y;
						//return newLeftHullPoints[i].y + dy1;
					} else if (Mathf.Abs(dy2) < Mathf.Abs(dy1) && dy2 > 0) {
						//print("Returning");
						//return newLeftHullPoints[i].y + dy2;
					} else if (Mathf.Abs(dy1) == Mathf.Abs(dy2)) {
						return newLeftHullPoints[i].y + Mathf.Abs(dy1);
					}

				} else {
					area += sectionArea;
				}
			}
		}

		return -1f;
	}
	*/
	private float GetSubmergedArea(float waterline, float waterlineAngleWithRoll, HullSection hullSection, float overallMinY, out Vector2 centerOfBuoyancy) {
		centerOfBuoyancy = Vector2.zero;

		if (waterline <= hullSection.bottomY * hullHeight + (hullHeight / 2f)) { // Out of water
			return 0f;
		}

		float area = 0f;

		Vector2[] rightHullPoints = GetRotatedHullPoints(GetUnnormalizedHullPoints(hullSection), waterlineAngleWithRoll);
		Vector2[] leftHullPoints = GetRotatedHullPoints(GetLeftHullPoints(GetUnnormalizedHullPoints(hullSection)), waterlineAngleWithRoll);

		// Find lowest y-position and its index
		//float minY = hull.hullPoints[0].y;
		//float maxY = hull.hullPoints[hull.hullPoints.Count - 1].y;
		float localMinY = hullSection.topY * hullHeight;
		float localMaxY = hullSection.bottomY * hullHeight;

		int localMinIndex = 0;
		for (int i = 0; i < rightHullPoints.Length; i++) {
			if (rightHullPoints[i].y < localMinY) {
				localMinY = rightHullPoints[i].y;
				localMinIndex = i;
			}
			if (rightHullPoints[i].y > localMaxY) {
				localMaxY = rightHullPoints[i].y;
			}
		}
		for (int i = 0; i < leftHullPoints.Length; i++) {
			if (leftHullPoints[i].y < localMinY) {
				localMinY = leftHullPoints[i].y;
				localMinIndex = i;
			}
			if (leftHullPoints[i].y > localMaxY) {
				localMaxY = leftHullPoints[i].y;
			}
		}		

		// Return appropriate values if fully submerged
		if (waterline >= localMaxY - overallMinY) {
			centerOfBuoyancy = hullSection.submergedCenterOfBuoyancy;
			return hullSection.totalArea;
		}

		// Redefine hull positions by setting a new origin at lowest point and flipping the left sides x-positions

		List<Vector2> newRightHullPoints = new List<Vector2>();
		List<Vector2> newLeftHullPoints = new List<Vector2>();

		Vector2 pivotOffset = Vector2.zero;

		// CCW Ship Roll is positive
		if (localMinIndex != 0 && waterlineAngleWithRoll >= 0) { // CCW Roll
			pivotOffset = new Vector2(leftHullPoints[localMinIndex].x, overallMinY);
			leftHullPoints[localMinIndex] = new Vector2(0f, 0f);

			newRightHullPoints.Add(leftHullPoints[localMinIndex]);
			for (int i = localMinIndex - 1; i > 0; i--) {
				newRightHullPoints.Add(new Vector2((leftHullPoints[i].x - pivotOffset.x), leftHullPoints[i].y - pivotOffset.y));
			}
			foreach (Vector2 point in rightHullPoints) {
				newRightHullPoints.Add(new Vector2((point.x - pivotOffset.x), point.y - pivotOffset.y));
			}

			newLeftHullPoints.Add(leftHullPoints[localMinIndex]);
			for (int i = localMinIndex + 1; i < leftHullPoints.Length; i++) {
				newLeftHullPoints.Add(new Vector2(-(leftHullPoints[i].x - pivotOffset.x), leftHullPoints[i].y - pivotOffset.y));
			}

		} else if (localMinIndex != 0 && waterlineAngleWithRoll < 0) { // CW Roll
			pivotOffset = new Vector2(rightHullPoints[localMinIndex].x, overallMinY);

			rightHullPoints[localMinIndex] = new Vector2(0f, 0f);

			newLeftHullPoints.Add(rightHullPoints[localMinIndex]);
			for (int i = localMinIndex - 1; i > 0; i--) {
				newLeftHullPoints.Add(new Vector2(-(rightHullPoints[i].x - pivotOffset.x), rightHullPoints[i].y - pivotOffset.y));
			}
			foreach (Vector2 point in leftHullPoints) {
				newLeftHullPoints.Add(new Vector2(-(point.x - pivotOffset.x), point.y - pivotOffset.y));
			}

			newRightHullPoints.Add(rightHullPoints[localMinIndex]);
			for (int i = localMinIndex + 1; i < rightHullPoints.Length; i++) {
				newRightHullPoints.Add(rightHullPoints[i] - pivotOffset);
			}
		} else {
			pivotOffset = new Vector2(0f, overallMinY);
			foreach (Vector2 point in rightHullPoints) {
				newRightHullPoints.Add(new Vector2(point.x, point.y - pivotOffset.y));
			}

			foreach (Vector2 point in leftHullPoints) {
				newLeftHullPoints.Add(new Vector2(-point.x, point.y - pivotOffset.y));
			}
		}

		//PrintList(newRightHullPoints);
		//PrintList(newLeftHullPoints);

		// Add deck point
		bool crossesY = false;
		Vector2 deckPoint = new Vector2(0, newLeftHullPoints[newLeftHullPoints.Count - 1].y + ((newRightHullPoints[newRightHullPoints.Count - 1].y - newLeftHullPoints[newLeftHullPoints.Count - 1].y) / (newRightHullPoints[newRightHullPoints.Count - 1].x + newLeftHullPoints[newLeftHullPoints.Count - 1].x) * newLeftHullPoints[newLeftHullPoints.Count - 1].x));

		if (waterlineAngleWithRoll > 0 && newLeftHullPoints.Count > 1) {
			for (int i = 0; i < newRightHullPoints.Count - 1; i++) {
				if (newRightHullPoints[i + 1].x < 0f && newRightHullPoints[i].x > 0f) { // Hull line crosses y-axis
					float yIntercept = newRightHullPoints[i].y - newRightHullPoints[i].x * (newRightHullPoints[i + 1].y - newRightHullPoints[i].y) / (newRightHullPoints[i + 1].x - newRightHullPoints[i].x);
					deckPoint = new Vector2(-((yIntercept - newLeftHullPoints[newLeftHullPoints.Count - 1].y) * (newRightHullPoints[newRightHullPoints.Count - 1].x + newLeftHullPoints[newLeftHullPoints.Count - 1].x) / (newRightHullPoints[newRightHullPoints.Count - 1].y - newLeftHullPoints[newLeftHullPoints.Count - 1].y) - newLeftHullPoints[newLeftHullPoints.Count - 1].x), yIntercept);
					crossesY = true;
					break;
				}
			}
			
			newLeftHullPoints.Add(deckPoint);
		} else if (waterlineAngleWithRoll < 0 && newRightHullPoints.Count > 1) {
			for (int i = 0; i < newLeftHullPoints.Count - 1; i++) {
				if (newLeftHullPoints[i + 1].x < 0f && newLeftHullPoints[i].x > 0f) { // Hull line crosses y-axis
					float yIntercept = newLeftHullPoints[i].y - newLeftHullPoints[i].x * (newLeftHullPoints[i + 1].y - newLeftHullPoints[i].y) / (newLeftHullPoints[i + 1].x - newLeftHullPoints[i].x);
					deckPoint = new Vector2(((yIntercept - newLeftHullPoints[newLeftHullPoints.Count - 1].y) * (newRightHullPoints[newRightHullPoints.Count - 1].x + newLeftHullPoints[newLeftHullPoints.Count - 1].x) / (newRightHullPoints[newRightHullPoints.Count - 1].y - newLeftHullPoints[newLeftHullPoints.Count - 1].y) - newLeftHullPoints[newLeftHullPoints.Count - 1].x), yIntercept);
					crossesY = true;
					break;
				}
			}

			newRightHullPoints.Add(deckPoint);
		}
		//PrintList(newRightHullPoints);
		//PrintList(newLeftHullPoints);

		// Interpolate points for equal step intervals

		for (int i = 1; i < newLeftHullPoints.Count; i++) {
			for (int j = 1; j < newRightHullPoints.Count; j++) {
				if (newLeftHullPoints[i].y < newRightHullPoints[j].y) {
					newRightHullPoints.Insert(j, new Vector2((newLeftHullPoints[i].y - newRightHullPoints[j - 1].y) * (newRightHullPoints[j].x - newRightHullPoints[j - 1].x) / (newRightHullPoints[j].y - newRightHullPoints[j - 1].y) + newRightHullPoints[j - 1].x, newLeftHullPoints[i].y));
					break;
				} else if (newLeftHullPoints[i].y == newRightHullPoints[j].y) {
					break;
				}
			}
		}

		for (int i = 1; i < newRightHullPoints.Count; i++) {
			for (int j = 1; j < newLeftHullPoints.Count; j++) {
				if (newRightHullPoints[i].y < newLeftHullPoints[j].y) {
					newLeftHullPoints.Insert(j, new Vector2((newRightHullPoints[i].y - newLeftHullPoints[j - 1].y) * (newLeftHullPoints[j].x - newLeftHullPoints[j - 1].x) / (newLeftHullPoints[j].y - newLeftHullPoints[j - 1].y) + newLeftHullPoints[j - 1].x, newRightHullPoints[i].y));
					break;
				} else if (newRightHullPoints[i].y == newLeftHullPoints[j].y) {
					break;
				}
			}
		}
		//PrintList(newRightHullPoints);
		//PrintList(newLeftHullPoints);

		// Adjust points for deckline
		List<Vector2> originalNewHullPoints = new List<Vector2>();
		List<float> aboveHalfDeckPoints = new List<float>();

		if (waterlineAngleWithRoll > 0) {
			for (int i = newLeftHullPoints.Count - 1; i < newRightHullPoints.Count; i++) {
				originalNewHullPoints.Add(newRightHullPoints[i]);
				//aboveHalfDeckPoints.Add(((newRightHullPoints[i].y - newLeftHullPoints[newLeftHullPoints.Count - 1].y) * (newRightHullPoints[newRightHullPoints.Count - 1].x + newLeftHullPoints[newLeftHullPoints.Count - 1].x) / (newRightHullPoints[newRightHullPoints.Count - 1].y - newLeftHullPoints[newLeftHullPoints.Count - 1].y) - newLeftHullPoints[newLeftHullPoints.Count - 1].x));
				newRightHullPoints[i] = new Vector2(newRightHullPoints[i].x - ((newRightHullPoints[i].y - newLeftHullPoints[newLeftHullPoints.Count - 1].y) * (newRightHullPoints[newRightHullPoints.Count - 1].x + newLeftHullPoints[newLeftHullPoints.Count - 1].x) / (newRightHullPoints[newRightHullPoints.Count - 1].y - newLeftHullPoints[newLeftHullPoints.Count - 1].y) - newLeftHullPoints[newLeftHullPoints.Count - 1].x), newRightHullPoints[i].y);
				aboveHalfDeckPoints.Add(-(newRightHullPoints[i].x - originalNewHullPoints[i - (newLeftHullPoints.Count - 1)].x));
			}
		} else if (waterlineAngleWithRoll < 0) {
			for (int i = newRightHullPoints.Count - 1; i < newLeftHullPoints.Count; i++) {
				originalNewHullPoints.Add(newLeftHullPoints[i]);
				//aboveHalfDeckPoints.Add(((newLeftHullPoints[i].y - newRightHullPoints[newRightHullPoints.Count - 1].y) * (newLeftHullPoints[newLeftHullPoints.Count - 1].x + newRightHullPoints[newRightHullPoints.Count - 1].x) / (newLeftHullPoints[newLeftHullPoints.Count - 1].y - newRightHullPoints[newRightHullPoints.Count - 1].y) - newRightHullPoints[newRightHullPoints.Count - 1].x));
				newLeftHullPoints[i] = new Vector2(newLeftHullPoints[i].x - ((newLeftHullPoints[i].y - newRightHullPoints[newRightHullPoints.Count - 1].y) * (newLeftHullPoints[newLeftHullPoints.Count - 1].x + newRightHullPoints[newRightHullPoints.Count - 1].x) / (newLeftHullPoints[newLeftHullPoints.Count - 1].y - newRightHullPoints[newRightHullPoints.Count - 1].y) - newRightHullPoints[newRightHullPoints.Count - 1].x), newLeftHullPoints[i].y);
				aboveHalfDeckPoints.Add(-(newLeftHullPoints[i].x - originalNewHullPoints[i - (newRightHullPoints.Count - 1)].x));
			}
		}

		// Get the number of points on the shortest half of hull

		int pointsToTop = newLeftHullPoints.Count;
		if (newRightHullPoints.Count < pointsToTop) { pointsToTop = newRightHullPoints.Count; }

		//PrintList(newRightHullPoints);
		//PrintList(newLeftHullPoints);

		// Iteratively add section areas to find waterline

		for (int i = 0; i < pointsToTop - 1; i++) {

			if (newRightHullPoints[i + 1].y > waterline) {
				// Calculate remaining area
				//Debug.Log(centerOfBuoyancy + pivotOffset);
				//Debug.Log(centerOfBuoyancy / area + pivotOffset);
				//Debug.Log(GetUnrotatedPoint(centerOfBuoyancy / area + pivotOffset, waterlineAngle));
				float xRight = 0f;
				float xLeft = 0f;


				if (crossesY && i == pointsToTop - 2 && waterlineAngleWithRoll > 0) {
					xRight = newRightHullPoints[i].x + (waterline - newRightHullPoints[i].y) * (-newRightHullPoints[i].x) / (newRightHullPoints[i + 1].y - newRightHullPoints[i].y);
					xLeft = newLeftHullPoints[i].x + (waterline - newLeftHullPoints[i].y) * (newLeftHullPoints[i + 1].x - newLeftHullPoints[i].x) / (newLeftHullPoints[i + 1].y - newLeftHullPoints[i].y);

				} else if (crossesY && i == pointsToTop - 2 && waterlineAngleWithRoll < 0) {
					xRight = newRightHullPoints[i].x + (waterline - newRightHullPoints[i].y) * (newRightHullPoints[i + 1].x - newRightHullPoints[i].x) / (newRightHullPoints[i + 1].y - newRightHullPoints[i].y);
					xLeft = newLeftHullPoints[i].x + (waterline - newLeftHullPoints[i].y) * (-newLeftHullPoints[i].x) / (newLeftHullPoints[i + 1].y - newLeftHullPoints[i].y);

				} else {
					xRight = newRightHullPoints[i].x + (waterline - newRightHullPoints[i].y) * (newRightHullPoints[i + 1].x - newRightHullPoints[i].x) / (newRightHullPoints[i + 1].y - newRightHullPoints[i].y);
					xLeft = newLeftHullPoints[i].x + (waterline - newLeftHullPoints[i].y) * (newLeftHullPoints[i + 1].x - newLeftHullPoints[i].x) / (newLeftHullPoints[i + 1].y - newLeftHullPoints[i].y);
				}

				float remainingAreaLeft = (newLeftHullPoints[i].x + xLeft) * (waterline - newLeftHullPoints[i].y) / 2;
				float remainingAreaRight = (newRightHullPoints[i].x + xRight) * (waterline - newRightHullPoints[i].y) / 2;

				Vector2 sectionAreaCenterRight = new Vector2((newRightHullPoints[i].x * newRightHullPoints[i].x + newRightHullPoints[i].x * xRight + xRight * xRight) / (3 * (newRightHullPoints[i].x + xRight)), newRightHullPoints[i].y + (newRightHullPoints[i].x + 2 * xRight) * (waterline - newRightHullPoints[i].y) / (3 * (newRightHullPoints[i].x + xRight)));
				Vector2 sectionAreaCenterLeft = new Vector2(-(newLeftHullPoints[i].x * newLeftHullPoints[i].x + newLeftHullPoints[i].x * xLeft + xLeft * xLeft) / (3 * (newLeftHullPoints[i].x + xLeft)), newLeftHullPoints[i].y + (newLeftHullPoints[i].x + 2 * xLeft) * (waterline - newLeftHullPoints[i].y) / (3 * (newLeftHullPoints[i].x + xLeft)));

				centerOfBuoyancy += sectionAreaCenterRight * remainingAreaRight + sectionAreaCenterLeft * remainingAreaLeft;

				area += remainingAreaLeft + remainingAreaRight;

				//Debug.Log(GetUnrotatedPoint((centerOfBuoyancy + pivotOffset) / area, waterlineAngle));
				centerOfBuoyancy = GetUnrotatedPoint(centerOfBuoyancy / area + pivotOffset, waterlineAngleWithRoll);
				return area;

			} else if (newRightHullPoints[i].y == waterline) {
				//print(newRightHullPoints[i].y);
				centerOfBuoyancy = GetUnrotatedPoint(centerOfBuoyancy / area + pivotOffset, waterlineAngleWithRoll);
				return area;
			} else {

				float sectionAreaLeft = 0f;
				float sectionAreaRight = 0f;

				Vector2 sectionAreaCenterRight = Vector2.zero;
				Vector2 sectionAreaCenterLeft = Vector2.zero;

				if (crossesY && i == pointsToTop - 2 && waterlineAngleWithRoll > 0) {
					//print(1);
					sectionAreaRight = newRightHullPoints[i].x / 2 * (newRightHullPoints[i + 1].y - newRightHullPoints[i].y);
					sectionAreaCenterRight = new Vector2((newRightHullPoints[i].x * newRightHullPoints[i].x) / (3 * newRightHullPoints[i].x), newRightHullPoints[i].y + (newRightHullPoints[i].x) * (newRightHullPoints[i + 1].y - newRightHullPoints[i].y) / (3 * (newRightHullPoints[i].x)));

					sectionAreaLeft = (newLeftHullPoints[i].x + newLeftHullPoints[i + 1].x) / 2 * (newLeftHullPoints[i + 1].y - newLeftHullPoints[i].y);
					sectionAreaCenterLeft = new Vector2(-(newLeftHullPoints[i].x * newLeftHullPoints[i].x + newLeftHullPoints[i].x * newLeftHullPoints[i + 1].x + newLeftHullPoints[i + 1].x * newLeftHullPoints[i + 1].x) / (3 * (newLeftHullPoints[i].x + newLeftHullPoints[i + 1].x)), newLeftHullPoints[i].y + (newLeftHullPoints[i].x + 2 * newLeftHullPoints[i + 1].x) * (newLeftHullPoints[i + 1].y - newLeftHullPoints[i].y) / (3 * (newLeftHullPoints[i].x + newLeftHullPoints[i + 1].x)));
					
				} else if (crossesY && i == pointsToTop - 2 && waterlineAngleWithRoll < 0) {
					//print(2);
					sectionAreaLeft = newLeftHullPoints[i].x / 2 * (newLeftHullPoints[i + 1].y - newLeftHullPoints[i].y);
					sectionAreaCenterLeft = new Vector2(-(newLeftHullPoints[i].x * newLeftHullPoints[i].x) / (3 * (newLeftHullPoints[i].x)), newLeftHullPoints[i].y + (newLeftHullPoints[i].x) * (newLeftHullPoints[i + 1].y - newLeftHullPoints[i].y) / (3 * (newLeftHullPoints[i].x)));

					sectionAreaRight = (newRightHullPoints[i].x + newRightHullPoints[i + 1].x) / 2 * (newRightHullPoints[i + 1].y - newRightHullPoints[i].y);
					sectionAreaCenterRight = new Vector2((newRightHullPoints[i].x * newRightHullPoints[i].x + newRightHullPoints[i].x * newRightHullPoints[i + 1].x + newRightHullPoints[i + 1].x * newRightHullPoints[i + 1].x) / (3 * (newRightHullPoints[i].x + newRightHullPoints[i + 1].x)), newRightHullPoints[i].y + (newRightHullPoints[i].x + 2 * newRightHullPoints[i + 1].x) * (newRightHullPoints[i + 1].y - newRightHullPoints[i].y) / (3 * (newRightHullPoints[i].x + newRightHullPoints[i + 1].x)));
					
				} else {
					//print(3);
					sectionAreaLeft = (newLeftHullPoints[i].x + newLeftHullPoints[i + 1].x) / 2 * (newLeftHullPoints[i + 1].y - newLeftHullPoints[i].y);
					sectionAreaRight = (newRightHullPoints[i].x + newRightHullPoints[i + 1].x) / 2 * (newRightHullPoints[i + 1].y - newRightHullPoints[i].y);

					sectionAreaCenterRight = new Vector2((newRightHullPoints[i].x * newRightHullPoints[i].x + newRightHullPoints[i].x * newRightHullPoints[i + 1].x + newRightHullPoints[i + 1].x * newRightHullPoints[i + 1].x) / (3 * (newRightHullPoints[i].x + newRightHullPoints[i + 1].x)), newRightHullPoints[i].y + (newRightHullPoints[i].x + 2 * newRightHullPoints[i + 1].x) * (newRightHullPoints[i + 1].y - newRightHullPoints[i].y) / (3 * (newRightHullPoints[i].x + newRightHullPoints[i + 1].x)));
					sectionAreaCenterLeft = new Vector2(-(newLeftHullPoints[i].x * newLeftHullPoints[i].x + newLeftHullPoints[i].x * newLeftHullPoints[i + 1].x + newLeftHullPoints[i + 1].x * newLeftHullPoints[i + 1].x) / (3 * (newLeftHullPoints[i].x + newLeftHullPoints[i + 1].x)), newLeftHullPoints[i].y + (newLeftHullPoints[i].x + 2 * newLeftHullPoints[i + 1].x) * (newLeftHullPoints[i + 1].y - newLeftHullPoints[i].y) / (3 * (newLeftHullPoints[i].x + newLeftHullPoints[i + 1].x)));
					
				}

				area += sectionAreaLeft + sectionAreaRight;
				//print(sectionAreaCenterRight);
				//print(sectionAreaCenterLeft);
				//print(sectionAreaRight);
				//print(sectionAreaLeft);
				centerOfBuoyancy += sectionAreaCenterRight * sectionAreaRight + sectionAreaCenterLeft * sectionAreaLeft;
				//print(sectionAreaCenterRight * sectionAreaRight + sectionAreaCenterLeft * sectionAreaLeft);
				/*Vector3 buoyantForcePos = transform.TransformPoint(new Vector3(GetUnrotatedPoint(sectionAreaCenterRight + pivotOffset, waterlineAngle).x / transform.localScale.x, GetUnrotatedPoint(sectionAreaCenterRight + pivotOffset, waterlineAngle).y / transform.localScale.y - 0.5f, (1f / longitudinalBuoyantForceSplit * 0 + 1f / (longitudinalBuoyantForceSplit * 2)) - 0.5f));
				Debug.DrawRay(buoyantForcePos, Vector3.up * 0.1f, UnityEngine.Color.blue, Time.deltaTime * 1);

				buoyantForcePos = transform.TransformPoint(new Vector3(GetUnrotatedPoint(sectionAreaCenterLeft + pivotOffset, waterlineAngle).x / transform.localScale.x, GetUnrotatedPoint(sectionAreaCenterLeft + pivotOffset, waterlineAngle).y / transform.localScale.y - 0.5f, (1f / longitudinalBuoyantForceSplit * 0 + 1f / (longitudinalBuoyantForceSplit * 2)) - 0.5f));
				Debug.DrawRay(buoyantForcePos, Vector3.up * 0.1f, UnityEngine.Color.blue, Time.deltaTime * 1);*/

				//Debug.Log(sectionAreaCenterRight);
				//Debug.Log(sectionAreaCenterLeft);

				//centerOfBuoyancy += sectionAreaCenterLeft * sectionAreaLeft;
			}
		}

		// One side is now submerged

		if (waterlineAngleWithRoll > 0) { // CCW
			//Debug.Log("CCW");
			for (int i = pointsToTop - 1; i < newRightHullPoints.Count - 1; i++) {

				if (newRightHullPoints[i + 1].y > waterline) {
					// Calculate remaining area

					float x = newRightHullPoints[i].x + (waterline - newRightHullPoints[i].y) * (newRightHullPoints[i + 1].x - newRightHullPoints[i].x) / (newRightHullPoints[i + 1].y - newRightHullPoints[i].y);

					float remainingArea = (newRightHullPoints[i].x + x) * (waterline - newRightHullPoints[i].y) / 2;

					// Calculate center of trapezoid polygon
					// https://en.wikipedia.org/wiki/Centroid#Of_a_polygon
					float xPos = 0f;
					float yPos = 0f;

					float xRight = originalNewHullPoints[i - (pointsToTop - 1)].x + (waterline - newRightHullPoints[i].y) * (originalNewHullPoints[(i + 1) - (pointsToTop - 1)].x - originalNewHullPoints[i - (pointsToTop - 1)].x) / (newRightHullPoints[i + 1].y - newRightHullPoints[i].y);
					float xLeft = aboveHalfDeckPoints[i - (pointsToTop - 1)] + (waterline - newRightHullPoints[i].y) * (aboveHalfDeckPoints[(i + 1) - (pointsToTop - 1)] - aboveHalfDeckPoints[i - (pointsToTop - 1)]) / (newRightHullPoints[i + 1].y - newRightHullPoints[i].y);
					
					/*Debug.Log(xRight);
					Debug.Log(xLeft);
					print(waterline);
					print(newRightHullPoints[i].y);
					print(aboveHalfDeckPoints[i - (pointsToTop - 1)]);
					print(originalNewHullPoints[i - (pointsToTop - 1)].x);*/

					xPos += (aboveHalfDeckPoints[i - (pointsToTop - 1)] + originalNewHullPoints[i - (pointsToTop - 1)].x) * (aboveHalfDeckPoints[i - (pointsToTop - 1)] * newRightHullPoints[i].y - originalNewHullPoints[i - (pointsToTop - 1)].x * newRightHullPoints[i].y);
					xPos += (originalNewHullPoints[i - (pointsToTop - 1)].x + xRight) * (originalNewHullPoints[i - (pointsToTop - 1)].x * waterline - xRight * newRightHullPoints[i].y);
					xPos += (xRight + xLeft) * (xRight * waterline - xLeft * waterline);
					xPos += (xLeft + aboveHalfDeckPoints[i - (pointsToTop - 1)]) * (xLeft * newRightHullPoints[i].y - aboveHalfDeckPoints[i - (pointsToTop - 1)] * waterline);
					xPos = xPos / (6 * remainingArea) + aboveHalfDeckPoints[i - (pointsToTop - 1)];

					yPos += (newRightHullPoints[i].y + newRightHullPoints[i].y) * (aboveHalfDeckPoints[i - (pointsToTop - 1)] * newRightHullPoints[i].y - originalNewHullPoints[i - (pointsToTop - 1)].x * newRightHullPoints[i].y);
					yPos += (newRightHullPoints[i].y + waterline) * (originalNewHullPoints[i - (pointsToTop - 1)].x * waterline - xRight * newRightHullPoints[i].y);
					yPos += (waterline + waterline) * (xRight * waterline - xLeft * waterline);
					yPos += (waterline + newRightHullPoints[i].y) * (xLeft * newRightHullPoints[i].y - aboveHalfDeckPoints[i - (pointsToTop - 1)] * waterline);
					yPos = yPos / (6 * remainingArea) + newRightHullPoints[i].y;
					//Debug.Log(new Vector2(xPos, yPos));
					//Debug.Log(centerOfBuoyancy);
					//print(new Vector2(xPos, yPos) * remainingArea);
					centerOfBuoyancy += new Vector2(xPos, yPos) * remainingArea;
					//Debug.Log(centerOfBuoyancy);

					area += remainingArea;

					//centerOfBuoyancy += sectionAreaCenter * remainingArea;
					//print(centerOfBuoyancy / area + pivotOffset);
					//print(area);
					//print(pivotOffset);
					centerOfBuoyancy = GetUnrotatedPoint(centerOfBuoyancy / area + pivotOffset, waterlineAngleWithRoll);
					return area;

				} else if (newRightHullPoints[i].y == waterline) {

					centerOfBuoyancy = GetUnrotatedPoint(centerOfBuoyancy / area + pivotOffset, waterlineAngleWithRoll);
					return area;
				} else {

					float sectionArea = (newRightHullPoints[i].x + newRightHullPoints[i + 1].x) / 2 * (newRightHullPoints[i + 1].y - newRightHullPoints[i].y);

					float xPos = 0f;
					float yPos = 0f;

					xPos += (aboveHalfDeckPoints[i - (pointsToTop - 1)] + originalNewHullPoints[i - (pointsToTop - 1)].x) * (aboveHalfDeckPoints[i - (pointsToTop - 1)] * newRightHullPoints[i].y - originalNewHullPoints[i - (pointsToTop - 1)].x * newRightHullPoints[i].y);
					xPos += (originalNewHullPoints[i - (pointsToTop - 1)].x + originalNewHullPoints[(i + 1) - (pointsToTop - 1)].x) * (originalNewHullPoints[i - (pointsToTop - 1)].x * newRightHullPoints[i + 1].y - originalNewHullPoints[(i + 1) - (pointsToTop - 1)].x * newRightHullPoints[i].y);
					xPos += (originalNewHullPoints[(i + 1) - (pointsToTop - 1)].x + aboveHalfDeckPoints[(i + 1) - (pointsToTop - 1)]) * (originalNewHullPoints[(i + 1) - (pointsToTop - 1)].x * newRightHullPoints[i + 1].y - aboveHalfDeckPoints[(i + 1) - (pointsToTop - 1)] * newRightHullPoints[i + 1].y);
					xPos += (aboveHalfDeckPoints[(i + 1) - (pointsToTop - 1)] + aboveHalfDeckPoints[i - (pointsToTop - 1)]) * (aboveHalfDeckPoints[(i + 1) - (pointsToTop - 1)] * newRightHullPoints[i].y - aboveHalfDeckPoints[i - (pointsToTop - 1)] * newRightHullPoints[i + 1].y);
					xPos = xPos / (6 * sectionArea) + aboveHalfDeckPoints[i - (pointsToTop - 1)];

					yPos += (newRightHullPoints[i].y + newRightHullPoints[i].y) * (aboveHalfDeckPoints[i - (pointsToTop - 1)] * newRightHullPoints[i].y - originalNewHullPoints[i - (pointsToTop - 1)].x * newRightHullPoints[i].y);
					yPos += (newRightHullPoints[i].y + newRightHullPoints[i + 1].y) * (originalNewHullPoints[i - (pointsToTop - 1)].x * newRightHullPoints[i + 1].y - originalNewHullPoints[(i + 1) - (pointsToTop - 1)].x * newRightHullPoints[i].y);
					yPos += (newRightHullPoints[i + 1].y + newRightHullPoints[i + 1].y) * (originalNewHullPoints[(i + 1) - (pointsToTop - 1)].x * newRightHullPoints[i + 1].y - aboveHalfDeckPoints[(i + 1) - (pointsToTop - 1)] * newRightHullPoints[i + 1].y);
					yPos += (newRightHullPoints[i + 1].y + newRightHullPoints[i].y) * (aboveHalfDeckPoints[(i + 1) - (pointsToTop - 1)] * newRightHullPoints[i].y - aboveHalfDeckPoints[i - (pointsToTop - 1)] * newRightHullPoints[i + 1].y);
					yPos = yPos / (6 * sectionArea) + newRightHullPoints[i].y;

					/*Debug.Log(xPos);
					Debug.Log(yPos);
					print(waterline);
					print(newRightHullPoints[i].y);
					print(aboveHalfDeckPoints[i - (pointsToTop - 1)]);
					print(originalNewHullPoints[i - (pointsToTop - 1)].x);*/

					centerOfBuoyancy += new Vector2(xPos, yPos) * sectionArea;

					area += sectionArea;
				}
			}
		} else if (waterlineAngleWithRoll < 0) { // CW
			//Debug.Log("CW");
			for (int i = pointsToTop - 1; i < newLeftHullPoints.Count - 1; i++) {

				if (newLeftHullPoints[i + 1].y > waterline) {
					// Calculate remaining area

					float x = newLeftHullPoints[i].x + (waterline - newLeftHullPoints[i].y) * (newLeftHullPoints[i + 1].x - newLeftHullPoints[i].x) / (newLeftHullPoints[i + 1].y - newLeftHullPoints[i].y);

					float remainingArea = (newLeftHullPoints[i].x + x) * (waterline - newLeftHullPoints[i].y) / 2;

					// Calculate center of trapezoid polygon
					// https://en.wikipedia.org/wiki/Centroid#Of_a_polygon
					float xPos = 0f;
					float yPos = 0f;

					float xRight = originalNewHullPoints[i - (pointsToTop - 1)].x + (waterline - newLeftHullPoints[i].y) * (originalNewHullPoints[(i + 1) - (pointsToTop - 1)].x - originalNewHullPoints[i - (pointsToTop - 1)].x) / (newLeftHullPoints[i + 1].y - newLeftHullPoints[i].y);
					float xLeft = aboveHalfDeckPoints[i - (pointsToTop - 1)] + (waterline - newLeftHullPoints[i].y) * (aboveHalfDeckPoints[(i + 1) - (pointsToTop - 1)] - aboveHalfDeckPoints[i - (pointsToTop - 1)]) / (newLeftHullPoints[i + 1].y - newLeftHullPoints[i].y);

					/*Debug.Log(xRight);
					Debug.Log(xLeft);
					print(waterline);
					print(newLeftHullPoints[i].y);
					print(aboveHalfDeckPoints[i - (pointsToTop - 1)]);
					print(originalNewHullPoints[i - (pointsToTop - 1)].x);*/

					xPos += (aboveHalfDeckPoints[i - (pointsToTop - 1)] + originalNewHullPoints[i - (pointsToTop - 1)].x) * (aboveHalfDeckPoints[i - (pointsToTop - 1)] * newLeftHullPoints[i].y - originalNewHullPoints[i - (pointsToTop - 1)].x * newLeftHullPoints[i].y);
					xPos += (originalNewHullPoints[i - (pointsToTop - 1)].x + xRight) * (originalNewHullPoints[i - (pointsToTop - 1)].x * waterline - xRight * newLeftHullPoints[i].y);
					xPos += (xRight + xLeft) * (xRight * waterline - xLeft * waterline);
					xPos += (xLeft + aboveHalfDeckPoints[i - (pointsToTop - 1)]) * (xLeft * newLeftHullPoints[i].y - aboveHalfDeckPoints[i - (pointsToTop - 1)] * waterline);
					xPos = xPos / (6 * remainingArea) + aboveHalfDeckPoints[i - (pointsToTop - 1)];

					yPos += (newLeftHullPoints[i].y + newLeftHullPoints[i].y) * (aboveHalfDeckPoints[i - (pointsToTop - 1)] * newLeftHullPoints[i].y - originalNewHullPoints[i - (pointsToTop - 1)].x * newLeftHullPoints[i].y);
					yPos += (newLeftHullPoints[i].y + waterline) * (originalNewHullPoints[i - (pointsToTop - 1)].x * waterline - xRight * newLeftHullPoints[i].y);
					yPos += (waterline + waterline) * (xRight * waterline - xLeft * waterline);
					yPos += (waterline + newLeftHullPoints[i].y) * (xLeft * newLeftHullPoints[i].y - aboveHalfDeckPoints[i - (pointsToTop - 1)] * waterline);
					yPos = yPos / (6 * remainingArea) + newLeftHullPoints[i].y;

					centerOfBuoyancy += new Vector2(-xPos, yPos) * remainingArea;

					area += remainingArea;
					//print(new Vector2(-xPos, yPos) * remainingArea);

					centerOfBuoyancy = GetUnrotatedPoint(centerOfBuoyancy / area + pivotOffset, waterlineAngleWithRoll);
					return area;

				} else if (newLeftHullPoints[i].y == waterline) {

					centerOfBuoyancy = GetUnrotatedPoint(centerOfBuoyancy / area + pivotOffset, waterlineAngleWithRoll);
					return area;
				} else {

					float sectionArea = (newLeftHullPoints[i].x + newLeftHullPoints[i + 1].x) / 2 * (newLeftHullPoints[i + 1].y - newLeftHullPoints[i].y);

					float xPos = 0f;
					float yPos = 0f;

					xPos += (aboveHalfDeckPoints[i - (pointsToTop - 1)] + originalNewHullPoints[i - (pointsToTop - 1)].x) * (aboveHalfDeckPoints[i - (pointsToTop - 1)] * newLeftHullPoints[i].y - originalNewHullPoints[i - (pointsToTop - 1)].x * newLeftHullPoints[i].y);
					xPos += (originalNewHullPoints[i - (pointsToTop - 1)].x + originalNewHullPoints[(i + 1) - (pointsToTop - 1)].x) * (originalNewHullPoints[i - (pointsToTop - 1)].x * newLeftHullPoints[i + 1].y - originalNewHullPoints[(i + 1) - (pointsToTop - 1)].x * newLeftHullPoints[i].y);
					xPos += (originalNewHullPoints[(i + 1) - (pointsToTop - 1)].x + aboveHalfDeckPoints[(i + 1) - (pointsToTop - 1)]) * (originalNewHullPoints[(i + 1) - (pointsToTop - 1)].x * newLeftHullPoints[i + 1].y - aboveHalfDeckPoints[(i + 1) - (pointsToTop - 1)] * newLeftHullPoints[i + 1].y);
					xPos += (aboveHalfDeckPoints[(i + 1) - (pointsToTop - 1)] + aboveHalfDeckPoints[i - (pointsToTop - 1)]) * (aboveHalfDeckPoints[(i + 1) - (pointsToTop - 1)] * newLeftHullPoints[i].y - aboveHalfDeckPoints[i - (pointsToTop - 1)] * newLeftHullPoints[i + 1].y);
					xPos = xPos / (6 * sectionArea) + aboveHalfDeckPoints[i - (pointsToTop - 1)];

					yPos += (newLeftHullPoints[i].y + newLeftHullPoints[i].y) * (aboveHalfDeckPoints[i - (pointsToTop - 1)] * newLeftHullPoints[i].y - originalNewHullPoints[i - (pointsToTop - 1)].x * newLeftHullPoints[i].y);
					yPos += (newLeftHullPoints[i].y + newLeftHullPoints[i + 1].y) * (originalNewHullPoints[i - (pointsToTop - 1)].x * newLeftHullPoints[i + 1].y - originalNewHullPoints[(i + 1) - (pointsToTop - 1)].x * newLeftHullPoints[i].y);
					yPos += (newLeftHullPoints[i + 1].y + newLeftHullPoints[i + 1].y) * (originalNewHullPoints[(i + 1) - (pointsToTop - 1)].x * newLeftHullPoints[i + 1].y - aboveHalfDeckPoints[(i + 1) - (pointsToTop - 1)] * newLeftHullPoints[i + 1].y);
					yPos += (newLeftHullPoints[i + 1].y + newLeftHullPoints[i].y) * (aboveHalfDeckPoints[(i + 1) - (pointsToTop - 1)] * newLeftHullPoints[i].y - aboveHalfDeckPoints[i - (pointsToTop - 1)] * newLeftHullPoints[i + 1].y);
					yPos = yPos / (6 * sectionArea) + newLeftHullPoints[i].y;

					centerOfBuoyancy += new Vector2(-xPos, yPos) * sectionArea;

					area += sectionArea;
				}
			}
		}
		return area; // Fully submerged
	}

	private Vector2 GetPolygonCentroid(Vector2[] vertices, float area) {
		Vector2 centroid = Vector2.zero;

		for (int i = 0; i < vertices.Length - 1; i++) {
			centroid.x += (vertices[i].x + vertices[i + 1].x) * (vertices[i].x * vertices[i + 1].y - vertices[i + 1].x * vertices[i].y);
			centroid.y += (vertices[i].y + vertices[i + 1].y) * (vertices[i].x * vertices[i + 1].y - vertices[i + 1].x * vertices[i].y);
		}

		centroid.x /= 6 * area;
		centroid.y /= 6 * area;

		return centroid;
	}

	private Vector2 GetPolygonCentroid(Vector2[] vertices) {
		Vector2 centroid = Vector2.zero;

		for (int i = 0; i < vertices.Length - 1; i++) {
			centroid.x += (vertices[i].x + vertices[i + 1].x) * (vertices[i].x * vertices[i + 1].y - vertices[i + 1].x * vertices[i].y);
			centroid.y += (vertices[i].y + vertices[i + 1].y) * (vertices[i].x * vertices[i + 1].y - vertices[i + 1].x * vertices[i].y);
		}

		float area = GetPolygonArea(vertices);

		centroid.x /= 6 * area;
		centroid.y /= 6 * area;

		return centroid;
	}

	private float GetPolygonArea(Vector2[] vertices) {
		float area = 0;

		for (int i = 0; i < vertices.Length - 1; i++) {
			area += vertices[i].x * vertices[i + 1].y - vertices[i + 1].x * vertices[i].y;
		}

		return area / 2f;
	}

	public float GetTotalArea(HullSection hullSection) {

		float totalArea = 0f;
		float yOffset = hullSection.topY * hullHeight - hullSection.hullShape.hullPoints[hullSection.hullShape.hullPoints.Count - 1].y * hullSection.normalizedSectionHeight * hullHeight;

		for (int i = 0; i < hullSection.hullShape.hullPoints.Count - 1; i++) {
			// Area of left and right side of section
			float doubleSectionArea = ((hullSection.hullShape.hullPoints[i].x * hullSection.normalizedSectionWidth * hullWidth) + (hullSection.hullShape.hullPoints[i + 1].x * hullSection.normalizedSectionWidth * hullWidth)) * ((hullSection.hullShape.hullPoints[i + 1].y * hullSection.normalizedSectionHeight * hullHeight + yOffset) - (hullSection.hullShape.hullPoints[i].y * hullSection.normalizedSectionHeight * hullHeight + yOffset));
			totalArea += doubleSectionArea;
		}

		return totalArea;
	}

	public Vector2 GetSubmergedCenterOfBuoyancy(HullSection hullSection) {
		Vector2 center = Vector2.zero;
		float area = 0f;
		float yOffset = hullSection.topY * hullHeight - hullSection.hullShape.hullPoints[hullSection.hullShape.hullPoints.Count - 1].y * hullSection.normalizedSectionHeight * hullHeight;
		
		for (int i = 0; i < hullSection.hullShape.hullPoints.Count - 1; i++) {
			float sectionArea = ((hullSection.hullShape.hullPoints[i].x * hullSection.normalizedSectionWidth * hullWidth) + (hullSection.hullShape.hullPoints[i + 1].x * hullSection.normalizedSectionWidth * hullWidth)) / 2 * ((hullSection.hullShape.hullPoints[i + 1].y * hullSection.normalizedSectionHeight * hullHeight + yOffset) - (hullSection.hullShape.hullPoints[i].y * hullSection.normalizedSectionHeight * hullHeight + yOffset));
			Vector2 sectionCenter = new Vector2(0, (hullSection.hullShape.hullPoints[i].y * hullSection.normalizedSectionHeight * hullHeight + yOffset) + ((hullSection.hullShape.hullPoints[i].x * hullSection.normalizedSectionWidth * hullWidth) + 2 * (hullSection.hullShape.hullPoints[i + 1].x * hullSection.normalizedSectionWidth * hullWidth)) * ((hullSection.hullShape.hullPoints[i + 1].y * hullSection.normalizedSectionHeight * hullHeight + yOffset) - (hullSection.hullShape.hullPoints[i].y * hullSection.normalizedSectionHeight * hullHeight + yOffset)) / (3 * ((hullSection.hullShape.hullPoints[i].x * hullSection.normalizedSectionWidth * hullWidth) + (hullSection.hullShape.hullPoints[i + 1].x * hullSection.normalizedSectionWidth * hullWidth))));
			
			area += sectionArea;
			center += sectionCenter * sectionArea;
		}

		return center / area;
	}

	public Vector2[] GetUnnormalizedHullPoints(HullSection hullSection)
	{
		Vector2[] unnormalizedPoints = new Vector2[hullSection.hullShape.hullPoints.Count];

		float yOffset = hullSection.topY * hullHeight - hullSection.hullShape.hullPoints[unnormalizedPoints.Length - 1].y * hullSection.normalizedSectionHeight * hullHeight;
		for (int i = 0; i < unnormalizedPoints.Length; i++)
		{
			unnormalizedPoints[i] = new Vector2(hullSection.hullShape.hullPoints[i].x * hullSection.normalizedSectionWidth * hullWidth, hullSection.hullShape.hullPoints[i].y * hullSection.normalizedSectionHeight * hullHeight + yOffset);
		}

		return unnormalizedPoints;
	}

	public Vector2[] GetLeftHullPoints(Vector2[] hullSectionShape) {
		Vector2[] flippedPoints = new Vector2[hullSectionShape.Length];

		// Flipping about the y-axis
		for (int i = 0; i < flippedPoints.Length; i++) {
			flippedPoints[i] = (new Vector2(-hullSectionShape[i].x, hullSectionShape[i].y));
		}

		return flippedPoints;
	}

	private Vector2[] GetRotatedHullPoints(Vector2[] fixedHullPoints, float angle) {
        Vector2[] rotatedPoints = new Vector2[fixedHullPoints.Length];

        // 2D coordinate transformation about longitudinal axis
        for (int i = 0; i < rotatedPoints.Length; i++) {
			rotatedPoints[i] = GetRotatedPoint(fixedHullPoints[i], angle);
        }

        return rotatedPoints;
    }

	private Vector2 GetRotatedPoint(Vector2 point, float angle) {
		return new Vector2(Mathf.Cos((angle) * Mathf.Deg2Rad) * point.x - Mathf.Sin((angle) * Mathf.Deg2Rad) * point.y, Mathf.Sin((angle) * Mathf.Deg2Rad) * point.x + Mathf.Cos((angle) * Mathf.Deg2Rad) * point.y);
	}

	private Vector2 GetUnrotatedPoint(Vector2 point, float angle) {
		return new Vector2(Mathf.Cos((angle) * Mathf.Deg2Rad) * point.x + Mathf.Sin((angle) * Mathf.Deg2Rad) * point.y, -Mathf.Sin((angle) * Mathf.Deg2Rad) * point.x + Mathf.Cos((angle) * Mathf.Deg2Rad) * point.y);
	}

	public static float ErrorSquared(Vector2[] points, float m, float b) {
		float total = 0;
		foreach (Vector2 pt in points) {
			float dy = pt.y - (m * pt.x + b);
			total += dy * dy;
		}
		return total;
	}

	public static float FindLinearLeastSquaresFit(Vector2[] points, out float m, out float b) {
		// Perform the calculation.
		// Find the values S1, Sx, Sy, Sxx, and Sxy.
		float S1 = points.Length;
		float Sx = 0;
		float Sy = 0;
		float Sxx = 0;
		float Sxy = 0;
		foreach (Vector2 pt in points) {
			Sx += pt.x;
			Sy += pt.y;
			Sxx += pt.x * pt.x;
			Sxy += pt.x * pt.y;
		}

		// Solve for m and b.
		m = (Sxy * S1 - Sx * Sy) / (Sxx * S1 - Sx * Sx);
		b = (Sxy * Sx - Sy * Sxx) / (Sx * Sx - S1 * Sxx);

		return Mathf.Sqrt(ErrorSquared(points, m, b));
	}

	public float GetMass() {
        return rb.mass;
    }

    public void SetMass(float newMass) {
        rb.mass = newMass;

        if (rb.mass < 0f) {
            rb.mass = 0f;
        }
    }

	public float GetLength() {
		return hullLength;
	}

	public void SetLength(float newLength) {
		hullLength = newLength;

		if (hullLength < 0f) {
			hullLength = 0f;
		}
	}

	public float GetRoll() {
		return shipRoll;
	}

	public void SetRoll(float newRoll) {
		shipRoll = newRoll;
	}

	public static void PrintList(List<Vector2> list) {
		string s = "";
		foreach (Vector2 v in list) {
			s += v.ToString() + ", ";
		}
		Debug.Log(s);
	}

	private float GetSectionMinRotatedY(HullSection hullSection, float waterlineAngle)
	{
		Vector2[] rightHullPoints = GetRotatedHullPoints(GetUnnormalizedHullPoints(hullSection), waterlineAngle);
		Vector2[] leftHullPoints = GetRotatedHullPoints(GetLeftHullPoints(GetUnnormalizedHullPoints(hullSection)), waterlineAngle);

		float minY = hullSection.topY * hullHeight;

		for (int i = 0; i < rightHullPoints.Length; i++)
		{
			if (rightHullPoints[i].y < minY)
			{
				minY = rightHullPoints[i].y;
			}
		}
		for (int i = 0; i < leftHullPoints.Length; i++)
		{
			if (leftHullPoints[i].y < minY)
			{
				minY = leftHullPoints[i].y;
			}
		}

		return minY;
	}

	private void OnDestroy() {
		waterSurfaceHeightSearch.Destroy();
	}
}