using System.Collections;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;

[RequireComponent(typeof(Rigidbody))]
public class BuoyantHull : MonoBehaviour
{
    public HullShape hull;
	public WaterSurface waterSurface;

    [SerializeField] private float shipMass = 10f; // kg
	[SerializeField] private float shipLength = 10f; // m
	[SerializeField] private float shipRoll = 10f; // Deg
	private float gravity = -9.81f; // m / s^2
    private float waterDensity = 1000f; // kg / m^3
	private int longitudinalBuoyantForceSplit = 6;
	private int lateralBuoyantForceSplit = 2;
	private float totalArea = 0f;
	private Vector2 submergedCenterOfBuoyancy = Vector2.zero;

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

		waterSurfaceHeightSearch = new WaterSurfaceHeightSearch(longitudinalBuoyantForceSplit, lateralBuoyantForceSplit, waterSurface);

		totalArea = GetTotalArea();
		submergedCenterOfBuoyancy = GetSubmergedCenterOfBuoyancy();
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

		//float buoyantForce = -waterDensity * gravity * GetSubmergedArea() * shipLength;
		//float total = 0f;

		Vector3[] forcePositions = new Vector3[longitudinalBuoyantForceSplit * lateralBuoyantForceSplit];

		for (int x = 0; x < longitudinalBuoyantForceSplit; x++) {
			for (int y = 0; y < lateralBuoyantForceSplit; y++) {
				//forcePositions[x * lateralBuoyantForceSplit + y] = transform.TransformPoint(new Vector3(((1f / lateralBuoyantForceSplit * y + 1f / (lateralBuoyantForceSplit * 2)) - 0.5f) / 1f, 0, (1f / longitudinalBuoyantForceSplit * x + 1f / (longitudinalBuoyantForceSplit * 2)) - 0.5f));
				forcePositions[x * lateralBuoyantForceSplit + y] = transform.TransformPoint(new Vector3(((hull.hullWidth / lateralBuoyantForceSplit * y + hull.hullWidth / (lateralBuoyantForceSplit * 2f)) - (hull.hullWidth / 2f)), 0, (shipLength / longitudinalBuoyantForceSplit * x + shipLength / (longitudinalBuoyantForceSplit * 2f)) - (shipLength / 2f)));
			}
		}

		float[] waterHeights = waterSurfaceHeightSearch.GetWaterHeights(forcePositions);

		for (int x = 0; x < longitudinalBuoyantForceSplit; x++) {

			List<Vector2> sectionWaterHeightPoints = new List<Vector2>();
			float avgSectionWaterHeight = 0f;

			for (int i = x * lateralBuoyantForceSplit; i < (x + 1) * lateralBuoyantForceSplit; i++) {
				avgSectionWaterHeight += waterHeights[i];
				sectionWaterHeightPoints.Add(new Vector2(forcePositions[i].x, waterHeights[i]));
			}
			avgSectionWaterHeight /= lateralBuoyantForceSplit;

			float sectionWaterLine = avgSectionWaterHeight - transform.TransformPoint(new Vector3(0, 0, (shipLength / longitudinalBuoyantForceSplit * x + shipLength / (longitudinalBuoyantForceSplit * 2f)) - (shipLength / 2f))).y + hull.hullPoints[hull.hullPoints.Count - 1].y;

			float sectionWaterSlope;
			float sectionWaterB;
			FindLinearLeastSquaresFit(sectionWaterHeightPoints, out sectionWaterSlope, out sectionWaterB);
			float sectionWaterAngle = Mathf.Atan(sectionWaterSlope) * Mathf.Rad2Deg;
			//float sectionWaterAngle = 0f;

			//if (sectionWaterAngle > 10 || sectionWaterAngle < -10) Debug.Log(sectionWaterAngle);
			Vector2 centerOfBuoyancy = Vector2.zero;
			float submergedArea = GetSubmergedArea(sectionWaterLine, sectionWaterAngle, out centerOfBuoyancy);
			//if (x == 0) print(centerOfBuoyancy);
			float sectionBuoyantForce = -waterDensity * gravity * submergedArea * shipLength / longitudinalBuoyantForceSplit;
			//Debug.Log(centerOfBuoyancy);
			//Vector3 buoyantForcePos = transform.TransformPoint(new Vector3(centerOfBuoyancy.x / transform.localScale.x, centerOfBuoyancy.y / transform.localScale.y - 0.5f, (1f / longitudinalBuoyantForceSplit * x + 1f / (longitudinalBuoyantForceSplit * 2)) - 0.5f));
			Vector3 buoyantForcePos = transform.TransformPoint(new Vector3(centerOfBuoyancy.x, centerOfBuoyancy.y, (shipLength / longitudinalBuoyantForceSplit * x + shipLength / (longitudinalBuoyantForceSplit * 2f)) - (shipLength / 2f)));
			//Debug.DrawRay(buoyantForcePos, Vector3.up * 2, UnityEngine.Color.blue, Time.deltaTime * 10);

			//Debug.Log(new Vector3(centerOfBuoyancy.x / transform.localScale.x, centerOfBuoyancy.y / transform.localScale.y - 0.5f, (1f / longitudinalBuoyantForceSplit * x + 1f / (longitudinalBuoyantForceSplit * 2)) - 0.5f));
			//Debug.Log(buoyantForcePos);
			/*for (int y = 0; y < lateralBuoyantForceSplit; y++) {
				//Vector3 pos = new Vector3(((hull.hullPoints[hull.hullPoints.Count-1].x * 2f) / (lateralBuoyantForceSplit - 1f) * x - (hull.hullPoints[hull.hullPoints.Count - 1].x * 2f) / 2f) / 4f, 0, shipLength / (longitudinalBuoyantForceSplit - 1f) * x - shipLength / 2f);
				//Vector3 pos = transform.TransformPoint(new Vector3(((1f / lateralBuoyantForceSplit * y + 1f / (lateralBuoyantForceSplit * 2)) - 0.5f) / 2f, 0, (1f / longitudinalBuoyantForceSplit * x + 1f / (longitudinalBuoyantForceSplit * 2)) - 0.5f));
				//total += buoyantForce / (longitudinalBuoyantForceSplit * lateralBuoyantForceSplit);

				rb.AddForceAtPosition(Vector3.up * sectionBuoyantForce / lateralBuoyantForceSplit, forcePositions[x * lateralBuoyantForceSplit + y], ForceMode.Force);
				Debug.DrawRay(forcePositions[x * lateralBuoyantForceSplit + y], Vector3.up*2, UnityEngine.Color.red, Time.deltaTime*10);
			}*/
			rb.AddForceAtPosition(Vector3.up * sectionBuoyantForce, buoyantForcePos, ForceMode.Force);
			Debug.DrawRay(buoyantForcePos, Vector3.up * 2, UnityEngine.Color.red, Time.deltaTime * 10);
		}
		//print(total);
		//print(buoyantForce);
		//rb.AddForce(Vector3.up * buoyantForce, ForceMode.Force);
	}

    private float CalculateFlatWaterline() {
        float halfDisplacedVolumePerLength = (shipMass / waterDensity) / shipLength / 2;
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

	private float CalculateRotatedWaterline(float waterlineAngle) {
		float displacedVolumePerLength = (shipMass / waterDensity) / shipLength;
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
			
			if ((newLeftHullPoints[i].x >= 0f && newLeftHullPoints[i + 1].x < 0f)/* || (newLeftHullPoints[i].x < 0f && newLeftHullPoints[i + 1].x >= 0f)*/) {
				sectionAreaLeft /= 2;
			} else if (newLeftHullPoints[i].x < 0f && newLeftHullPoints[i + 1].x < 0f) {
				sectionAreaLeft = -sectionAreaLeft;
			}

			if ((newRightHullPoints[i].x >= 0f && newRightHullPoints[i + 1].x < 0f)/* || (newRightHullPoints[i].x < 0f && newRightHullPoints[i + 1].x >= 0f)*/) {
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

				/*if (Mathf.Abs(dy1) < Mathf.Abs(dy2) && dy1 > 0) {
					//print("Returning");
					print(GetUnrotatedPoint(new Vector2(-(x - pivotOffset.x), dy1 - pivotOffset.y)));
					return GetUnrotatedPoint(new Vector2(-(x - pivotOffset.x), dy1 - pivotOffset.y)).y;
					//return dy1;
				} else if (Mathf.Abs(dy2) < Mathf.Abs(dy1) && dy2 > 0) {
					//print("Returning");
					//return dy2;
				} else if (Mathf.Abs(dy1) == Mathf.Abs(dy2)) {
					return newRightHullPoints[i].y + Mathf.Abs(dy1);
				}*/

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

					/*if (Mathf.Abs(dy1) < Mathf.Abs(dy2) && dy1 > 0) {
						//print("Returning");
						print(GetUnrotatedPoint(new Vector2(x, dy1)));
						return GetUnrotatedPoint(new Vector2(x, dy1)).y;
						//return newRightHullPoints[i].y + dy1;
					} else if (Mathf.Abs(dy2) < Mathf.Abs(dy1) && dy2 > 0) {
						//print("Returning");
						//return newRightHullPoints[i].y + dy2;
					} else if (Mathf.Abs(dy1) == Mathf.Abs(dy2)) {
						return newRightHullPoints[i].y + Mathf.Abs(dy1);
					}*/

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

					/*if (Mathf.Abs(dy1) < Mathf.Abs(dy2) && dy1 > 0) {
						//print("Returning");
						return GetUnrotatedPoint(new Vector2(x, newLeftHullPoints[i].y + dy1)).y;
						//return newLeftHullPoints[i].y + dy1;
					} else if (Mathf.Abs(dy2) < Mathf.Abs(dy1) && dy2 > 0) {
						//print("Returning");
						//return newLeftHullPoints[i].y + dy2;
					} else if (Mathf.Abs(dy1) == Mathf.Abs(dy2)) {
						return newLeftHullPoints[i].y + Mathf.Abs(dy1);
					}*/

				} else {
					area += sectionArea;
				}
			}
		}

		return -1f;
	}

	private float GetSubmergedArea(float waterline, float waterlineAngle, out Vector2 centerOfBuoyancy) {
		centerOfBuoyancy = Vector2.zero;

		if (waterline <= 0) { // Out of water
			return 0f;
		}

		float area = 0f;

		List<Vector2> rightHullPoints = GetRotatedHullPoints(hull.hullPoints, waterlineAngle);
		List<Vector2> leftHullPoints = GetRotatedHullPoints(GetLeftHullPoints(), waterlineAngle);

		// Find lowest y-position and its index
		float minY = hull.hullPoints[0].y;
		float maxY = hull.hullPoints[hull.hullPoints.Count - 1].y;
		int index = 0;
		for (int i = 0; i < rightHullPoints.Count; i++) {
			if (rightHullPoints[i].y < minY) {
				minY = rightHullPoints[i].y;
				index = i;
			}
			if (rightHullPoints[i].y > maxY) {
				maxY = rightHullPoints[i].y;
			}
		}
		for (int i = 0; i < leftHullPoints.Count; i++) {
			if (leftHullPoints[i].y < minY) {
				minY = leftHullPoints[i].y;
				index = i;
			}
			if (leftHullPoints[i].y > maxY) {
				maxY = leftHullPoints[i].y;
			}
		}

		// Return appropriate values if fully submerged
		if (waterline >= maxY - minY) {
			centerOfBuoyancy = submergedCenterOfBuoyancy;
			return totalArea;
		}

		// Redefine hull positions by setting a new origin at lowest point and flipping the left sides x-positions

		List<Vector2> newRightHullPoints = new List<Vector2>();
		List<Vector2> newLeftHullPoints = new List<Vector2>();

		Vector2 pivotOffset = Vector2.zero;

		// CCW Ship Roll is positive
		if (index != 0 && shipRoll + waterlineAngle >= 0) { // CCW Roll
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

		} else if (index != 0 && shipRoll + waterlineAngle < 0) { // CW Roll
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
			pivotOffset = new Vector2(0f, hull.hullPoints[0].y);
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

		if (shipRoll + waterlineAngle > 0) {
			for (int i = 0; i < newRightHullPoints.Count - 1; i++) {
				if (newRightHullPoints[i + 1].x < 0f && newRightHullPoints[i].x > 0f) { // Hull line crosses y-axis
					float yIntercept = newRightHullPoints[i].y - newRightHullPoints[i].x * (newRightHullPoints[i + 1].y - newRightHullPoints[i].y) / (newRightHullPoints[i + 1].x - newRightHullPoints[i].x);
					deckPoint = new Vector2(-((yIntercept - newLeftHullPoints[newLeftHullPoints.Count - 1].y) * (newRightHullPoints[newRightHullPoints.Count - 1].x + newLeftHullPoints[newLeftHullPoints.Count - 1].x) / (newRightHullPoints[newRightHullPoints.Count - 1].y - newLeftHullPoints[newLeftHullPoints.Count - 1].y) - newLeftHullPoints[newLeftHullPoints.Count - 1].x), yIntercept);
					crossesY = true;
					break;
				}
			}
			
			newLeftHullPoints.Add(deckPoint);
		} else if (shipRoll + waterlineAngle < 0) {
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

		if (shipRoll + waterlineAngle > 0) {
			for (int i = newLeftHullPoints.Count - 1; i < newRightHullPoints.Count; i++) {
				originalNewHullPoints.Add(newRightHullPoints[i]);
				//aboveHalfDeckPoints.Add(((newRightHullPoints[i].y - newLeftHullPoints[newLeftHullPoints.Count - 1].y) * (newRightHullPoints[newRightHullPoints.Count - 1].x + newLeftHullPoints[newLeftHullPoints.Count - 1].x) / (newRightHullPoints[newRightHullPoints.Count - 1].y - newLeftHullPoints[newLeftHullPoints.Count - 1].y) - newLeftHullPoints[newLeftHullPoints.Count - 1].x));
				newRightHullPoints[i] = new Vector2(newRightHullPoints[i].x - ((newRightHullPoints[i].y - newLeftHullPoints[newLeftHullPoints.Count - 1].y) * (newRightHullPoints[newRightHullPoints.Count - 1].x + newLeftHullPoints[newLeftHullPoints.Count - 1].x) / (newRightHullPoints[newRightHullPoints.Count - 1].y - newLeftHullPoints[newLeftHullPoints.Count - 1].y) - newLeftHullPoints[newLeftHullPoints.Count - 1].x), newRightHullPoints[i].y);
				aboveHalfDeckPoints.Add(-(newRightHullPoints[i].x - originalNewHullPoints[i - (newLeftHullPoints.Count - 1)].x));
			}
		} else if (shipRoll + waterlineAngle < 0) {
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


				if (crossesY && i == pointsToTop - 2 && shipRoll + waterlineAngle > 0) {
					xRight = newRightHullPoints[i].x + (waterline - newRightHullPoints[i].y) * (-newRightHullPoints[i].x) / (newRightHullPoints[i + 1].y - newRightHullPoints[i].y);
					xLeft = newLeftHullPoints[i].x + (waterline - newLeftHullPoints[i].y) * (newLeftHullPoints[i + 1].x - newLeftHullPoints[i].x) / (newLeftHullPoints[i + 1].y - newLeftHullPoints[i].y);

				} else if (crossesY && i == pointsToTop - 2 && shipRoll + waterlineAngle < 0) {
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
				centerOfBuoyancy = GetUnrotatedPoint(centerOfBuoyancy / area + pivotOffset, waterlineAngle);
				return area;

			} else if (newRightHullPoints[i].y == waterline) {
				//print(newRightHullPoints[i].y);
				centerOfBuoyancy = GetUnrotatedPoint(centerOfBuoyancy / area + pivotOffset, waterlineAngle);
				return area;
			} else {

				float sectionAreaLeft = 0f;
				float sectionAreaRight = 0f;

				Vector2 sectionAreaCenterRight = Vector2.zero;
				Vector2 sectionAreaCenterLeft = Vector2.zero;

				if (crossesY && i == pointsToTop - 2 && shipRoll + waterlineAngle > 0) {
					sectionAreaRight = newRightHullPoints[i].x / 2 * (newRightHullPoints[i + 1].y - newRightHullPoints[i].y);
					sectionAreaCenterRight = new Vector2((newRightHullPoints[i].x * newRightHullPoints[i].x) / (3 * newRightHullPoints[i].x), newRightHullPoints[i].y + (newRightHullPoints[i].x) * (newRightHullPoints[i + 1].y - newRightHullPoints[i].y) / (3 * (newRightHullPoints[i].x)));

					sectionAreaLeft = (newLeftHullPoints[i].x + newLeftHullPoints[i + 1].x) / 2 * (newLeftHullPoints[i + 1].y - newLeftHullPoints[i].y);
					sectionAreaCenterLeft = new Vector2(-(newLeftHullPoints[i].x * newLeftHullPoints[i].x + newLeftHullPoints[i].x * newLeftHullPoints[i + 1].x + newLeftHullPoints[i + 1].x * newLeftHullPoints[i + 1].x) / (3 * (newLeftHullPoints[i].x + newLeftHullPoints[i + 1].x)), newLeftHullPoints[i].y + (newLeftHullPoints[i].x + 2 * newLeftHullPoints[i + 1].x) * (newLeftHullPoints[i + 1].y - newLeftHullPoints[i].y) / (3 * (newLeftHullPoints[i].x + newLeftHullPoints[i + 1].x)));
					
				} else if (crossesY && i == pointsToTop - 2 && shipRoll + waterlineAngle < 0) {
					sectionAreaLeft = newLeftHullPoints[i].x / 2 * (newLeftHullPoints[i + 1].y - newLeftHullPoints[i].y);
					sectionAreaCenterLeft = new Vector2(-(newLeftHullPoints[i].x * newLeftHullPoints[i].x) / (3 * (newLeftHullPoints[i].x)), newLeftHullPoints[i].y + (newLeftHullPoints[i].x) * (newLeftHullPoints[i + 1].y - newLeftHullPoints[i].y) / (3 * (newLeftHullPoints[i].x)));

					sectionAreaRight = (newRightHullPoints[i].x + newRightHullPoints[i + 1].x) / 2 * (newRightHullPoints[i + 1].y - newRightHullPoints[i].y);
					sectionAreaCenterRight = new Vector2((newRightHullPoints[i].x * newRightHullPoints[i].x + newRightHullPoints[i].x * newRightHullPoints[i + 1].x + newRightHullPoints[i + 1].x * newRightHullPoints[i + 1].x) / (3 * (newRightHullPoints[i].x + newRightHullPoints[i + 1].x)), newRightHullPoints[i].y + (newRightHullPoints[i].x + 2 * newRightHullPoints[i + 1].x) * (newRightHullPoints[i + 1].y - newRightHullPoints[i].y) / (3 * (newRightHullPoints[i].x + newRightHullPoints[i + 1].x)));
					
				} else {
					sectionAreaLeft = (newLeftHullPoints[i].x + newLeftHullPoints[i + 1].x) / 2 * (newLeftHullPoints[i + 1].y - newLeftHullPoints[i].y);
					sectionAreaRight = (newRightHullPoints[i].x + newRightHullPoints[i + 1].x) / 2 * (newRightHullPoints[i + 1].y - newRightHullPoints[i].y);

					sectionAreaCenterRight = new Vector2((newRightHullPoints[i].x * newRightHullPoints[i].x + newRightHullPoints[i].x * newRightHullPoints[i + 1].x + newRightHullPoints[i + 1].x * newRightHullPoints[i + 1].x) / (3 * (newRightHullPoints[i].x + newRightHullPoints[i + 1].x)), newRightHullPoints[i].y + (newRightHullPoints[i].x + 2 * newRightHullPoints[i + 1].x) * (newRightHullPoints[i + 1].y - newRightHullPoints[i].y) / (3 * (newRightHullPoints[i].x + newRightHullPoints[i + 1].x)));
					sectionAreaCenterLeft = new Vector2(-(newLeftHullPoints[i].x * newLeftHullPoints[i].x + newLeftHullPoints[i].x * newLeftHullPoints[i + 1].x + newLeftHullPoints[i + 1].x * newLeftHullPoints[i + 1].x) / (3 * (newLeftHullPoints[i].x + newLeftHullPoints[i + 1].x)), newLeftHullPoints[i].y + (newLeftHullPoints[i].x + 2 * newLeftHullPoints[i + 1].x) * (newLeftHullPoints[i + 1].y - newLeftHullPoints[i].y) / (3 * (newLeftHullPoints[i].x + newLeftHullPoints[i + 1].x)));
					
				}

				area += sectionAreaLeft + sectionAreaRight;
				
				centerOfBuoyancy += sectionAreaCenterRight * sectionAreaRight + sectionAreaCenterLeft * sectionAreaLeft;

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

		if (shipRoll + waterlineAngle > 0) { // CCW
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
					centerOfBuoyancy += new Vector2(xPos, yPos) * remainingArea;
					//Debug.Log(centerOfBuoyancy);

					area += remainingArea;

					//centerOfBuoyancy += sectionAreaCenter * remainingArea;

					centerOfBuoyancy = GetUnrotatedPoint(centerOfBuoyancy / area + pivotOffset, waterlineAngle);
					return area;

				} else if (newRightHullPoints[i].y == waterline) {

					centerOfBuoyancy = GetUnrotatedPoint(centerOfBuoyancy / area + pivotOffset, waterlineAngle);
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
		} else if (shipRoll + waterlineAngle < 0) { // CW
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

					centerOfBuoyancy = GetUnrotatedPoint(centerOfBuoyancy / area + pivotOffset, waterlineAngle);
					return area;

				} else if (newLeftHullPoints[i].y == waterline) {

					centerOfBuoyancy = GetUnrotatedPoint(centerOfBuoyancy / area + pivotOffset, waterlineAngle);
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

	public float GetTotalArea() {

		float totalArea = 0f;

		for (int i = 0; i < hull.hullPoints.Count - 1; i++) {
			float sectionArea = (hull.hullPoints[i].x + hull.hullPoints[i + 1].x) * (hull.hullPoints[i + 1].y - hull.hullPoints[i].y);
			totalArea += sectionArea;
		}

		return totalArea;
	}

	public Vector2 GetSubmergedCenterOfBuoyancy() {
		Vector2 center = Vector2.zero;
		float area = 0f;

		for (int i = 0; i < hull.hullPoints.Count - 1; i++) {
			float sectionArea = (hull.hullPoints[i].x + hull.hullPoints[i + 1].x) / 2 * (hull.hullPoints[i + 1].y - hull.hullPoints[i].y);
			Vector2 sectionCenter = new Vector2(0, hull.hullPoints[i].y + (hull.hullPoints[i].x + 2 * hull.hullPoints[i + 1].x) * (hull.hullPoints[i + 1].y - hull.hullPoints[i].y) / (3 * (hull.hullPoints[i].x + hull.hullPoints[i + 1].x)));
			
			area += sectionArea;
			center += sectionCenter * sectionArea;
		}

		return center / area;
	}

	public List<Vector2> GetLeftHullPoints() {
		List<Vector2> flippedPoints = new List<Vector2>();

		// 2D coordinate transformation about longitudinal axis
		foreach (Vector2 point in hull.hullPoints) {
			flippedPoints.Add(new Vector2(-point.x, point.y));
		}

		return flippedPoints;
	}

	private List<Vector2> GetRotatedHullPoints(List<Vector2> fixedHullPoints, float waterlineAngle) {
        List<Vector2> rotatedPoints = new List<Vector2>();

        // 2D coordinate transformation about longitudinal axis
        foreach (Vector2 point in fixedHullPoints) {
            rotatedPoints.Add(GetRotatedPoint(point, waterlineAngle));
        }

        return rotatedPoints;
    }

	private Vector2 GetRotatedPoint(Vector2 point, float waterlineAngle) {
		return new Vector2(Mathf.Cos((shipRoll + waterlineAngle) * Mathf.Deg2Rad) * point.x - Mathf.Sin((shipRoll + waterlineAngle) * Mathf.Deg2Rad) * point.y, Mathf.Sin((shipRoll + waterlineAngle) * Mathf.Deg2Rad) * point.x + Mathf.Cos((shipRoll + waterlineAngle) * Mathf.Deg2Rad) * point.y);
	}

	private Vector2 GetUnrotatedPoint(Vector2 point, float waterlineAngle) {
		return new Vector2(Mathf.Cos((shipRoll + waterlineAngle) * Mathf.Deg2Rad) * point.x + Mathf.Sin((shipRoll + waterlineAngle) * Mathf.Deg2Rad) * point.y, -Mathf.Sin((shipRoll + waterlineAngle) * Mathf.Deg2Rad) * point.x + Mathf.Cos((shipRoll + waterlineAngle) * Mathf.Deg2Rad) * point.y);
	}

	public static float ErrorSquared(List<Vector2> points, float m, float b) {
		float total = 0;
		foreach (Vector2 pt in points) {
			float dy = pt.y - (m * pt.x + b);
			total += dy * dy;
		}
		return total;
	}

	public static float FindLinearLeastSquaresFit(List<Vector2> points, out float m, out float b) {
		// Perform the calculation.
		// Find the values S1, Sx, Sy, Sxx, and Sxy.
		float S1 = points.Count;
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
        return shipMass;
    }

    public void SetMass(float newMass) {
        shipMass = newMass;

        if (shipMass < 0f) {
            shipMass = 0f;
        }
    }

	public float GetLength() {
		return shipLength;
	}

	public void SetLength(float newLength) {
		shipLength = newLength;

		if (shipLength < 0f) {
			shipLength = 0f;
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
}