using System.Collections;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Xml.Linq;
using Unity.Mathematics;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;
using static UnityEngine.Rendering.ProbeTouchupVolume;
using Color = UnityEngine.Color;

[RequireComponent(typeof(Rigidbody))]
public class BuoyantHull : MonoBehaviour
{
	public HullShape hull;
	public WaterSurface waterSurface;
	[SerializeField] private float hullLength = 1f; // m
	[SerializeField] private float hullWidth = 1f; // m
	[SerializeField] private float hullHeight = 1f; // m
	//[SerializeField] private float dragCoeff = 0.4f; // Deg
	[SerializeField] private bool debug = false;
	[SerializeField] private int debugForceScaling = 500000; // Inverse, larger is smaller
	[SerializeField] private DirectionalDrag dragCoeffs;
	[SerializeField] private float rudderDragNormalCoeff = 1.28f;
	[SerializeField] private float rudderDragShearCoeff = 0.2f;
	public List<Transform> rotatingRigging;

	private float shipRoll = 0f; // Deg
	private float shipPitch = 0f; // Deg
	private float rudderAngle = 0f; // Deg
	private float riggingAngle = 0f; // Deg
	private float gravity = -9.81f; // m / s^2
	private float waterDensity = 1000f; // kg / m^3
	private float airDensity = 1.293f; // kg / m^3
	private int lateralBuoyantForceSplit = 2;

	private Rigidbody rb;
	private WaterSurfaceHeightSearch waterSurfaceHeightSearch;

	private bool[] isSectionSubmerged;
	private bool[] isSectionAboveWater;
	private List<Vector2> totalSideOutline = new List<Vector2>();
	private List<Vector2> underwaterSideOutline = new List<Vector2>();
	//private List<Vector2> abovewaterSideOutline = new List<Vector2>();
	private float underwaterSideArea = 0f;
	private float abovewaterSideArea = 0f;
	private Vector2 underwaterSideCentroid = Vector2.zero;
	private Vector2 abovewaterSideCentroid = Vector2.zero;
	private float totalSideArea = 0f;
	private Vector2 totalSideCentroid = Vector2.zero;
	public float[] sectionTotalArea { get; set; }
	public Vector2[] sectionSubmergedCenterOfBuoyancy { get; set; }
	private float abovewaterLongitudinalArea = 0f;
	private Vector2 abovewaterLongitudinalCentroid = Vector2.zero;
	private float underwaterLongitudinalArea = 0f;
	private Vector2 underwaterLongitudinalCentroid = Vector2.zero;
	private float bottomWaterArea = 0f;
	private Vector2 bottomWaterCentroid = Vector2.zero;
	private List<Vector2> bottomOutline = new List<Vector2>(); // Represents the water outline on the bottom of the ship
	private float bottomAirArea = 0f;
	private Vector2 bottomAirCentroid = Vector2.zero;
	private float topWaterArea = 0f;
	private Vector2 topWaterCentroid = Vector2.zero;
	private float topAirArea = 0f;
	private Vector2 topAirCentroid = Vector2.zero;
	private List<Vector2> topOutline = new List<Vector2>(); // Represents the air outline on the top of the ship
	private Vector2[,] hullSectionWaterPoints;
	private List<Vector2> verticalHullOutline = new List<Vector2>();
	private Vector2 verticalHullCentroid = Vector2.zero;
	private float verticalHullArea = 0f;
	private Vector3 localVelocity;
	private Vector3[] rudderOutline;
	private float rudderDragFactor = 0;
	private Vector3 rudderNormal = Vector3.right;
	private Vector3 rudderCentroid = Vector3.zero;

	private bool isFullySubmerged = false;
	private bool isOutOfWater = false;
	private int numSecInWater = 0;
	private bool isInitialized = false;

	// Start is called before the first frame update
	void Start()
	{
		rb = GetComponent<Rigidbody>();
		localVelocity = transform.InverseTransformVector(rb.velocity);

		isSectionSubmerged = new bool[hull.hullSections.Count];
		isSectionAboveWater = new bool[hull.hullSections.Count];

		// Looks in scene for existing water surface if it's not set
		if (waterSurface == null) {
			waterSurface = FindObjectsByType<WaterSurface>(FindObjectsSortMode.None)[0];
		}

		waterSurfaceHeightSearch = new WaterSurfaceHeightSearch(hull.hullSections.Count, lateralBuoyantForceSplit, waterSurface);

		sectionTotalArea = new float[hull.hullSections.Count];
		sectionSubmergedCenterOfBuoyancy = new Vector2[hull.hullSections.Count];

		for (int i = 0; i < hull.hullSections.Count; i++)
		{
			sectionTotalArea[i] = GetTotalArea(hull.hullSections[i]);
			sectionSubmergedCenterOfBuoyancy[i] = GetSubmergedCenterOfBuoyancy(hull.hullSections[i]);
		}

		totalSideArea = GetTotalSideAreaAndCentroid(out totalSideCentroid);
		hullSectionWaterPoints = new Vector2[hull.hullSections.Count, 2];

		rudderDragFactor = GetPolygonArea(hull.rudder.GetUnnormalizedOutline(hullHeight, hullLength)) * 0.5f * waterDensity;
		rudderOutline = hull.rudder.GetUnnormalizedRotatedPoints(hullWidth, hullHeight, hullLength, rudderAngle);
		rudderCentroid = hull.rudder.GetCentroid(hullWidth, hullHeight, hullLength, rudderAngle);
		rudderNormal = Vector3.Cross(rudderOutline[1] - rudderOutline[0], rudderOutline[2] - rudderOutline[0]).normalized;

		#region Construct Vertical Hull Outline
		for (int i = 0; i < hull.hullSections.Count; i++) {
			// Find widest points on the section
			float minX = -0.5f;
			float maxX = 0.5f;

			for(int j = 0; j < hull.hullSections[i].hullShape.hullPoints.Count; j++) {
				if (minX > hull.hullSections[i].hullShape.hullPoints[j].x) {
					minX = hull.hullSections[i].hullShape.hullPoints[j].x;
				}

				if (maxX < hull.hullSections[i].hullShape.hullPoints[j].x) {
					maxX = hull.hullSections[i].hullShape.hullPoints[j].x;
				}
			}

			minX *= hullWidth * hull.hullSections[i].normalizedSectionWidth;
			maxX *= hullWidth * hull.hullSections[i].normalizedSectionWidth;

			verticalHullOutline.Insert(i, new Vector2(minX, hull.hullSections[i].zPos * hullLength));
			verticalHullOutline.Insert(verticalHullOutline.Count - i, new Vector2(maxX, hull.hullSections[i].zPos * hullLength));
		}

		verticalHullArea = GetPolygonArea(verticalHullOutline.ToArray());
		verticalHullCentroid = GetPolygonCentroid(verticalHullOutline.ToArray(), verticalHullArea);
		#endregion

	}

	// Update is called once per frame
	void Update()
	{
		if (!isInitialized) {
			if (PositionOnWater()) {
				isInitialized = true;
			}
		}

		#region Wrap Rotations
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
		#endregion

	}

	private void FixedUpdate() {
		if (!isInitialized) return;

		#region Reset variables for recalculation
		float[] sectionWaterLines = new float[hull.hullSections.Count];
		float[] sectionWaterRollAngles = new float[hull.hullSections.Count];
		float[] sectionWaterPitchAngles = new float[hull.hullSections.Count];
		Vector3 currentDirection = Vector3.zero;
		float currentSpeed = 0.5f;
		float overallMinRotatedY = -hullHeight / 2f;
		float largestSubmergedHullSectionArea = 0f;
		Vector2 largestSubmergedHullSectionCentroid = Vector2.zero;
		int largestSubmergedHullSectionIndex = 0;
		bottomOutline.Clear();
		topOutline.Clear();
		#endregion

		// Update local velocity
		localVelocity = transform.InverseTransformVector(rb.velocity);

		// Get Latest Water Parameters
		if (!UpdateWaterValues(out sectionWaterLines, out sectionWaterRollAngles, out sectionWaterPitchAngles, out overallMinRotatedY, out currentDirection)) {
			return; // Return if error occurs in retrieving water data
		}

		Vector3 avgForcePos = Vector3.zero;
		// Calculates forces and submersion data for each hull section
		ApplyBuoyancy(sectionWaterLines, sectionWaterRollAngles, sectionWaterPitchAngles, overallMinRotatedY, out largestSubmergedHullSectionArea, out largestSubmergedHullSectionCentroid, out largestSubmergedHullSectionIndex, out avgForcePos);
		
		// Calculates transverse ship outlines (Opposite sides are assumed equivalent)
		UpdateSideProperties(sectionWaterLines, largestSubmergedHullSectionArea, largestSubmergedHullSectionCentroid, largestSubmergedHullSectionIndex);

		// Applies calculated linear and torsional drag
		Vector3 relativeCurrentDirection = transform.InverseTransformDirection(currentDirection);
		ApplyDrag(sectionWaterLines, relativeCurrentDirection, currentSpeed);

		#region DRAW DEBUG OUTLINES

		if (debug) {

			Vector3 flattenedVelocity = Vector3.ProjectOnPlane(rb.velocity, Vector3.up);
			Debug.DrawRay(transform.TransformPoint(rb.centerOfMass), flattenedVelocity * 40, Color.yellow);

			Vector2[][] hullPoints = new Vector2[hull.hullSections.Count][];
			for (int i = 0; i < hull.hullSections.Count; i++) {
				hullPoints[i] = hull.GetUnnormalizedHullPoints(i, hullHeight, hullWidth);
				Vector2[] leftHullPoints = hull.GetMirroredHullPoints(hullPoints[i]);
				hullPoints[i] = hullPoints[i].Concat(leftHullPoints.Reverse()).ToArray();
				DrawOutline(OutlineTo3D(hullPoints[i], Vector3.forward, hull.hullSections[i].zPos * hullLength), Color.white);
			}

			Vector3[][] horizontalOutlines = new Vector3[hull.hullSections[0].hullShape.hullPoints.Count][];
			for (int i = 0; i < hull.hullSections[0].hullShape.hullPoints.Count; i++) {
				horizontalOutlines[i] = new Vector3[hull.hullSections.Count * 2];

				for (int j = 0; j < hull.hullSections.Count; j++) {
					horizontalOutlines[i][j] = transform.TransformPoint(hullPoints[j][i].x, hullPoints[j][i].y, hull.hullSections[j].zPos * hullLength);
					horizontalOutlines[i][hull.hullSections.Count * 2 - 1 - j] = transform.TransformPoint(-hullPoints[j][i].x, hullPoints[j][i].y, hull.hullSections[j].zPos * hullLength);
				}

				DrawOutline(horizontalOutlines[i], Color.white);
			}

			Vector3[] transformedRudderPoints = hull.rudder.GetUnnormalizedRotatedPoints(hullWidth, hullHeight, hullLength, rudderAngle);
			transform.TransformPoints(transformedRudderPoints);
			DrawOutline(transformedRudderPoints, Color.white);
		}

		#endregion

		#region Water Displacement Reaction Force
		if (numSecInWater > 0 && localVelocity.y < 0) {
			float volumeToBeDisplaced = hullWidth * hullLength / hull.hullSections.Count * numSecInWater * -localVelocity.y;

			float buoyantReactionForce = -waterDensity * gravity * volumeToBeDisplaced * 0.2f;

			rb.AddForceAtPosition(Vector3.up * buoyantReactionForce, avgForcePos, ForceMode.Force);
			//if (debug) Debug.DrawRay(avgForcePos, Vector3.up * buoyantReactionForce / debugForceScaling, Color.magenta, Time.deltaTime * 5);
		}
		#endregion
	}

	private bool PositionOnWater() {
		Vector3[] waterSamplePositions = new Vector3[hull.hullSections.Count * lateralBuoyantForceSplit];
		for (int x = 0; x < hull.hullSections.Count; x++) {
			for (int y = 0; y < lateralBuoyantForceSplit; y++) {
				waterSamplePositions[x * lateralBuoyantForceSplit + y] = transform.TransformPoint(new Vector3((-0.4f + y * 0.8f) * hullWidth * hull.hullSections[x].normalizedSectionWidth, 0, hull.hullSections[x].zPos * hullLength));

			}
		}

		Vector3 currentDirection = Vector3.zero;
		float[] waterHeights = waterSurfaceHeightSearch.GetWaterHeights(waterSamplePositions, out currentDirection);

		float avgWaterHeight = 0f;
		for (int i = 0; i < hull.hullSections.Count * lateralBuoyantForceSplit; i++) {
			try {
				avgWaterHeight += waterHeights[i];
			} catch {
				return false; // Height values failed to be found
			}
		}
		avgWaterHeight /= waterHeights.Length;
		rb.position = new Vector3(rb.position.x, avgWaterHeight, rb.position.z);
		return true;
	}

	private bool UpdateWaterValues(out float[] sectionWaterLines, out float[] sectionWaterRollAngles, out float[] sectionWaterPitchAngles, out float overallMinRotatedY, out Vector3 currentDirection) {

		sectionWaterLines = new float[hull.hullSections.Count];
		sectionWaterRollAngles = new float[hull.hullSections.Count];
		sectionWaterPitchAngles = new float[hull.hullSections.Count];
		currentDirection = Vector3.zero;
		overallMinRotatedY = -hullHeight / 2f;

		Vector3[] waterSamplePositions = new Vector3[hull.hullSections.Count * lateralBuoyantForceSplit];
		for (int x = 0; x < hull.hullSections.Count; x++) {
			for (int y = 0; y < lateralBuoyantForceSplit; y++) {
				waterSamplePositions[x * lateralBuoyantForceSplit + y] = transform.TransformPoint(new Vector3((-0.4f + y * 0.8f) * hullWidth * hull.hullSections[x].normalizedSectionWidth, 0, hull.hullSections[x].zPos * hullLength));

			}
		}

		float[] waterHeights = waterSurfaceHeightSearch.GetWaterHeights(waterSamplePositions, out currentDirection);

		for (int x = 0; x < hull.hullSections.Count; x++) {

			Vector2[] sectionWaterHeightPoints = new Vector2[lateralBuoyantForceSplit];
			float avgSectionWaterHeight = 0f;

			for (int i = x * lateralBuoyantForceSplit; i < (x + 1) * lateralBuoyantForceSplit; i++) {
				try {
					avgSectionWaterHeight += waterHeights[i];
				} catch {
					return false; // Height values failed to be found
				}
				sectionWaterHeightPoints[i - x * lateralBuoyantForceSplit] = new Vector2((-0.4f + (i - x * lateralBuoyantForceSplit) * 0.8f) * hullWidth * hull.hullSections[x].normalizedSectionWidth, waterHeights[i]);
			}
			avgSectionWaterHeight /= lateralBuoyantForceSplit;

			sectionWaterLines[x] = avgSectionWaterHeight - transform.TransformPoint(new Vector3(0, 0, hull.hullSections[x].zPos * hullLength)).y + (hullHeight / 2f);

			float sectionWaterRollSlope;
			float sectionWaterB;
			FindLinearLeastSquaresFit(sectionWaterHeightPoints, out sectionWaterRollSlope, out sectionWaterB);
			sectionWaterRollAngles[x] = Mathf.Atan(sectionWaterRollSlope) * Mathf.Rad2Deg;

			if (x == 0) { // Calculate water pitch angle based on back-end and adjacent water height values

				Vector2[] lengthwiseHeightPoints = new Vector2[2];
				lengthwiseHeightPoints[0] = new Vector2(hull.hullSections[x].zPos * hullLength, avgSectionWaterHeight);
				lengthwiseHeightPoints[1] = new Vector2(hull.hullSections[x + 1].zPos * hullLength, (waterHeights[(x + 1) * lateralBuoyantForceSplit] + waterHeights[(x + 1) * lateralBuoyantForceSplit + 1]) / 2f);


				float sectionWaterPitchSlope;
				FindLinearLeastSquaresFit(lengthwiseHeightPoints, out sectionWaterPitchSlope, out sectionWaterB);

				sectionWaterPitchAngles[x] = Mathf.Atan(sectionWaterPitchSlope) * Mathf.Rad2Deg;

			} else if (x == hull.hullSections.Count - 1) { // Calculate water pitch angle based on front-end and adjacent water height values

				Vector2[] lengthwiseHeightPoints = new Vector2[2];
				lengthwiseHeightPoints[0] = new Vector2(hull.hullSections[x - 1].zPos * hullLength, (waterHeights[(x - 1) * lateralBuoyantForceSplit] + waterHeights[(x - 1) * lateralBuoyantForceSplit + 1]) / 2f);
				lengthwiseHeightPoints[1] = new Vector2(hull.hullSections[x].zPos * hullLength, avgSectionWaterHeight);

				float sectionWaterPitchSlope;
				FindLinearLeastSquaresFit(lengthwiseHeightPoints, out sectionWaterPitchSlope, out sectionWaterB);

				sectionWaterPitchAngles[x] = Mathf.Atan(sectionWaterPitchSlope) * Mathf.Rad2Deg;

			} else { // Caluclate water pitch angle based on current section water height and 2 adjacent sides water heights

				Vector2[] lengthwiseHeightPoints = new Vector2[3];
				lengthwiseHeightPoints[0] = new Vector2(hull.hullSections[x - 1].zPos * hullLength, (waterHeights[(x - 1) * lateralBuoyantForceSplit] + waterHeights[(x - 1) * lateralBuoyantForceSplit + 1]) / 2f);
				lengthwiseHeightPoints[1] = new Vector2(hull.hullSections[x].zPos * hullLength, avgSectionWaterHeight);
				lengthwiseHeightPoints[2] = new Vector2(hull.hullSections[x + 1].zPos * hullLength, (waterHeights[(x + 1) * lateralBuoyantForceSplit] + waterHeights[(x + 1) * lateralBuoyantForceSplit + 1]) / 2f);

				float sectionWaterPitchSlope;
				FindLinearLeastSquaresFit(lengthwiseHeightPoints, out sectionWaterPitchSlope, out sectionWaterB);

				sectionWaterPitchAngles[x] = Mathf.Atan(sectionWaterPitchSlope) * Mathf.Rad2Deg;

			}

			float sectionMinY = GetSectionMinRotatedY(x, shipRoll + sectionWaterRollAngles[x]);
			if (sectionMinY < overallMinRotatedY) {
				overallMinRotatedY = sectionMinY;
				//lowestSectionIndex = x;
			}
		}

		return true; // Height values found successfully
	}

	private void ApplyBuoyancy(float[] sectionWaterLines, float[] sectionWaterRollAngles, float[] sectionWaterPitchAngles, float overallMinRotatedY, out float largestSubmergedHullSectionArea, out Vector2 largestSubmergedHullSectionCentroid, out int largestSubmergedHullSectionIndex, out Vector3 avgForcePos) {
		float[] submergedAreas = new float[hull.hullSections.Count];
		largestSubmergedHullSectionArea = 0f;
		largestSubmergedHullSectionCentroid = Vector2.zero;
		largestSubmergedHullSectionIndex = 0;
		avgForcePos = Vector3.zero;

		float translationalForce = 0f;
		Vector3 translationalForceVector = Vector3.zero;

		isFullySubmerged = true; // Assume true until proven false
		isOutOfWater = true;

		numSecInWater = 0;
		float totalMagnitude = 0f;

		for (int x = 0; x < hull.hullSections.Count; x++) {

			Vector2 centerOfBuoyancy = Vector2.zero;

			bool sectionFullySubmerged = false;
			submergedAreas[x] = GetSubmergedArea(sectionWaterLines[x], shipRoll + sectionWaterRollAngles[x], x, overallMinRotatedY, out centerOfBuoyancy, out sectionFullySubmerged);
			isSectionSubmerged[x] = sectionFullySubmerged;

			if (sectionFullySubmerged) {
				submergedAreas[x] = sectionTotalArea[x];
				centerOfBuoyancy = sectionSubmergedCenterOfBuoyancy[x];
			} else {
				isFullySubmerged = false;
			}

			float sectionBuoyantForce = -waterDensity * gravity * submergedAreas[x] * (hullLength * hull.hullSections[x].normalizedSectionLength);

			Vector3 buoyantForcePos = transform.TransformPoint(new Vector3(centerOfBuoyancy.x, centerOfBuoyancy.y, hull.hullSections[x].zPos * hullLength));

			if (sectionBuoyantForce > 0) { // Apply force is section is "displacing" water
				isSectionAboveWater[x] = false;
				isOutOfWater = false;
				numSecInWater++;

				if (sectionFullySubmerged) { // If section is fully submerged, buoyant force is applied vertically

					rb.AddForceAtPosition(Vector3.up * sectionBuoyantForce, buoyantForcePos, ForceMode.Force);
					//if (debug) Debug.DrawRay(buoyantForcePos, Vector3.up * sectionBuoyantForce / debugForceScaling, Color.red);
					avgForcePos += buoyantForcePos * sectionBuoyantForce;
					totalMagnitude += sectionBuoyantForce;

				} else { // If section is partially submerged, buoyant force is applied normal to water surface

					float percentUnder = (sectionWaterLines[x] - (hull.hullSections[x].bottomY * hullHeight + hullHeight / 2f)) / (hull.hullSections[x].topY * hullHeight * 2f);

					float sectionTranslationalForce = Mathf.Sqrt(Mathf.Pow(Mathf.Sin(sectionWaterRollAngles[x] * Mathf.Deg2Rad) * sectionBuoyantForce * Mathf.Pow((1f - percentUnder), 3), 2) + Mathf.Pow(Mathf.Sin(sectionWaterPitchAngles[x] * Mathf.Deg2Rad) * sectionBuoyantForce * Mathf.Pow((1f - percentUnder), 3), 2));
					Vector3 sectionTranslationalForceVector = new Vector3(Mathf.Sin(sectionWaterRollAngles[x] * Mathf.Deg2Rad), 0, Mathf.Sin(sectionWaterPitchAngles[x] * Mathf.Deg2Rad)).normalized;

					translationalForce += sectionTranslationalForce;
					translationalForceVector += sectionTranslationalForceVector * sectionTranslationalForce;

					rb.AddForceAtPosition(Vector3.up * sectionBuoyantForce + sectionTranslationalForceVector * sectionTranslationalForce, buoyantForcePos, ForceMode.Force);
					//if (debug) Debug.DrawRay(buoyantForcePos, (Vector3.up * sectionBuoyantForce + sectionTranslationalForceVector * sectionTranslationalForce) / debugForceScaling, Color.red);
					avgForcePos += buoyantForcePos * sectionBuoyantForce;
					totalMagnitude += sectionBuoyantForce;

				}
			} else {
				isSectionAboveWater[x] = true;
			}

			if (translationalForce != 0) {
				translationalForceVector /= translationalForce;

				rb.AddForceAtPosition(translationalForceVector * translationalForce * -2f, transform.position + rb.centerOfMass, ForceMode.Force);
			}


			if (submergedAreas[x] > largestSubmergedHullSectionArea) {
				largestSubmergedHullSectionArea = submergedAreas[x];
				largestSubmergedHullSectionCentroid = centerOfBuoyancy;
				largestSubmergedHullSectionIndex = x;
			}
		}

		avgForcePos /= totalMagnitude;
	}

	private void ApplyDrag(float[] sectionWaterLines, Vector3 relativeCurrentDirection, float currentSpeed) {

		Vector3 kineticEnergy = 0.5f * new Vector3(localVelocity.x * localVelocity.x, localVelocity.y * localVelocity.y, localVelocity.z * localVelocity.z);
		Vector3 currentAdjustedVelocity = relativeCurrentDirection * currentSpeed - localVelocity;
		Vector3 kineticEnergyWater = 0.5f * waterDensity * new Vector3(currentAdjustedVelocity.x * currentAdjustedVelocity.x, currentAdjustedVelocity.y * currentAdjustedVelocity.y, currentAdjustedVelocity.z * currentAdjustedVelocity.z);

		#region Linear Drag in X
		// Drag in local x direction (left-right)
		if (localVelocity.x != 0 && abovewaterSideArea > 0) {

			Vector3 dragForce = kineticEnergy.x * airDensity * dragCoeffs.linearX * abovewaterSideArea * transform.right;
			Vector3 dragCentroid = new Vector3(-hullWidth / 2f, abovewaterSideCentroid.y, abovewaterSideCentroid.x);

			if (localVelocity.x > 0) {
				dragForce *= -1;
				dragCentroid.x *= -1;
			}

			rb.AddForceAtPosition(dragForce, transform.TransformPoint(dragCentroid), ForceMode.Force);
			if (debug) Debug.DrawRay(transform.TransformPoint(dragCentroid), dragForce / debugForceScaling, Color.gray);
		}

		if (currentAdjustedVelocity.x != 0 && underwaterSideArea > 0) {

			Vector3 dragForce = kineticEnergyWater.x * dragCoeffs.linearX * underwaterSideArea * transform.right;
			Vector3 dragCentroid = new Vector3(-hullWidth / 2f, underwaterSideCentroid.y, underwaterSideCentroid.x);

			if (currentAdjustedVelocity.x < 0) {
				dragForce *= -1;
				dragCentroid.x *= -1;
			}

			rb.AddForceAtPosition(dragForce, transform.TransformPoint(dragCentroid), ForceMode.Force);
			if (debug) Debug.DrawRay(transform.TransformPoint(dragCentroid), dragForce / debugForceScaling, Color.gray);

		}
		#endregion

		#region Linear Drag in Y
		// Drag in local y direction (up-down)
		if (localVelocity.y != 0) {
			if (localVelocity.y > 0) {
				// Moving up, only care about top side

				float topAirArea = 0f;
				Vector2 topAirCentroid = Vector2.zero;
				
				if (topOutline.Count > 2) {
					// Exposed to some region of air

					topAirArea = GetPolygonArea(topOutline.ToArray());
					topAirCentroid = GetPolygonCentroid(topOutline.ToArray(), topAirArea);
				}

				float topWaterArea = verticalHullArea - topAirArea;
				Vector2 topWaterCentroid = (verticalHullCentroid * verticalHullArea - topAirCentroid * topAirArea) / topWaterArea;

				if (topAirArea > 0) {

					float dragForce = kineticEnergy.y * airDensity * dragCoeffs.linearY * topAirArea;
					Vector3 dragCentroid = new Vector3(topAirCentroid.x, hullHeight / 2f, topAirCentroid.y);

					rb.AddForceAtPosition(dragForce * -transform.up, transform.TransformPoint(dragCentroid), ForceMode.Force);
					if (debug) Debug.DrawRay(transform.TransformPoint(dragCentroid), -transform.up * dragForce / debugForceScaling, Color.gray);

				}

				if (topWaterArea > 0) {

					float dragForce = kineticEnergy.y * waterDensity * dragCoeffs.linearY * topWaterArea;
					Vector3 dragCentroid = new Vector3(topWaterCentroid.x, hullHeight / 2f, topWaterCentroid.y);

					rb.AddForceAtPosition(dragForce * -transform.up, transform.TransformPoint(dragCentroid), ForceMode.Force);
					if (debug) Debug.DrawRay(transform.TransformPoint(dragCentroid), -transform.up * dragForce / debugForceScaling, Color.gray);

				}

				if (debug) {
					DrawOutline(OutlineTo3D(verticalHullOutline.ToArray(), Vector3.up, 0), Color.blue); // Water
					DrawOutline(OutlineTo3D(topOutline.ToArray(), Vector3.up, 0), Color.white); // Air
				}

			} else {
				// Moving down, only care about bottom side

				float bottomWaterArea = 0f;
				Vector2 bottomWaterCentroid = Vector2.zero;

				if (bottomOutline.Count > 2) {
					// Exposed to some region of air

					bottomWaterArea = GetPolygonArea(bottomOutline.ToArray());
					bottomWaterCentroid = GetPolygonCentroid(bottomOutline.ToArray(), bottomWaterArea);
				}

				float bottomAirArea = verticalHullArea - bottomWaterArea;
				Vector2 bottomAirCentroid = (verticalHullCentroid * verticalHullArea - bottomWaterCentroid * bottomWaterArea) / bottomAirArea;

				if (bottomWaterArea > 0) {

					float dragForce = kineticEnergy.y * airDensity * dragCoeffs.linearY * bottomWaterArea;
					Vector3 dragCentroid = new Vector3(bottomWaterCentroid.x, -hullHeight / 2f, bottomWaterCentroid.y);

					rb.AddForceAtPosition(dragForce * transform.up, transform.TransformPoint(dragCentroid), ForceMode.Force);
					if (debug) Debug.DrawRay(transform.TransformPoint(dragCentroid), transform.up * dragForce / debugForceScaling, Color.gray);

				}

				if (bottomAirArea > 0) {

					float dragForce = kineticEnergy.y * waterDensity * dragCoeffs.linearY * bottomAirArea;
					Vector3 dragCentroid = new Vector3(bottomAirCentroid.x, -hullHeight / 2f, bottomAirCentroid.y);

					rb.AddForceAtPosition(dragForce * transform.up, transform.TransformPoint(dragCentroid), ForceMode.Force);
					if (debug) Debug.DrawRay(transform.TransformPoint(dragCentroid), transform.up * dragForce / debugForceScaling, Color.gray);

				}

				if (debug) {
					DrawOutline(OutlineTo3D(verticalHullOutline.ToArray(), Vector3.up, 0), Color.white); // Air
					DrawOutline(OutlineTo3D(bottomOutline.ToArray(), Vector3.up, 0), Color.blue); // Water
				}
			}
		}
		#endregion

		#region Linear Drag in Z
		// Drag in local z direction (forward-back)
		if (localVelocity.z != 0 && abovewaterLongitudinalArea > 0) {

			Vector3 dragForce = kineticEnergy.z * airDensity * dragCoeffs.linearZ * abovewaterLongitudinalArea * transform.forward;
			Vector3 dragCentroid = new Vector3(abovewaterLongitudinalCentroid.x, abovewaterLongitudinalCentroid.y, -hullLength / 2f);

			if (localVelocity.z > 0) {
				dragForce *= -1;
				dragCentroid.z *= -1;
			}

			rb.AddForceAtPosition(dragForce, transform.TransformPoint(dragCentroid), ForceMode.Force);
			if (debug) Debug.DrawRay(transform.TransformPoint(dragCentroid), dragForce / debugForceScaling, Color.gray);
		}

		if (currentAdjustedVelocity.z != 0 && underwaterLongitudinalArea > 0) {

			Vector3 dragForce = kineticEnergyWater.z * dragCoeffs.linearZ * underwaterLongitudinalArea * transform.forward;
			Vector3 dragCentroid = new Vector3(underwaterLongitudinalCentroid.x, underwaterLongitudinalCentroid.y, -hullLength / 2f);

			if (currentAdjustedVelocity.z < 0) {
				dragForce *= -1;
				dragCentroid.z *= -1;
			}

			rb.AddForceAtPosition(dragForce, transform.TransformPoint(dragCentroid), ForceMode.Force);
			if (debug) Debug.DrawRay(transform.TransformPoint(dragCentroid), dragForce / debugForceScaling, Color.gray);

		}
		#endregion

		#region Angular Drag

		float dampedX = rb.angularVelocity.x;
		float dampedY = rb.angularVelocity.y;
		float dampedZ = rb.angularVelocity.z;

		if (dampedX > 0) {
			dampedX *= Mathf.Clamp01(1f - dragCoeffs.angularX * Time.fixedDeltaTime);
		} else if (dampedX < 0) {
			dampedX *= Mathf.Clamp01(1f - dragCoeffs.angularX * Time.fixedDeltaTime);
		}

		if (dampedY > 0) {
			dampedY *= Mathf.Clamp01(1f - dragCoeffs.angularY * Time.fixedDeltaTime);
		} else if (dampedY < 0) {
			dampedY *= Mathf.Clamp01(1f - dragCoeffs.angularY * Time.fixedDeltaTime);
		}

		if (dampedZ > 0) {
			dampedZ *= Mathf.Clamp01(1f - dragCoeffs.angularZ * Time.fixedDeltaTime);
		} else if (dampedZ < 0) {
			dampedZ *= Mathf.Clamp01(1f - dragCoeffs.angularZ * Time.fixedDeltaTime);
		}

		rb.angularVelocity = new Vector3(dampedX, dampedY, dampedZ);

		#endregion

		#region Rudder Drag
		if (!isOutOfWater) {
			Vector3 velocityNormal = currentAdjustedVelocity.normalized;

			Vector3 newRudderNormal = Vector3.Dot(velocityNormal, rudderNormal) < 0 ? -rudderNormal : rudderNormal;
			float waterToRudderNormal = Vector3.Dot(currentAdjustedVelocity, newRudderNormal);

			Vector3 rudderShearNormal = (currentAdjustedVelocity - waterToRudderNormal * newRudderNormal).normalized;
			float waterRudderShear = Vector3.Dot(currentAdjustedVelocity, rudderShearNormal);

			Vector3 forceNormal = rudderDragNormalCoeff * waterToRudderNormal * waterToRudderNormal * newRudderNormal;
			forceNormal.z /= rudderDragNormalCoeff;
			Vector3 forceShear = rudderDragShearCoeff * waterRudderShear * waterRudderShear * rudderShearNormal;

			Vector3 rudderForce = transform.TransformPoint(rudderDragFactor * (forceNormal + forceShear));
			rb.AddForceAtPosition(rudderForce, transform.TransformPoint(rudderCentroid), ForceMode.Force);
			if (debug) Debug.DrawRay(transform.TransformPoint(rudderCentroid), rudderForce / debugForceScaling, Color.gray);
		}
		#endregion
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
	private float GetSubmergedArea(float waterline, float waterlineAngleWithRoll, int sectionIndex, float overallMinY, out Vector2 centerOfBuoyancy, out bool isFullySubmerged) {
		
		centerOfBuoyancy = Vector2.zero;
		isFullySubmerged = false;

		float area = 0f;

		Vector2[] unnormalizedPoints = hull.GetUnnormalizedHullPoints(sectionIndex, hullHeight, hullWidth);
		Vector2[] rightHullPoints = GetRotatedHullPoints(unnormalizedPoints, waterlineAngleWithRoll);
		Vector2[] leftHullPoints = GetRotatedHullPoints(hull.GetMirroredHullPoints(unnormalizedPoints), waterlineAngleWithRoll);

		// Find lowest and highest y-position and their indices
		float localMinY = hull.hullSections[sectionIndex].topY * hullHeight;
		float localMaxY = hull.hullSections[sectionIndex].bottomY * hullHeight;
		float minX = 0f;
		float maxX = 0f;

		int localMinIndex = 0;
		for (int i = 0; i < rightHullPoints.Length; i++) {
			if (rightHullPoints[i].y < localMinY) {
				localMinY = rightHullPoints[i].y;
				localMinIndex = i;
			}
			if (rightHullPoints[i].y > localMaxY) {
				localMaxY = rightHullPoints[i].y;
			}

			if (rightHullPoints[i].x > maxX) {
				maxX = rightHullPoints[i].x;
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

			if (leftHullPoints[i].x < minX) {
				minX = leftHullPoints[i].x;
			}
		}

		if (waterline <= localMinY - overallMinY) { // Out of water

			if (localVelocity.y > 0) {
				topOutline.Insert(sectionIndex, verticalHullOutline[sectionIndex]);
				topOutline.Insert(topOutline.Count - sectionIndex, verticalHullOutline[verticalHullOutline.Count - sectionIndex - 1]);
			} else if (localVelocity.y < 0) {
				bottomOutline.Insert(sectionIndex, new Vector2(0, verticalHullOutline[sectionIndex].y));
				bottomOutline.Insert(bottomOutline.Count - sectionIndex, new Vector2(0, verticalHullOutline[sectionIndex].y));
			}

			return 0f;
		}

		// Return appropriate values if fully submerged
		if (waterline >= localMaxY - overallMinY) {
			isFullySubmerged = true;

			if (localVelocity.y < 0) {
				bottomOutline.Insert(sectionIndex, verticalHullOutline[sectionIndex]);
				bottomOutline.Insert(bottomOutline.Count - sectionIndex, verticalHullOutline[verticalHullOutline.Count - sectionIndex - 1]);
			} else if (localVelocity.y > 0) {
				topOutline.Insert(sectionIndex, new Vector2(0, verticalHullOutline[sectionIndex].y));
				topOutline.Insert(topOutline.Count - sectionIndex, new Vector2(0, verticalHullOutline[sectionIndex].y));
			}

			return 1;
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

		// Iteratively add section areas to find waterline

		for (int i = 0; i < pointsToTop - 1; i++) {

			if (newRightHullPoints[i + 1].y >= waterline) {
				// Calculate remaining area

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

				#region Vertical Outline Definition

				if (localVelocity.y > 0) {
					topOutline.Insert(sectionIndex, verticalHullOutline[sectionIndex]);
					topOutline.Insert(topOutline.Count - sectionIndex, verticalHullOutline[verticalHullOutline.Count - sectionIndex - 1]);
				} else if (localVelocity.y < 0) {
					bottomOutline.Insert(sectionIndex, new Vector2(GetUnrotatedPoint(new Vector2(-xLeft, waterline) + pivotOffset, waterlineAngleWithRoll).x, hull.hullSections[sectionIndex].zPos * hullLength));
					bottomOutline.Insert(bottomOutline.Count - sectionIndex, new Vector2(GetUnrotatedPoint(new Vector2(xRight, waterline) + pivotOffset, waterlineAngleWithRoll).x, hull.hullSections[sectionIndex].zPos * hullLength));
				}


				#endregion

				float remainingAreaLeft = (newLeftHullPoints[i].x + xLeft) * (waterline - newLeftHullPoints[i].y) / 2;
				float remainingAreaRight = (newRightHullPoints[i].x + xRight) * (waterline - newRightHullPoints[i].y) / 2;

				Vector2 sectionAreaCenterRight = new Vector2((newRightHullPoints[i].x * newRightHullPoints[i].x + newRightHullPoints[i].x * xRight + xRight * xRight) / (3 * (newRightHullPoints[i].x + xRight)), newRightHullPoints[i].y + (newRightHullPoints[i].x + 2 * xRight) * (waterline - newRightHullPoints[i].y) / (3 * (newRightHullPoints[i].x + xRight)));
				Vector2 sectionAreaCenterLeft = new Vector2(-(newLeftHullPoints[i].x * newLeftHullPoints[i].x + newLeftHullPoints[i].x * xLeft + xLeft * xLeft) / (3 * (newLeftHullPoints[i].x + xLeft)), newLeftHullPoints[i].y + (newLeftHullPoints[i].x + 2 * xLeft) * (waterline - newLeftHullPoints[i].y) / (3 * (newLeftHullPoints[i].x + xLeft)));

				centerOfBuoyancy += sectionAreaCenterRight * remainingAreaRight + sectionAreaCenterLeft * remainingAreaLeft;

				area += remainingAreaLeft + remainingAreaRight;

				centerOfBuoyancy = GetUnrotatedPoint(centerOfBuoyancy / area + pivotOffset, waterlineAngleWithRoll);
				return area;

			} else {

				float sectionAreaLeft = 0f;
				float sectionAreaRight = 0f;

				Vector2 sectionAreaCenterRight = Vector2.zero;
				Vector2 sectionAreaCenterLeft = Vector2.zero;

				if (crossesY && i == pointsToTop - 2 && waterlineAngleWithRoll > 0) {
					sectionAreaRight = newRightHullPoints[i].x / 2 * (newRightHullPoints[i + 1].y - newRightHullPoints[i].y);
					sectionAreaCenterRight = new Vector2((newRightHullPoints[i].x * newRightHullPoints[i].x) / (3 * newRightHullPoints[i].x), newRightHullPoints[i].y + (newRightHullPoints[i].x) * (newRightHullPoints[i + 1].y - newRightHullPoints[i].y) / (3 * (newRightHullPoints[i].x)));

					sectionAreaLeft = (newLeftHullPoints[i].x + newLeftHullPoints[i + 1].x) / 2 * (newLeftHullPoints[i + 1].y - newLeftHullPoints[i].y);
					sectionAreaCenterLeft = new Vector2(-(newLeftHullPoints[i].x * newLeftHullPoints[i].x + newLeftHullPoints[i].x * newLeftHullPoints[i + 1].x + newLeftHullPoints[i + 1].x * newLeftHullPoints[i + 1].x) / (3 * (newLeftHullPoints[i].x + newLeftHullPoints[i + 1].x)), newLeftHullPoints[i].y + (newLeftHullPoints[i].x + 2 * newLeftHullPoints[i + 1].x) * (newLeftHullPoints[i + 1].y - newLeftHullPoints[i].y) / (3 * (newLeftHullPoints[i].x + newLeftHullPoints[i + 1].x)));
					
				} else if (crossesY && i == pointsToTop - 2 && waterlineAngleWithRoll < 0) {
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

			}
		}

		// One side is now submerged

		if (waterlineAngleWithRoll > 0) { // CCW
			for (int i = pointsToTop - 1; i < newRightHullPoints.Count - 1; i++) {

				if (newRightHullPoints[i + 1].y >= waterline) {
					// Calculate remaining area

					float x = newRightHullPoints[i].x + (waterline - newRightHullPoints[i].y) * (newRightHullPoints[i + 1].x - newRightHullPoints[i].x) / (newRightHullPoints[i + 1].y - newRightHullPoints[i].y);

					float remainingArea = (newRightHullPoints[i].x + x) * (waterline - newRightHullPoints[i].y) / 2;

					// Calculate center of trapezoid polygon
					// https://en.wikipedia.org/wiki/Centroid#Of_a_polygon
					float xPos = 0f;
					float yPos = 0f;

					float xRight = originalNewHullPoints[i - (pointsToTop - 1)].x + (waterline - newRightHullPoints[i].y) * (originalNewHullPoints[(i + 1) - (pointsToTop - 1)].x - originalNewHullPoints[i - (pointsToTop - 1)].x) / (newRightHullPoints[i + 1].y - newRightHullPoints[i].y);
					float xLeft = aboveHalfDeckPoints[i - (pointsToTop - 1)] + (waterline - newRightHullPoints[i].y) * (aboveHalfDeckPoints[(i + 1) - (pointsToTop - 1)] - aboveHalfDeckPoints[i - (pointsToTop - 1)]) / (newRightHullPoints[i + 1].y - newRightHullPoints[i].y);

					#region Vertical Outline Definition

					if (localVelocity.y > 0) {
						topOutline.Insert(sectionIndex, new Vector2(GetUnrotatedPoint(new Vector2(xLeft, waterline) + pivotOffset, waterlineAngleWithRoll).x, hull.hullSections[sectionIndex].zPos * hullLength));
						topOutline.Insert(topOutline.Count - sectionIndex, verticalHullOutline[verticalHullOutline.Count - sectionIndex - 1]);
					} else if (localVelocity.y < 0) {
						bottomOutline.Insert(sectionIndex, verticalHullOutline[sectionIndex]);
						bottomOutline.Insert(bottomOutline.Count - sectionIndex, new Vector2(GetUnrotatedPoint(new Vector2(xRight, waterline) + pivotOffset, waterlineAngleWithRoll).x, hull.hullSections[sectionIndex].zPos * hullLength));
					}

					
					#endregion

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
					
					centerOfBuoyancy += new Vector2(xPos, yPos) * remainingArea;

					area += remainingArea;

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

					centerOfBuoyancy += new Vector2(xPos, yPos) * sectionArea;

					area += sectionArea;
				}
			}
		} else if (waterlineAngleWithRoll < 0) { // CW
			for (int i = pointsToTop - 1; i < newLeftHullPoints.Count - 1; i++) {

				if (newLeftHullPoints[i + 1].y >= waterline) {
					// Calculate remaining area

					float x = newLeftHullPoints[i].x + (waterline - newLeftHullPoints[i].y) * (newLeftHullPoints[i + 1].x - newLeftHullPoints[i].x) / (newLeftHullPoints[i + 1].y - newLeftHullPoints[i].y);

					float remainingArea = (newLeftHullPoints[i].x + x) * (waterline - newLeftHullPoints[i].y) / 2;

					// Calculate center of trapezoid polygon
					// https://en.wikipedia.org/wiki/Centroid#Of_a_polygon
					float xPos = 0f;
					float yPos = 0f;

					float xRight = originalNewHullPoints[i - (pointsToTop - 1)].x + (waterline - newLeftHullPoints[i].y) * (originalNewHullPoints[(i + 1) - (pointsToTop - 1)].x - originalNewHullPoints[i - (pointsToTop - 1)].x) / (newLeftHullPoints[i + 1].y - newLeftHullPoints[i].y);
					float xLeft = aboveHalfDeckPoints[i - (pointsToTop - 1)] + (waterline - newLeftHullPoints[i].y) * (aboveHalfDeckPoints[(i + 1) - (pointsToTop - 1)] - aboveHalfDeckPoints[i - (pointsToTop - 1)]) / (newLeftHullPoints[i + 1].y - newLeftHullPoints[i].y);

					#region Vertical Outline Definition

					if (localVelocity.y > 0) {
						topOutline.Insert(topOutline.Count - sectionIndex, verticalHullOutline[sectionIndex]);
						topOutline.Insert(sectionIndex, new Vector2(GetUnrotatedPoint(new Vector2(xRight, waterline) + pivotOffset, waterlineAngleWithRoll).x, hull.hullSections[sectionIndex].zPos * hullLength));
					} else if (localVelocity.y < 0) {
						bottomOutline.Insert(bottomOutline.Count - sectionIndex, new Vector2(GetUnrotatedPoint(new Vector2(xLeft, waterline) + pivotOffset, waterlineAngleWithRoll).x, hull.hullSections[sectionIndex].zPos * hullLength));
						bottomOutline.Insert(sectionIndex, verticalHullOutline[verticalHullOutline.Count - sectionIndex - 1]);
					}

					#endregion

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

	public void RotateRudder(bool rotateLeft) {

		if (rotateLeft) {
			rudderAngle += 50f * Time.deltaTime;
		} else {
			rudderAngle -= 50f * Time.deltaTime;
		}

		rudderAngle = Mathf.Clamp(rudderAngle, -35f, 35f);

		rudderOutline = hull.rudder.GetUnnormalizedRotatedPoints(hullWidth, hullHeight, hullLength, rudderAngle);
		rudderCentroid = hull.rudder.GetCentroid(hullWidth, hullHeight, hullLength, rudderAngle);
		rudderNormal = Vector3.Cross(rudderOutline[1] - rudderOutline[0], rudderOutline[2] - rudderOutline[0]).normalized;
	}

	public void RotateRigging(bool rotateLeft) {

		if (rotateLeft) {
			riggingAngle -= 10f * Time.deltaTime;
		} else {
			riggingAngle += 10f * Time.deltaTime;
		}

		riggingAngle = Mathf.Clamp(riggingAngle, -60f, 60f);

		foreach (Transform rigging in rotatingRigging) {
			rigging.localEulerAngles = new Vector3(0f, riggingAngle, 0f);
		}
	}

	private Vector2 GetPolygonCentroid(Vector2[] vertices, float area) {
		Vector2 centroid = Vector2.zero;

		if (area <= 0) {
			return centroid;
		}

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
		float area = GetPolygonArea(vertices);

		if (area <= 0) {
			return centroid;
		}

		for (int i = 0; i < vertices.Length - 1; i++) {
			centroid.x += (vertices[i].x + vertices[i + 1].x) * (vertices[i].x * vertices[i + 1].y - vertices[i + 1].x * vertices[i].y);
			centroid.y += (vertices[i].y + vertices[i + 1].y) * (vertices[i].x * vertices[i + 1].y - vertices[i + 1].x * vertices[i].y);
		}

		centroid.x /= 6 * area;
		centroid.y /= 6 * area;

		return centroid;
	}

	private float GetPolygonArea(Vector2[] vertices) {
		float area = 0;

		for (int i = 0; i < vertices.Length; i++) {
			area += vertices[i].x * vertices[(i + 1) % vertices.Length].y - vertices[(i + 1) % vertices.Length].x * vertices[i].y;
		}

		return Mathf.Abs(area / 2f);
	}

	public float GetTotalArea(HullSection hullSection) {

		float totalArea = 0f;
		float yOffset = hullSection.topY * hullHeight - hullSection.hullShape.hullPoints[hullSection.hullShape.hullPoints.Count - 1].y * hullSection.normalizedSectionHeight * hullHeight;

		int numPoints = hullSection.hullShape.hullPoints.Count;
		for (int i = 0; i < numPoints; i++) {
			// Area of left and right side of section
			float doubleSectionArea = ((hullSection.hullShape.hullPoints[i].x * hullSection.normalizedSectionWidth * hullWidth) + (hullSection.hullShape.hullPoints[(i + 1) % numPoints].x * hullSection.normalizedSectionWidth * hullWidth)) * ((hullSection.hullShape.hullPoints[(i + 1) % numPoints].y * hullSection.normalizedSectionHeight * hullHeight + yOffset) - (hullSection.hullShape.hullPoints[i].y * hullSection.normalizedSectionHeight * hullHeight + yOffset));
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

	/*public Vector2[] GetUnnormalizedHullPoints(HullSection hullSection)
	{
		Vector2[] unnormalizedPoints = new Vector2[hullSection.hullShape.hullPoints.Count];

		float yOffset = hullSection.topY * hullHeight - hullSection.hullShape.hullPoints[unnormalizedPoints.Length - 1].y * hullSection.normalizedSectionHeight * hullHeight;
		for (int i = 0; i < unnormalizedPoints.Length; i++)
		{
			unnormalizedPoints[i] = new Vector2(hullSection.hullShape.hullPoints[i].x * hullSection.normalizedSectionWidth * hullWidth, hullSection.hullShape.hullPoints[i].y * hullSection.normalizedSectionHeight * hullHeight + yOffset);
		}

		return unnormalizedPoints;
	}*/

	/*public Vector2[] GetLeftHullPoints(int sectionIndex) {
		Vector2[] flippedPoints = new Vector2[hullSectionShape.Length];

		// Flipping about the y-axis
		for (int i = 0; i < flippedPoints.Length; i++) {
			flippedPoints[i] = (new Vector2(-hullSectionShape[i].x, hullSectionShape[i].y));
		}

		return flippedPoints;
	}*/

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

	private void UpdateSideProperties(float[] sectionWaterLines, float largestSubmergedHullSectionArea, Vector2 largestSubmergedHullSectionCentroid, int largestSubmergedHullSectionIndex) {
		underwaterSideOutline.Clear();
		//abovewaterSideOutline.Clear();

		// Errors when only 1 section is in the water because there is only 2 points in the outline. To improve, interpolate between adjacent sections. For now, act as if entirely above water.

		if (/*allAboveWater*/ numSecInWater <= 1) { // If every section is above the water, then return
			underwaterSideArea = 0;
			abovewaterSideArea = totalSideArea;
			abovewaterSideCentroid = totalSideCentroid;
			return;
		}


		int count = 0;

		for (int i = 0; i < hull.hullSections.Count; i++) {

			float sectionWaterLineCentered = sectionWaterLines[i] - hullHeight / 2f;
			if (hull.hullSections[i].bottomY * hullHeight < sectionWaterLineCentered) { // Section is in the water

				underwaterSideOutline.Insert(count, new Vector2(hull.hullSections[i].zPos * hullLength, hull.hullSections[i].bottomY * hullHeight));

				if (hull.hullSections[i].topY * hullHeight <= sectionWaterLineCentered) { // Section is fully submerged

					underwaterSideOutline.Insert(underwaterSideOutline.Count - count, new Vector2(hull.hullSections[i].zPos * hullLength, hull.hullSections[i].topY * hullHeight));
				} else { // Section is partially submerged

					underwaterSideOutline.Insert(underwaterSideOutline.Count - count, new Vector2(hull.hullSections[i].zPos * hullLength, sectionWaterLineCentered));
				}

				count++;
			}
		}

		float underMinY = hullHeight / 2f;
		float underMinX = hullLength / 2f;
		for (int i = 0; i < underwaterSideOutline.Count; i++) {
			if (underwaterSideOutline[i].y < underMinY) {
				underMinY = underwaterSideOutline[i].y;
			}
			if (underwaterSideOutline[i].x < underMinX) {
				underMinX = underwaterSideOutline[i].x;
			}
		}

		for (int i = 0; i < underwaterSideOutline.Count; i++) {
			underwaterSideOutline[i] -= new Vector2(underMinX, underMinY);
		}

		underwaterSideArea = GetPolygonArea(underwaterSideOutline.ToArray());
		abovewaterSideArea = totalSideArea - underwaterSideArea;


		underwaterSideCentroid = GetPolygonCentroid(underwaterSideOutline.ToArray(), underwaterSideArea);
		underwaterSideCentroid += new Vector2(underMinX, underMinY);

		// --- Revert for drawing outline
		for (int i = 0; i < underwaterSideOutline.Count; i++) {
			underwaterSideOutline[i] += new Vector2(underMinX, underMinY);
		}
		// ---

		//abovewaterSideCentroid = GetPolygonCentroid(abovewaterSideOutline.ToArray(), abovewaterSideArea);
		//abovewaterSideCentroid += new Vector2(aboveMinX, aboveMinY);

		abovewaterSideCentroid = (totalSideCentroid * totalSideArea - underwaterSideCentroid * underwaterSideArea) / abovewaterSideArea;


		// Longitudinal Ends

		underwaterLongitudinalArea = largestSubmergedHullSectionArea;
		underwaterLongitudinalCentroid = largestSubmergedHullSectionCentroid;
		abovewaterLongitudinalArea = sectionTotalArea[largestSubmergedHullSectionIndex] - largestSubmergedHullSectionArea;
		abovewaterLongitudinalCentroid = (sectionSubmergedCenterOfBuoyancy[largestSubmergedHullSectionIndex] * sectionTotalArea[largestSubmergedHullSectionIndex] - largestSubmergedHullSectionCentroid * largestSubmergedHullSectionArea) / abovewaterLongitudinalArea;
		
	}

	private float GetTotalSideAreaAndCentroid(out Vector2 sideCentroid) {
		float sideArea = 0f;
		Vector2[] sideOutline = new Vector2[hull.hullSections.Count * 2];

		for (int i = 0; i < hull.hullSections.Count; i++) {
			sideOutline[i] = new Vector2(hull.hullSections[i].zPos * hullLength, hull.hullSections[i].bottomY * hullHeight);
			sideOutline[hull.hullSections.Count * 2 - i - 1] = new Vector2(hull.hullSections[i].zPos * hullLength, hull.hullSections[i].topY * hullHeight);
		}

		// --- For Drawing Outline
		totalSideOutline = sideOutline.ToList();
		// ---

		float minY = hullHeight / 2f;
		float minX = hullLength / 2f;
		for (int i = 0; i < sideOutline.Length; i++) {
			if (sideOutline[i].y < minY) {
				minY = sideOutline[i].y;
			}
			if (sideOutline[i].x < minX) {
				minX = sideOutline[i].x;
			}
		}

		for (int i = 0; i < sideOutline.Length; i++) {
			sideOutline[i] -= new Vector2(minX, minY);
		}

		sideArea = GetPolygonArea(sideOutline.ToArray());
		

		if (sideArea == 0) {
			sideCentroid = Vector2.zero;
		} else {
			sideCentroid = GetPolygonCentroid(sideOutline.ToArray(), sideArea);
		}

		sideCentroid += new Vector2(minX, minY);

		return sideArea;
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

	private float GetSectionMinRotatedY(int sectionIndex, float waterlineAngle)
	{
		Vector2[] unnormalizedPoints = hull.GetUnnormalizedHullPoints(sectionIndex, hullHeight, hullWidth);
		Vector2[] rightHullPoints = GetRotatedHullPoints(unnormalizedPoints, waterlineAngle);
		Vector2[] leftHullPoints = GetRotatedHullPoints(hull.GetMirroredHullPoints(unnormalizedPoints), waterlineAngle);

		float minY = hull.hullSections[sectionIndex].topY * hullHeight;

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

	private Vector3[] OutlineTo3D(Vector2[] points, Vector3 normal, float pos) {
		Vector3[] output = new Vector3[points.Length];

		for (int i = 0; i < points.Length; i++) {
			if (normal.x == 1) {
				output[i] = new Vector3(pos, points[i].y, points[i].x);
			} else if (normal.y == 1) {
				output[i] = new Vector3(points[i].x, pos, points[i].y);
			} else {
				output[i] = new Vector3(points[i].x, points[i].y, pos);
			}
		}

		transform.TransformPoints(output);

		return output;
	}

	private void DrawOutline(Vector3[] points, Color color, bool obscure = false) {
		if (points.Length < 3) return;

		for (int i = 0; i < points.Length - 1; i++) {
			Debug.DrawLine(points[i], points[i + 1], color, 0, obscure);
		}
		Debug.DrawLine(points[points.Length - 1], points[0], color, 0, obscure);
	}

	private void OnDestroy() {
		waterSurfaceHeightSearch.Destroy();
	}
}

[System.Serializable]
struct DirectionalDrag {
	public float linearX;
	public float linearY;
	public float linearZ;
	public float angularX;
	public float angularY;
	public float angularZ;
}