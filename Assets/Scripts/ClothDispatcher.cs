using System;
using System.Runtime.InteropServices;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.Collections;
using Unity.VisualScripting.FullSerializer;
using UnityEngine;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using UnityEngine.Rendering;
using UnityEngine.UIElements;
using System.Threading.Tasks;
using Unity.Burst;
using Unity.Jobs;
using System.Threading;

public class ClothDispatcher : MonoBehaviour
{
    public ComputeShader clothCompute;

	[Range(0, 50)]
	public float windSpeed = 1f; // m/s
	[Min(1)]
	public int projectionIterations = 3;
	public bool isSimulating = true;
	public bool isRendering = true;
	public bool isReadingBackMesh = false;
	public bool gravityActive = true;
	public bool windActive = true;
	public Material clothMaterial;
	public List<Texture2D> textures = new List<Texture2D>();
	private Vector3[] clothsLastPosition;
	private ClothSimulation[] cloths;

	#region ClothCompute Shader Kernels
	const int updateVelocityKernel = 0;
	const int predictPositionKernel = 1;
	const int solveStretchingKernel1 = 2;
	const int solveStretchingKernel2 = 3;
	const int solveStretchingKernel3 = 4;
	const int solveStretchingKernel4 = 5;
	const int solveStretchingKernel5 = 6;
	const int solveStretchingKernel6 = 7;
	const int solveBendingKernel1 = 8;
	const int solveBendingKernel2 = 9;
	const int solveBendingKernel3 = 10;
	const int solveBendingKernel4 = 11;
	const int solveBendingKernel5 = 12;
	const int solveBendingKernel6 = 13;
	const int calculateTriangleNormals = 14;
	const int calculateVertexNormals = 15;
	#endregion

	private

	#region Compute Buffers
	ComputeBuffer xBuffer;
	ComputeBuffer vBuffer;
	ComputeBuffer pBuffer;
	ComputeBuffer wBuffer;
	ComputeBuffer stepVelocityBuffer;
	ComputeBuffer gravityVectorBuffer;
	ComputeBuffer windVectorBuffer;
	ComputeBuffer startIndicesBuffer;
	ComputeBuffer substepsBuffer;
	ComputeBuffer stretchingAlphaBuffer;
	ComputeBuffer bendingAlphaBuffer;
	ComputeBuffer stepTimeBuffer;
	ComputeBuffer dampingBuffer;
	ComputeBuffer maxVelocityBuffer;
	ComputeBuffer[] d0Buffers;
	ComputeBuffer[] dihedral0Buffers;
	ComputeBuffer[] bendingIDsBuffers;
	ComputeBuffer[] stretchingIDsBuffers;
	ComputeBuffer normalsBuffer;
	ComputeBuffer trianglesBuffer;
	ComputeBuffer vertexToTrianglesBuffer;
	ComputeBuffer triangleNormalsBuffer;
	ComputeBuffer uvBuffer;
	GraphicsBuffer[] commandBuffers;
	GraphicsBuffer transformMatrixBuffer;
	ComputeBuffer[] renderedTriangleLocalStartIndexBuffers;
	ComputeBuffer[] renderedTriangleOffsetsBuffers;
	ComputeBuffer[] sideToInstanceBuffers;
	ComputeBuffer dragFactorBuffer;
	ComputeBuffer windForceBuffer;
	ComputeBuffer pinnedVertBuffer;

	#endregion

	#region Native Arrays
	NativeArray<Vector3> xNative; // Vertices
	NativeArray<float> wNative; // Inverse Mass of Vertex
	NativeArray<Vector3> vNative; // Velocity of Vertex
	NativeArray<int> substepsNative; // Number of Substeps for each cloth
	NativeArray<float> stretchingAlphaNative;
	NativeArray<float> bendingAlphaNative;
	NativeArray<float> stepTimeNative;
	NativeArray<float> dampingNative;
	NativeArray<float> maxVelocityNative;
	NativeArray<bool> handleCollisionsNative;
	NativeArray<bool> applyGravityNative;
	NativeArray<bool> applyWindNative;
	NativeArray<int> numVertsPerSubstepNative;
	NativeArray<int> sortedClothIndicesNative;
	NativeArray<Vector3> normalsNative;
	NativeArray<Vector3> normalsInverseNative;
	NativeArray<int> trianglesNative;
	NativeArray<AdjacentTriangleIndices> vertexToTrianglesNative;
	NativeArray<Vector3> triangleNormalsNative;
	NativeArray<Vector2> uvNative;
	NativeArray<int> activeTriangleStartIndexNative;
	NativeArray<int> textureSortedClothsNative;
	NativeArray<int> activeVertexStartIndexNative;
	NativeArray<int> renderedTriangleLocalStartIndexNative;
	NativeArray<int> renderedTriangleOffsetsNative;
	NativeArray<int> renderedToSortedNative;
	NativeArray<int> activeBackTriangleStartIndexNative;
	NativeArray<int> sideToInstanceNative;
	NativeArray<Vector3> stepVelocitiesNative;
	NativeArray<Vector3> localGravityVectorsNative;
	NativeArray<Vector3> localWindVectorsNative;
	NativeArray<float> dragFactorNative;
	NativeArray<Vector3> windForcesNative;
	#endregion

	List<List<float>> d0 = new List<List<float>>();
	List<List<float>> dihedral0 = new List<List<float>>();
	List<List<int>> numD0PerSubstep = new List<List<int>>();
	List<List<int>> numDihedral0PerSubstep = new List<List<int>>();
	List<List<int>> stretchingIDs = new List<List<int>>();
	List<List<int>> bendingIDs = new List<List<int>>();
	List<int> simulatedClothIndices = new List<int>();
	List<int> activeClothIndices = new List<int>();
	List<int> numStepsPerLoop = new List<int>();
	List<int> substepsIndexPerLoop = new List<int>();
	List<int> textureGroupLastIndex = new List<int>();
	List<int> textureGroupFirstIndex = new List<int>();
	List<int> numSidesPerGroup = new List<int>();

	bool buffersGenerated = false;
	const int stretchBatches = 6;
	const int bendingBatches = 6;
	int numActiveVerts = 0; // How many vertices are in the active cloths
	int numSimulatedVerts = 0; // How many vertices are in the cloths being simulated
	int numRenderedVerts = 0; // How many vertices are in the cloths being rendered
	int totalVerts = 0;
	int numActiveCloths = 0; // How many cloths are active in the hierarchy
	int numSimulatedCloths = 0;
	int numRenderedCloths = 0;
	int numTrianglesTotal = 0;
	int numOneSidedTrianglesTotal = 0;
	int numSimulatedTriangles = 0;

	float dragCoeffPerp = 1.28f;
	float dragCoeffShear = 0.2f;

	MaterialPropertyBlock[] matProps;
	RenderParams rp;

	private AsyncGPUReadbackRequest xReadbackRequest;
	private AsyncGPUReadbackRequest normalsReadbackRequest;
	private AsyncGPUReadbackRequest windForceReadbackRequest;
	private bool pendingMeshReadback = false;
	private bool pendingForceReadback = false;

	[StructLayout(LayoutKind.Sequential)]
	private struct AdjacentTriangleIndices {
		public int id0;
		public int id1;
		public int id2;
		public int id3;
		public int id4;
		public int id5;

		public AdjacentTriangleIndices(int id0 = -1, int id1 = -1, int id2 = -1, int id3 = -1, int id4 = -1, int id5 = -1, int offset = 0) {
			this.id0 = (id0 != -1) ? (id0 + offset) : -1;
			this.id1 = (id1 != -1) ? (id1 + offset) : -1;
			this.id2 = (id2 != -1) ? (id2 + offset) : -1;
			this.id3 = (id3 != -1) ? (id3 + offset) : -1;
			this.id4 = (id4 != -1) ? (id4 + offset) : -1;
			this.id5 = (id5 != -1) ? (id5 + offset) : -1;
		}

		public override string ToString() {
			return $"[{id0}, {id1}, {id2}, {id3}, {id4}, {id5}]";
		}
	}

	[BurstCompile]
	private struct InvertNormalsJob : IJobParallelFor {
		[ReadOnly]
		public NativeArray<Vector3> normalsIn;

		[WriteOnly]
		public NativeArray<Vector3> normalsOut;

		public void Execute(int index) {
			normalsOut[index] = -normalsIn[index];
		}
	}

	// Start is called before the first frame update
	void Start()
    {
		clothMaterial = new Material(clothMaterial);

		rp = new RenderParams(clothMaterial);
		rp.worldBounds = new Bounds(Vector3.zero, 1000 * Vector3.one); // use tighter bounds

		InitializeDispatcher();
	}

    // Update is called once per frame
    void FixedUpdate()
    {
		if (!buffersGenerated) {
			return;
		}

		/*bool clothStateChanged = false;
		for (int i = 0; i < cloths.Length; i++) {
			if ((cloths[i].isInitialized && cloths[i].gameObject.activeInHierarchy) != cloths[i].isSimulating) {
				clothStateChanged = true;
				break;
			}
		}

		if (clothStateChanged) {
			Debug.Log("Cloth state changed");
			CreateBuffers();
		}*/

		if (isSimulating && numSimulatedCloths != 0) {
			for (int i = 0; i < numSimulatedCloths; i++) {
				for (int j = 0; j < cloths[sortedClothIndicesNative[i]].pinnedVertices.Count; j++) {
					xNative[cloths[sortedClothIndicesNative[i]].pinnedVertices[j] + activeVertexStartIndexNative[i]] = cloths[sortedClothIndicesNative[i]].transform.TransformPoint(cloths[sortedClothIndicesNative[i]].pinnedVertLocalPos[j]);
				}

				// Acceleration Due to Gravity, Acceleration
				localGravityVectorsNative[i] = (gravityActive && applyGravityNative[i]) ? Physics.gravity : Vector3.zero;

				// Wind Velocity, Velocity
				localWindVectorsNative[i] = (windActive && applyWindNative[i]) ? transform.forward * windSpeed * UnityEngine.Random.Range(0.9f, 1.1f) + Vector3.up * UnityEngine.Random.Range(-1f, 1f) + Vector3.right * UnityEngine.Random.Range(-1f, 1f) : Vector3.zero;
			}

			for (int i = 0; i < cloths.Length; i++) {
				clothsLastPosition[i] = cloths[i].transform.position;
			}

			/*if (handleCollisions) {
				hash.Create(x);
				//float maxTravelDist = maxVelocity * dt;
				float maxTravelDist = thickness;
				hash.QueryAll(x, maxTravelDist);
			}*/

			gravityVectorBuffer.SetData(localGravityVectorsNative);
			windVectorBuffer.SetData(localWindVectorsNative);
			pinnedVertBuffer.SetData(xNative);

			clothCompute.SetBuffer(predictPositionKernel, "gravityVector", gravityVectorBuffer);
			clothCompute.SetBuffer(predictPositionKernel, "windVector", windVectorBuffer);
			clothCompute.SetBuffer(predictPositionKernel, "pinnedVertPos", pinnedVertBuffer);
			clothCompute.SetBuffer(updateVelocityKernel, "pinnedVertPos", pinnedVertBuffer);

			SimulationLoop();
		}

		if (isReadingBackMesh && !pendingMeshReadback) {
			xReadbackRequest = AsyncGPUReadback.Request(xBuffer);
			normalsReadbackRequest = AsyncGPUReadback.Request(normalsBuffer);
			pendingMeshReadback = true;
		}

		if (!pendingForceReadback) {
			windForceReadbackRequest = AsyncGPUReadback.Request(windForceBuffer);
			pendingForceReadback = true;
		}
	}

	void Update() {

		if (!buffersGenerated) {
			Refresh();
			return;
		}


		if (isReadingBackMesh && pendingMeshReadback) {
			if (xReadbackRequest.done && normalsReadbackRequest.done) {
				if (xReadbackRequest.hasError || normalsReadbackRequest.hasError) {
					//Debug.LogError("GPU Readback Error!");
				} else {

					xNative = xReadbackRequest.GetData<Vector3>();
					normalsNative = normalsReadbackRequest.GetData<Vector3>();

					//normalsInverse = normals.Select(x => -x).ToArray();
					InvertNormalsJob invertJob = new InvertNormalsJob {
						normalsIn = normalsNative,
						normalsOut = normalsInverseNative
					};

					JobHandle jobHandle = invertJob.Schedule(numActiveVerts, 64);
					jobHandle.Complete();

					// Update positions for each cloth
					for (int i = 0; i < numActiveCloths; i++) {
						// Copy specific cloth's data from global arrays
						unsafe {
							// Copy Vertices
							int size = Marshal.SizeOf(typeof(Vector3)) * cloths[sortedClothIndicesNative[i]].numOneSidedVerts;
							IntPtr sourcePtr = (IntPtr)((Vector3*)xNative.GetUnsafePtr() + activeVertexStartIndexNative[i]); // Calculate offset for the subset
							IntPtr destPtr = Marshal.UnsafeAddrOfPinnedArrayElement(cloths[sortedClothIndicesNative[i]].x, 0);
							UnsafeUtility.MemCpy((void*)destPtr, (void*)sourcePtr, size);

							// Copy Normals
							size = Marshal.SizeOf(typeof(Vector3)) * cloths[sortedClothIndicesNative[i]].numOneSidedVerts;
							sourcePtr = (IntPtr)((Vector3*)normalsNative.GetUnsafePtr() + activeVertexStartIndexNative[i]); // Calculate offset for the subset
							destPtr = Marshal.UnsafeAddrOfPinnedArrayElement(cloths[sortedClothIndicesNative[i]].normals, 0);
							UnsafeUtility.MemCpy((void*)destPtr, (void*)sourcePtr, size);

							if (cloths[sortedClothIndicesNative[i]].isDoubleSided) {
								// Copy Vertices
								size = Marshal.SizeOf(typeof(Vector3)) * cloths[sortedClothIndicesNative[i]].numOneSidedVerts;
								sourcePtr = (IntPtr)((Vector3*)xNative.GetUnsafePtr() + activeVertexStartIndexNative[i]); // Calculate offset for the subset
								destPtr = Marshal.UnsafeAddrOfPinnedArrayElement(cloths[sortedClothIndicesNative[i]].x, cloths[sortedClothIndicesNative[i]].numOneSidedVerts);
								UnsafeUtility.MemCpy((void*)destPtr, (void*)sourcePtr, size);

								// Copy Backside Normals
								size = Marshal.SizeOf(typeof(Vector3)) * cloths[sortedClothIndicesNative[i]].numOneSidedVerts;
								sourcePtr = (IntPtr)((Vector3*)normalsInverseNative.GetUnsafePtr() + activeVertexStartIndexNative[i]); // Calculate offset for the subset
								destPtr = Marshal.UnsafeAddrOfPinnedArrayElement(cloths[sortedClothIndicesNative[i]].normals, cloths[sortedClothIndicesNative[i]].numOneSidedVerts);
								UnsafeUtility.MemCpy((void*)destPtr, (void*)sourcePtr, size);
							}
						}
					}
				}

				pendingMeshReadback = false;
			}
		}

		if (pendingForceReadback) {
			if (windForceReadbackRequest.done) {
				if (windForceReadbackRequest.hasError) {
					Debug.LogError("GPU Readback Error!");
				} else {
					windForcesNative = windForceReadbackRequest.GetData<Vector3>();

					for (int i = 0; i < numActiveCloths; i++) {
						// Copy specific cloth's data from global arrays
						unsafe {
							// Copy Wind Forces
							int size = Marshal.SizeOf(typeof(Vector3)) * cloths[sortedClothIndicesNative[i]].numOneSidedVerts;
							IntPtr sourcePtr = (IntPtr)((Vector3*)windForcesNative.GetUnsafePtr() + activeVertexStartIndexNative[i]); // Calculate offset for the subset
							//IntPtr destPtr = Marshal.UnsafeAddrOfPinnedArrayElement(cloths[sortedClothIndicesNative[i]].windForcesNative, 0);
							IntPtr destPtr = (IntPtr)((Vector3*)cloths[sortedClothIndicesNative[i]].windForcesNative.GetUnsafePtr());
							UnsafeUtility.MemCpy((void*)destPtr, (void*)sourcePtr, size);
						}
					}
				}

				pendingForceReadback = false;
			}
		}

		// Render cloths
		if (isRendering && numRenderedCloths > 0) {
			Matrix4x4[] instanceMatrices = new Matrix4x4[numActiveCloths];
			for (int i = 0; i < numActiveCloths; i++) {
				instanceMatrices[i] = cloths[sortedClothIndicesNative[i]].transform.localToWorldMatrix;
			}

			transformMatrixBuffer.SetData(instanceMatrices);
			clothMaterial.SetBuffer("InstanceTransforms", transformMatrixBuffer);

			//Graphics.RenderPrimitivesIndirect(rp, MeshTopology.Triangles, commandBuffer, commandCount);

			for (int i = 0; i < commandBuffers.Length; i++) {
				Graphics.DrawProceduralIndirect(clothMaterial, new Bounds(Vector3.zero, 1000 * Vector3.one), MeshTopology.Triangles, commandBuffers[i], 0, null, matProps[i], ShadowCastingMode.On, true, gameObject.layer);
			}
		}

	}

	private void SimulationLoop() {
		int step = 0;
		for (int i = 0; i < numStepsPerLoop.Count; i++) {
			for (int j = 0; j < numStepsPerLoop[i]; j++) {
				clothCompute.SetInt("globalSubstep", step);
				step++;

				// Predict new positions
				ComputeHelper.Dispatch(clothCompute, numVertsPerSubstepNative[substepsIndexPerLoop[i]], kernelIndex: predictPositionKernel);

				// Solve stretching constraint for each batch of edges
				ComputeHelper.Dispatch(clothCompute, numD0PerSubstep[0][substepsIndexPerLoop[i]], kernelIndex: solveStretchingKernel1);
				ComputeHelper.Dispatch(clothCompute, numD0PerSubstep[1][substepsIndexPerLoop[i]], kernelIndex: solveStretchingKernel2);
				ComputeHelper.Dispatch(clothCompute, numD0PerSubstep[2][substepsIndexPerLoop[i]], kernelIndex: solveStretchingKernel3);
				ComputeHelper.Dispatch(clothCompute, numD0PerSubstep[3][substepsIndexPerLoop[i]], kernelIndex: solveStretchingKernel4);
				ComputeHelper.Dispatch(clothCompute, numD0PerSubstep[4][substepsIndexPerLoop[i]], kernelIndex: solveStretchingKernel5);
				ComputeHelper.Dispatch(clothCompute, numD0PerSubstep[5][substepsIndexPerLoop[i]], kernelIndex: solveStretchingKernel6);

				// Solve bending constraint for each batch of triangle pairs
				ComputeHelper.Dispatch(clothCompute, numDihedral0PerSubstep[0][substepsIndexPerLoop[i]], kernelIndex: solveBendingKernel1);
				ComputeHelper.Dispatch(clothCompute, numDihedral0PerSubstep[1][substepsIndexPerLoop[i]], kernelIndex: solveBendingKernel2);
				ComputeHelper.Dispatch(clothCompute, numDihedral0PerSubstep[2][substepsIndexPerLoop[i]], kernelIndex: solveBendingKernel3);
				ComputeHelper.Dispatch(clothCompute, numDihedral0PerSubstep[3][substepsIndexPerLoop[i]], kernelIndex: solveBendingKernel4);
				ComputeHelper.Dispatch(clothCompute, numDihedral0PerSubstep[4][substepsIndexPerLoop[i]], kernelIndex: solveBendingKernel5);
				ComputeHelper.Dispatch(clothCompute, numDihedral0PerSubstep[5][substepsIndexPerLoop[i]], kernelIndex: solveBendingKernel6);

				// Update velocities
				ComputeHelper.Dispatch(clothCompute, numVertsPerSubstepNative[substepsIndexPerLoop[i]], kernelIndex: updateVelocityKernel);
			}
		}

		ComputeHelper.Dispatch(clothCompute, numOneSidedTrianglesTotal, kernelIndex: calculateTriangleNormals);
		ComputeHelper.Dispatch(clothCompute, numSimulatedVerts, kernelIndex: calculateVertexNormals);
	}


	private void InitializeDispatcher() {

		// Clear and initialize lists for stretch constraints
		d0?.Clear();
		numD0PerSubstep?.Clear();
		stretchingIDs?.Clear();

		for (int i = 0; i < stretchBatches; i++) {
			d0.Add(new List<float>());
			numD0PerSubstep.Add(new List<int>());
			stretchingIDs.Add(new List<int>());
		}

		// Clear and initialize lists for bending constraints
		dihedral0?.Clear();
		numDihedral0PerSubstep?.Clear();
		bendingIDs?.Clear();

		for (int i = 0; i < bendingBatches; i++) {
			dihedral0.Add(new List<float>());
			numDihedral0PerSubstep.Add(new List<int>());
			bendingIDs.Add(new List<int>());
		}
	}

	private void Refresh() {
		buffersGenerated = false;

		UpdateClothInfo();

		if (!UpdateSimulationBuffers()) {
			Debug.Log("No simulated cloths");
			return;
		}

		if (!UpdateRenderingBuffers()) {
			Debug.Log("No rendered cloths");
			return;
		}

		buffersGenerated = true;
	}

	private void UpdateClothInfo() {
		// Finds all Cloths (Inactive included)
		cloths = Resources.FindObjectsOfTypeAll<ClothSimulation>();

		// Record cloth information
		totalVerts = 0;
		numActiveCloths = 0;
		numSimulatedCloths = 0;
		numRenderedCloths = 0;

		simulatedClothIndices?.Clear();
		activeClothIndices?.Clear();

		if (cloths.Length > 0) clothsLastPosition = new Vector3[cloths.Length];

		numActiveVerts = 0;
		numSimulatedVerts = 0;
		numRenderedVerts = 0;
		numTrianglesTotal = 0;
		numOneSidedTrianglesTotal = 0;

		for (int i = 0; i < cloths.Length; i++) {
			clothsLastPosition[i] = cloths[i].transform.position;

			// Add the number of vertices representing a single side of the cloth
			totalVerts += cloths[i].numOneSidedVerts;

			if (cloths[i].isInitialized && cloths[i].isActive) {
				activeClothIndices.Add(i);

				// Add the number of vertices representing a single side of the cloth
				numActiveVerts += cloths[i].numOneSidedVerts;
				numOneSidedTrianglesTotal += cloths[i].numOneSidedTriangles;
				numTrianglesTotal += cloths[i].triangles.Length / 3;

				if (cloths[i].isSimulating) {
					simulatedClothIndices.Add(i);
					numSimulatedVerts += cloths[i].numOneSidedVerts;
					numSimulatedTriangles += cloths[i].numOneSidedTriangles;
				}

				if (cloths[i].isRendering) {
					numRenderedCloths++;
					numRenderedVerts += cloths[i].numOneSidedVerts;
				}

			}
		}

		Debug.Log("Total Verts: " + numActiveVerts);

		numActiveCloths = activeClothIndices.Count;
		numSimulatedCloths = simulatedClothIndices.Count;
	}

	private bool UpdateSimulationBuffers() {
		// New dispose function for just simulating natives & buffers

		ReleaseSimulationBuffersAndNatives();

		if (numActiveCloths == 0) return false; // Return if no cloths are active/initialized/visible

		// Clear Lists

		numStepsPerLoop?.Clear();
		substepsIndexPerLoop?.Clear();

		for (int i = 0; i < stretchBatches; i++) {
			d0[i].Clear();
			numD0PerSubstep[i].Clear();
			stretchingIDs[i].Clear();
		}

		for (int i = 0; i < bendingBatches; i++) {
			dihedral0[i].Clear();
			numDihedral0PerSubstep[i].Clear();
			bendingIDs[i].Clear();
		};

		// Create Arrays
		trianglesNative = new NativeArray<int>(numTrianglesTotal * 3, Allocator.Temp);
		uvNative = new NativeArray<Vector2>(numActiveVerts, Allocator.Temp);
		wNative = new NativeArray<float>(numSimulatedVerts, Allocator.Temp);
		vNative = new NativeArray<Vector3>(numSimulatedVerts, Allocator.Temp);
		stretchingAlphaNative = new NativeArray<float>(numSimulatedCloths, Allocator.Temp);
		bendingAlphaNative = new NativeArray<float>(numSimulatedCloths, Allocator.Temp);
		stepTimeNative = new NativeArray<float>(numSimulatedCloths, Allocator.Temp);
		dampingNative = new NativeArray<float>(numSimulatedCloths, Allocator.Temp);
		maxVelocityNative = new NativeArray<float>(numSimulatedCloths, Allocator.Temp);
		handleCollisionsNative = new NativeArray<bool>(numSimulatedCloths, Allocator.Temp);
		vertexToTrianglesNative = new NativeArray<AdjacentTriangleIndices>(numSimulatedVerts, Allocator.Temp);
		triangleNormalsNative = new NativeArray<Vector3>(numSimulatedTriangles, Allocator.Temp);
		dragFactorNative = new NativeArray<float>(numSimulatedVerts, Allocator.Temp);

		xNative = new NativeArray<Vector3>(numActiveVerts, Allocator.Persistent);
		normalsNative = new NativeArray<Vector3>(numActiveVerts, Allocator.Persistent);
		normalsInverseNative = new NativeArray<Vector3>(numActiveVerts, Allocator.Persistent);
		stepVelocitiesNative = new NativeArray<Vector3>(numSimulatedCloths, Allocator.Persistent);
		localGravityVectorsNative = new NativeArray<Vector3>(numSimulatedCloths, Allocator.Persistent);
		localWindVectorsNative = new NativeArray<Vector3>(numSimulatedCloths, Allocator.Persistent);
		activeVertexStartIndexNative = new NativeArray<int>(numActiveCloths, Allocator.Persistent);
		activeTriangleStartIndexNative = new NativeArray<int>(numActiveCloths, Allocator.Persistent);
		activeBackTriangleStartIndexNative = new NativeArray<int>(numActiveCloths, Allocator.Persistent);
		sortedClothIndicesNative = new NativeArray<int>(numActiveCloths, Allocator.Persistent);
		substepsNative = new NativeArray<int>(numActiveCloths, Allocator.Persistent);
		applyGravityNative = new NativeArray<bool>(numSimulatedCloths, Allocator.Persistent);
		applyWindNative = new NativeArray<bool>(numSimulatedCloths, Allocator.Persistent);
		numVertsPerSubstepNative = new NativeArray<int>(numSimulatedCloths, Allocator.Persistent);
		windForcesNative = new NativeArray<Vector3>(numSimulatedVerts, Allocator.Persistent);

		// Sort simulated cloths by number of substeps

		int addedCloths = 0;
		for (int i = 0; i < numSimulatedCloths; i++) {
			sortedClothIndicesNative[addedCloths] = simulatedClothIndices[i];
			substepsNative[addedCloths++] = cloths[simulatedClothIndices[i]].substeps;
		}
		for (int i = 0; i < numActiveCloths; i++) {
			if (!cloths[activeClothIndices[i]].isSimulating) {
				sortedClothIndicesNative[addedCloths] = activeClothIndices[i];
				substepsNative[addedCloths++] = 0;
			}
		}

		QuickSort(substepsNative, sortedClothIndicesNative, 0, substepsNative.Length - 1);

		/*string[] names = new string[numActiveCloths];
		for (int i = 0; i < numActiveCloths; i++) {
			names[i] = cloths[sortedClothIndicesNative[i]].name;
		}

		Debug.Log("Sorted Cloths: " + string.Join(", ", names));*/

		// Populate Arrays

		int backIndex = -1;

		for (int i = 0; i < numActiveCloths; i++) {

			int clothIndex = sortedClothIndicesNative[i];

			if (i == 0) {
				activeVertexStartIndexNative[i] = 0;
				activeTriangleStartIndexNative[i] = 0;

				if (cloths[clothIndex].isDoubleSided) {
					if (backIndex == -1) {
						activeBackTriangleStartIndexNative[i] = 0;
					} else {
						activeBackTriangleStartIndexNative[i] = activeBackTriangleStartIndexNative[backIndex] + cloths[sortedClothIndicesNative[backIndex]].numOneSidedTriangles * 3;
					}
					backIndex = i;
				}
			} else {
				activeVertexStartIndexNative[i] = activeVertexStartIndexNative[i - 1] + cloths[sortedClothIndicesNative[i - 1]].numOneSidedVerts;
				activeTriangleStartIndexNative[i] = activeTriangleStartIndexNative[i - 1] + cloths[sortedClothIndicesNative[i - 1]].numOneSidedTriangles * 3;

				if (cloths[clothIndex].isDoubleSided) {
					if (backIndex == -1) {
						activeBackTriangleStartIndexNative[i] = 0;
					} else {
						activeBackTriangleStartIndexNative[i] = activeBackTriangleStartIndexNative[backIndex] + cloths[sortedClothIndicesNative[backIndex]].numOneSidedTriangles * 3;
					}
					backIndex = i;
				}
			}

			// Copy vertices, triangle indices, normals, and uv's from individual cloths to combined array
			unsafe {
				// Copy Vertices
				int size = Marshal.SizeOf(typeof(Vector3)) * cloths[clothIndex].numOneSidedVerts;
				IntPtr sourcePtr = Marshal.UnsafeAddrOfPinnedArrayElement(cloths[clothIndex].x, 0);
				IntPtr destPtr = (IntPtr)((Vector3*)xNative.GetUnsafePtr() + activeVertexStartIndexNative[i]); // Calculate offset for the subset
				UnsafeUtility.MemCpy((void*)destPtr, (void*)sourcePtr, size);

				// Copy Frontside Normals
				size = Marshal.SizeOf(typeof(Vector3)) * cloths[clothIndex].numOneSidedVerts;
				sourcePtr = Marshal.UnsafeAddrOfPinnedArrayElement(cloths[clothIndex].normals, 0);
				destPtr = (IntPtr)((Vector3*)normalsNative.GetUnsafePtr() + activeVertexStartIndexNative[i]); // Calculate offset for the subset
				UnsafeUtility.MemCpy((void*)destPtr, (void*)sourcePtr, size);

				if (cloths[clothIndex].isDoubleSided) {
					// Copy Triangle Indices
					size = Marshal.SizeOf(typeof(int)) * cloths[clothIndex].numOneSidedTriangles * 3;
					sourcePtr = Marshal.UnsafeAddrOfPinnedArrayElement(cloths[clothIndex].triangles, cloths[clothIndex].numOneSidedTriangles * 3);
					destPtr = (IntPtr)((int*)trianglesNative.GetUnsafePtr() + activeTriangleStartIndexNative[i]); // Calculate offset for the subset
					UnsafeUtility.MemCpy((void*)destPtr, (void*)sourcePtr, size);

					sourcePtr = Marshal.UnsafeAddrOfPinnedArrayElement(cloths[clothIndex].triangles, 0);
					destPtr = (IntPtr)((int*)trianglesNative.GetUnsafePtr() + activeBackTriangleStartIndexNative[i] + numOneSidedTrianglesTotal * 3); // Calculate offset for the subset
					UnsafeUtility.MemCpy((void*)destPtr, (void*)sourcePtr, size);

					for (int j = 0; j < cloths[clothIndex].numOneSidedTriangles * 3; j++) {
						trianglesNative[activeTriangleStartIndexNative[i] + j] += activeVertexStartIndexNative[i];
						trianglesNative[activeBackTriangleStartIndexNative[i] + numOneSidedTrianglesTotal * 3 + j] += activeVertexStartIndexNative[i];
					}

				} else {
					// Copy Triangle Indices
					size = Marshal.SizeOf(typeof(int)) * cloths[clothIndex].numOneSidedTriangles * 3;
					sourcePtr = Marshal.UnsafeAddrOfPinnedArrayElement(cloths[clothIndex].triangles, 0);
					destPtr = (IntPtr)((int*)trianglesNative.GetUnsafePtr() + activeTriangleStartIndexNative[i]); // Calculate offset for the subset
					UnsafeUtility.MemCpy((void*)destPtr, (void*)sourcePtr, size);

					for (int j = 0; j < cloths[clothIndex].numOneSidedTriangles; j++) {
						int temp = trianglesNative[activeTriangleStartIndexNative[i] + j * 3 + 2];
						trianglesNative[activeTriangleStartIndexNative[i] + j * 3 + 2] = trianglesNative[activeTriangleStartIndexNative[i] + j * 3 + 1];
						trianglesNative[activeTriangleStartIndexNative[i] + j * 3 + 1] = temp;

						trianglesNative[activeTriangleStartIndexNative[i] + j * 3] += activeVertexStartIndexNative[i];
						trianglesNative[activeTriangleStartIndexNative[i] + j * 3 + 1] += activeVertexStartIndexNative[i];
						trianglesNative[activeTriangleStartIndexNative[i] + j * 3 + 2] += activeVertexStartIndexNative[i];
					}

					
				}

				// Copy UVs
				size = Marshal.SizeOf(typeof(Vector2)) * cloths[clothIndex].numOneSidedVerts;
				sourcePtr = Marshal.UnsafeAddrOfPinnedArrayElement(cloths[clothIndex].uv, 0);
				destPtr = (IntPtr)((Vector2*)uvNative.GetUnsafePtr() + activeVertexStartIndexNative[i]); // Calculate offset for the subset
				UnsafeUtility.MemCpy((void*)destPtr, (void*)sourcePtr, size);
			}

			// If cloth is being simulated
			if (i < numSimulatedCloths) {
				float dt_step = Time.fixedDeltaTime / substepsNative[i];
				stretchingAlphaNative[i] = cloths[clothIndex].stretchingCompliance / (dt_step * dt_step);
				bendingAlphaNative[i] = cloths[clothIndex].bendingCompliance / (dt_step * dt_step);
				stepTimeNative[i] = dt_step;
				dampingNative[i] = cloths[clothIndex].damping;
				maxVelocityNative[i] = 4 * substepsNative[i];
				handleCollisionsNative[i] = cloths[clothIndex].handleCollisions;
				applyGravityNative[i] = cloths[clothIndex].applyGravity;
				applyWindNative[i] = cloths[clothIndex].applyWind;

				// Count number of iterations to dispatch per simulation step
				for (int j = i; j >= 0; j--) {
					numVertsPerSubstepNative[i] += cloths[sortedClothIndicesNative[j]].numOneSidedVerts;

					if (j == i) {
						for (int k = 0; k < stretchBatches; k++) {
							numD0PerSubstep[k].Add(cloths[sortedClothIndicesNative[j]].d0[k].Length);
						}
						for (int k = 0; k < bendingBatches; k++) {
							numDihedral0PerSubstep[k].Add(cloths[sortedClothIndicesNative[j]].dihedral0[k].Length);
						}
					} else {
						for (int k = 0; k < stretchBatches; k++) {
							numD0PerSubstep[k][i] += cloths[sortedClothIndicesNative[j]].d0[k].Length;
						}
						for (int k = 0; k < bendingBatches; k++) {
							numDihedral0PerSubstep[k][i] += cloths[sortedClothIndicesNative[j]].dihedral0[k].Length;
						}
					}
				}

				// Copy inverse mass and drag factors from individual cloths to combined array
				unsafe {
					// Copy Inverse Masses
					int size = Marshal.SizeOf(typeof(float)) * cloths[clothIndex].numOneSidedVerts;
					IntPtr sourcePtr = Marshal.UnsafeAddrOfPinnedArrayElement(cloths[clothIndex].w, 0);
					IntPtr destPtr = (IntPtr)((float*)wNative.GetUnsafePtr() + activeVertexStartIndexNative[i]); // Calculate offset for the subset
					UnsafeUtility.MemCpy((void*)destPtr, (void*)sourcePtr, size);

					// Copy Drag Factors
					size = Marshal.SizeOf(typeof(float)) * cloths[clothIndex].numOneSidedVerts;
					sourcePtr = Marshal.UnsafeAddrOfPinnedArrayElement(cloths[clothIndex].dragFactor, 0);
					destPtr = (IntPtr)((float*)dragFactorNative.GetUnsafePtr() + activeVertexStartIndexNative[i]); // Calculate offset for the subset
					UnsafeUtility.MemCpy((void*)destPtr, (void*)sourcePtr, size);
				}

				int triangleOffset = activeTriangleStartIndexNative[i] / 3;
				for (int j = 0; j < cloths[clothIndex].vertexToTriangles.Length; j++) {
					if (cloths[clothIndex].vertexToTriangles[j].Count == 6) {
						vertexToTrianglesNative[activeVertexStartIndexNative[i] + j] = new AdjacentTriangleIndices(cloths[clothIndex].vertexToTriangles[j][0], cloths[clothIndex].vertexToTriangles[j][1], cloths[clothIndex].vertexToTriangles[j][2], cloths[clothIndex].vertexToTriangles[j][3], cloths[clothIndex].vertexToTriangles[j][4], cloths[clothIndex].vertexToTriangles[j][5], triangleOffset);
					} else if (cloths[clothIndex].vertexToTriangles[j].Count == 5) {
						vertexToTrianglesNative[activeVertexStartIndexNative[i] + j] = new AdjacentTriangleIndices(cloths[clothIndex].vertexToTriangles[j][0], cloths[clothIndex].vertexToTriangles[j][1], cloths[clothIndex].vertexToTriangles[j][2], cloths[clothIndex].vertexToTriangles[j][3], cloths[clothIndex].vertexToTriangles[j][4], offset: triangleOffset);
					} else if (cloths[clothIndex].vertexToTriangles[j].Count == 4) {
						vertexToTrianglesNative[activeVertexStartIndexNative[i] + j] = new AdjacentTriangleIndices(cloths[clothIndex].vertexToTriangles[j][0], cloths[clothIndex].vertexToTriangles[j][1], cloths[clothIndex].vertexToTriangles[j][2], cloths[clothIndex].vertexToTriangles[j][3], offset: triangleOffset);
					} else if (cloths[clothIndex].vertexToTriangles[j].Count == 3) {
						vertexToTrianglesNative[activeVertexStartIndexNative[i] + j] = new AdjacentTriangleIndices(cloths[clothIndex].vertexToTriangles[j][0], cloths[clothIndex].vertexToTriangles[j][1], cloths[clothIndex].vertexToTriangles[j][2], offset: triangleOffset);
					} else if (cloths[clothIndex].vertexToTriangles[j].Count == 2) {
						vertexToTrianglesNative[activeVertexStartIndexNative[i] + j] = new AdjacentTriangleIndices(cloths[clothIndex].vertexToTriangles[j][0], cloths[clothIndex].vertexToTriangles[j][1], offset: triangleOffset);
					} else if (cloths[clothIndex].vertexToTriangles[j].Count == 1) {
						vertexToTrianglesNative[activeVertexStartIndexNative[i] + j] = new AdjacentTriangleIndices(cloths[clothIndex].vertexToTriangles[j][0], offset: triangleOffset);
					} else {
						vertexToTrianglesNative[activeVertexStartIndexNative[i] + j] = new AdjacentTriangleIndices();
					}
				}

				// Create combined constraint information
				for (int j = 0; j < stretchBatches; j++) {
					d0[j].AddRange(cloths[clothIndex].d0[j]);
				}

				for (int j = 0; j < bendingBatches; j++) {
					dihedral0[j].AddRange(cloths[clothIndex].dihedral0[j]);
				}

				for (int j = 0; j < stretchBatches; j++) {
					for (int k = 0; k < cloths[clothIndex].stretchingIDs[j].Length; k++) {
						stretchingIDs[j].Add((cloths[clothIndex].stretchingIDs[j])[k] + activeVertexStartIndexNative[i]);
					}
				}

				for (int j = 0; j < bendingBatches; j++) {
					for (int k = 0; k < cloths[clothIndex].bendingIDs[j].Length; k++) {
						bendingIDs[j].Add((cloths[clothIndex].bendingIDs[j])[k] + activeVertexStartIndexNative[i]);
					}
				}
			}
		}

		// Set Mesh Data Buffers

		xBuffer = ComputeHelper.CreateStructuredBuffer<Vector3>(numActiveVerts);
		xBuffer.SetData(xNative);
		clothCompute.SetBuffer(calculateTriangleNormals, "x", xBuffer);
		clothCompute.SetBuffer(updateVelocityKernel, "x", xBuffer);
		clothCompute.SetBuffer(predictPositionKernel, "x", xBuffer);

		normalsBuffer = ComputeHelper.CreateStructuredBuffer<Vector3>(normalsNative.Length);
		normalsBuffer.SetData(normalsNative);
		clothCompute.SetBuffer(predictPositionKernel, "normals", normalsBuffer);
		clothCompute.SetBuffer(calculateVertexNormals, "normals", normalsBuffer);

		trianglesBuffer = ComputeHelper.CreateStructuredBuffer<int>(numTrianglesTotal * 3);
		trianglesBuffer.SetData(trianglesNative);
		clothCompute.SetBuffer(calculateTriangleNormals, "triangles", trianglesBuffer);

		uvBuffer = ComputeHelper.CreateStructuredBuffer<Vector2>(numActiveVerts);
		uvBuffer.SetData(uvNative);

		startIndicesBuffer = ComputeHelper.CreateStructuredBuffer<int>(numActiveCloths);
		startIndicesBuffer.SetData(activeVertexStartIndexNative);
		clothCompute.SetBuffer(updateVelocityKernel, "startIndex", startIndicesBuffer);
		clothCompute.SetBuffer(predictPositionKernel, "startIndex", startIndicesBuffer);
		clothCompute.SetBuffer(solveStretchingKernel1, "startIndex", startIndicesBuffer);
		clothCompute.SetBuffer(solveStretchingKernel2, "startIndex", startIndicesBuffer);
		clothCompute.SetBuffer(solveStretchingKernel3, "startIndex", startIndicesBuffer);
		clothCompute.SetBuffer(solveStretchingKernel4, "startIndex", startIndicesBuffer);
		clothCompute.SetBuffer(solveStretchingKernel5, "startIndex", startIndicesBuffer);
		clothCompute.SetBuffer(solveStretchingKernel6, "startIndex", startIndicesBuffer);
		clothCompute.SetBuffer(solveBendingKernel1, "startIndex", startIndicesBuffer);
		clothCompute.SetBuffer(solveBendingKernel2, "startIndex", startIndicesBuffer);
		clothCompute.SetBuffer(solveBendingKernel3, "startIndex", startIndicesBuffer);
		clothCompute.SetBuffer(solveBendingKernel4, "startIndex", startIndicesBuffer);
		clothCompute.SetBuffer(solveBendingKernel5, "startIndex", startIndicesBuffer);
		clothCompute.SetBuffer(solveBendingKernel6, "startIndex", startIndicesBuffer);

		substepsBuffer = ComputeHelper.CreateStructuredBuffer<int>(numActiveCloths);
		substepsBuffer.SetData(substepsNative);
		clothCompute.SetBuffer(predictPositionKernel, "substeps", substepsBuffer);
		clothCompute.SetBuffer(updateVelocityKernel, "substeps", substepsBuffer);
		clothCompute.SetBuffer(solveStretchingKernel1, "substeps", substepsBuffer);
		clothCompute.SetBuffer(solveStretchingKernel2, "substeps", substepsBuffer);
		clothCompute.SetBuffer(solveStretchingKernel3, "substeps", substepsBuffer);
		clothCompute.SetBuffer(solveStretchingKernel4, "substeps", substepsBuffer);
		clothCompute.SetBuffer(solveStretchingKernel5, "substeps", substepsBuffer);
		clothCompute.SetBuffer(solveStretchingKernel6, "substeps", substepsBuffer);
		clothCompute.SetBuffer(solveBendingKernel1, "substeps", substepsBuffer);
		clothCompute.SetBuffer(solveBendingKernel2, "substeps", substepsBuffer);
		clothCompute.SetBuffer(solveBendingKernel3, "substeps", substepsBuffer);
		clothCompute.SetBuffer(solveBendingKernel4, "substeps", substepsBuffer);
		clothCompute.SetBuffer(solveBendingKernel5, "substeps", substepsBuffer);
		clothCompute.SetBuffer(solveBendingKernel6, "substeps", substepsBuffer);

		clothCompute.SetInt("numCloths", numActiveCloths);
		clothCompute.SetFloat("projectionIterations", projectionIterations);

		clothMaterial.SetBuffer("Vertices", xBuffer);
		clothMaterial.SetBuffer("Triangles", trianglesBuffer);
		clothMaterial.SetBuffer("Normals", normalsBuffer);
		clothMaterial.SetBuffer("UVs", uvBuffer);

		// Set Simulation Buffers

		if (numSimulatedCloths == 0) return true;

		vBuffer = ComputeHelper.CreateStructuredBuffer<Vector3>(numSimulatedVerts);
		vBuffer.SetData(vNative);
		clothCompute.SetBuffer(updateVelocityKernel, "v", vBuffer);
		clothCompute.SetBuffer(predictPositionKernel, "v", vBuffer);

		vertexToTrianglesBuffer = ComputeHelper.CreateStructuredBuffer<AdjacentTriangleIndices>(numSimulatedVerts);
		vertexToTrianglesBuffer.SetData(vertexToTrianglesNative);
		clothCompute.SetBuffer(calculateVertexNormals, "vertexToTriangle", vertexToTrianglesBuffer);

		triangleNormalsBuffer = ComputeHelper.CreateStructuredBuffer<Vector3>(numSimulatedTriangles);
		triangleNormalsBuffer.SetData(triangleNormalsNative);
		clothCompute.SetBuffer(calculateVertexNormals, "triangleNormals", triangleNormalsBuffer);
		clothCompute.SetBuffer(calculateTriangleNormals, "triangleNormals", triangleNormalsBuffer);

		wBuffer = ComputeHelper.CreateStructuredBuffer<float>(numSimulatedVerts);
		wBuffer.SetData(wNative);
		clothCompute.SetBuffer(updateVelocityKernel, "w", wBuffer);
		clothCompute.SetBuffer(predictPositionKernel, "w", wBuffer);
		clothCompute.SetBuffer(solveStretchingKernel1, "w", wBuffer);
		clothCompute.SetBuffer(solveStretchingKernel2, "w", wBuffer);
		clothCompute.SetBuffer(solveStretchingKernel3, "w", wBuffer);
		clothCompute.SetBuffer(solveStretchingKernel4, "w", wBuffer);
		clothCompute.SetBuffer(solveStretchingKernel5, "w", wBuffer);
		clothCompute.SetBuffer(solveStretchingKernel6, "w", wBuffer);
		clothCompute.SetBuffer(solveBendingKernel1, "w", wBuffer);
		clothCompute.SetBuffer(solveBendingKernel2, "w", wBuffer);
		clothCompute.SetBuffer(solveBendingKernel3, "w", wBuffer);
		clothCompute.SetBuffer(solveBendingKernel4, "w", wBuffer);
		clothCompute.SetBuffer(solveBendingKernel5, "w", wBuffer);
		clothCompute.SetBuffer(solveBendingKernel6, "w", wBuffer);

		pBuffer = ComputeHelper.CreateStructuredBuffer<Vector3>(numSimulatedVerts);
		clothCompute.SetBuffer(updateVelocityKernel, "p", pBuffer);
		clothCompute.SetBuffer(predictPositionKernel, "p", pBuffer);
		clothCompute.SetBuffer(solveStretchingKernel1, "p", pBuffer);
		clothCompute.SetBuffer(solveStretchingKernel2, "p", pBuffer);
		clothCompute.SetBuffer(solveStretchingKernel3, "p", pBuffer);
		clothCompute.SetBuffer(solveStretchingKernel4, "p", pBuffer);
		clothCompute.SetBuffer(solveStretchingKernel5, "p", pBuffer);
		clothCompute.SetBuffer(solveStretchingKernel6, "p", pBuffer);
		clothCompute.SetBuffer(solveBendingKernel1, "p", pBuffer);
		clothCompute.SetBuffer(solveBendingKernel2, "p", pBuffer);
		clothCompute.SetBuffer(solveBendingKernel3, "p", pBuffer);
		clothCompute.SetBuffer(solveBendingKernel4, "p", pBuffer);
		clothCompute.SetBuffer(solveBendingKernel5, "p", pBuffer);
		clothCompute.SetBuffer(solveBendingKernel6, "p", pBuffer);

		dragFactorBuffer = ComputeHelper.CreateStructuredBuffer<float>(numSimulatedVerts);
		dragFactorBuffer.SetData(dragFactorNative);
		clothCompute.SetBuffer(predictPositionKernel, "dragFactor", dragFactorBuffer);

		windForceBuffer = ComputeHelper.CreateStructuredBuffer<Vector3>(numSimulatedVerts);
		clothCompute.SetBuffer(predictPositionKernel, "windForce", windForceBuffer);

		pinnedVertBuffer = ComputeHelper.CreateStructuredBuffer<Vector3>(numActiveVerts);
		pinnedVertBuffer.SetData(xNative);
		clothCompute.SetBuffer(predictPositionKernel, "pinnedVertPos", pinnedVertBuffer);
		clothCompute.SetBuffer(updateVelocityKernel, "pinnedVertPos", pinnedVertBuffer);

		d0Buffers = new ComputeBuffer[stretchBatches];
		stretchingIDsBuffers = new ComputeBuffer[stretchBatches];
		for (int i = 0; i < stretchBatches; i++) {
			d0Buffers[i] = ComputeHelper.CreateStructuredBuffer<float>(d0[i].Count);
			d0Buffers[i].SetData(d0[i]);

			stretchingIDsBuffers[i] = ComputeHelper.CreateStructuredBuffer<int>(stretchingIDs[i].Count);
			stretchingIDsBuffers[i].SetData(stretchingIDs[i]);
		}

		dihedral0Buffers = new ComputeBuffer[bendingBatches];
		bendingIDsBuffers = new ComputeBuffer[bendingBatches];
		for (int i = 0; i < bendingBatches; i++) {
			dihedral0Buffers[i] = ComputeHelper.CreateStructuredBuffer<float>(dihedral0[i].Count);
			dihedral0Buffers[i].SetData(dihedral0[i]);

			bendingIDsBuffers[i] = ComputeHelper.CreateStructuredBuffer<int>(bendingIDs[i].Count);
			bendingIDsBuffers[i].SetData(bendingIDs[i]);
		}

		clothCompute.SetBuffer(solveStretchingKernel1, "d0", d0Buffers[0]);
		clothCompute.SetBuffer(solveStretchingKernel1, "stretchingIDs", stretchingIDsBuffers[0]);
		clothCompute.SetBuffer(solveStretchingKernel2, "d0", d0Buffers[1]);
		clothCompute.SetBuffer(solveStretchingKernel2, "stretchingIDs", stretchingIDsBuffers[1]);
		clothCompute.SetBuffer(solveStretchingKernel3, "d0", d0Buffers[2]);
		clothCompute.SetBuffer(solveStretchingKernel3, "stretchingIDs", stretchingIDsBuffers[2]);
		clothCompute.SetBuffer(solveStretchingKernel4, "d0", d0Buffers[3]);
		clothCompute.SetBuffer(solveStretchingKernel4, "stretchingIDs", stretchingIDsBuffers[3]);
		clothCompute.SetBuffer(solveStretchingKernel5, "d0", d0Buffers[4]);
		clothCompute.SetBuffer(solveStretchingKernel5, "stretchingIDs", stretchingIDsBuffers[4]);
		clothCompute.SetBuffer(solveStretchingKernel6, "d0", d0Buffers[5]);
		clothCompute.SetBuffer(solveStretchingKernel6, "stretchingIDs", stretchingIDsBuffers[5]);

		clothCompute.SetBuffer(solveBendingKernel1, "dihedral0", dihedral0Buffers[0]);
		clothCompute.SetBuffer(solveBendingKernel1, "bendingIDs", bendingIDsBuffers[0]);
		clothCompute.SetBuffer(solveBendingKernel2, "dihedral0", dihedral0Buffers[1]);
		clothCompute.SetBuffer(solveBendingKernel2, "bendingIDs", bendingIDsBuffers[1]);
		clothCompute.SetBuffer(solveBendingKernel3, "dihedral0", dihedral0Buffers[2]);
		clothCompute.SetBuffer(solveBendingKernel3, "bendingIDs", bendingIDsBuffers[2]);
		clothCompute.SetBuffer(solveBendingKernel4, "dihedral0", dihedral0Buffers[3]);
		clothCompute.SetBuffer(solveBendingKernel4, "bendingIDs", bendingIDsBuffers[3]);
		clothCompute.SetBuffer(solveBendingKernel5, "dihedral0", dihedral0Buffers[4]);
		clothCompute.SetBuffer(solveBendingKernel5, "bendingIDs", bendingIDsBuffers[4]);
		clothCompute.SetBuffer(solveBendingKernel6, "dihedral0", dihedral0Buffers[5]);
		clothCompute.SetBuffer(solveBendingKernel6, "bendingIDs", bendingIDsBuffers[5]);

		stepVelocityBuffer = ComputeHelper.CreateStructuredBuffer<Vector3>(numSimulatedVerts);
		clothCompute.SetBuffer(updateVelocityKernel, "stepVelocity", stepVelocityBuffer);
		clothCompute.SetBuffer(predictPositionKernel, "stepVelocity", stepVelocityBuffer);

		gravityVectorBuffer = ComputeHelper.CreateStructuredBuffer<Vector3>(numSimulatedCloths);
		windVectorBuffer = ComputeHelper.CreateStructuredBuffer<Vector3>(numSimulatedCloths);

		stretchingAlphaBuffer = ComputeHelper.CreateStructuredBuffer<float>(numSimulatedCloths);
		stretchingAlphaBuffer.SetData(stretchingAlphaNative);
		clothCompute.SetBuffer(solveStretchingKernel1, "stretchingAlpha", stretchingAlphaBuffer);
		clothCompute.SetBuffer(solveStretchingKernel2, "stretchingAlpha", stretchingAlphaBuffer);
		clothCompute.SetBuffer(solveStretchingKernel3, "stretchingAlpha", stretchingAlphaBuffer);
		clothCompute.SetBuffer(solveStretchingKernel4, "stretchingAlpha", stretchingAlphaBuffer);
		clothCompute.SetBuffer(solveStretchingKernel5, "stretchingAlpha", stretchingAlphaBuffer);
		clothCompute.SetBuffer(solveStretchingKernel6, "stretchingAlpha", stretchingAlphaBuffer);

		bendingAlphaBuffer = ComputeHelper.CreateStructuredBuffer<float>(numSimulatedCloths);
		bendingAlphaBuffer.SetData(bendingAlphaNative);
		clothCompute.SetBuffer(solveBendingKernel1, "bendingAlpha", bendingAlphaBuffer);
		clothCompute.SetBuffer(solveBendingKernel2, "bendingAlpha", bendingAlphaBuffer);
		clothCompute.SetBuffer(solveBendingKernel3, "bendingAlpha", bendingAlphaBuffer);
		clothCompute.SetBuffer(solveBendingKernel4, "bendingAlpha", bendingAlphaBuffer);
		clothCompute.SetBuffer(solveBendingKernel5, "bendingAlpha", bendingAlphaBuffer);
		clothCompute.SetBuffer(solveBendingKernel6, "bendingAlpha", bendingAlphaBuffer);

		stepTimeBuffer = ComputeHelper.CreateStructuredBuffer<float>(numSimulatedCloths);
		stepTimeBuffer.SetData(stepTimeNative);
		clothCompute.SetBuffer(predictPositionKernel, "stepTime", stepTimeBuffer);
		clothCompute.SetBuffer(updateVelocityKernel, "stepTime", stepTimeBuffer);

		dampingBuffer = ComputeHelper.CreateStructuredBuffer<float>(numSimulatedCloths);
		dampingBuffer.SetData(dampingNative);
		clothCompute.SetBuffer(updateVelocityKernel, "damping", dampingBuffer);

		maxVelocityBuffer = ComputeHelper.CreateStructuredBuffer<float>(numSimulatedCloths);
		maxVelocityBuffer.SetData(maxVelocityNative);
		clothCompute.SetBuffer(predictPositionKernel, "maxVelocity", maxVelocityBuffer);

		clothCompute.SetInt("numTriangles", numSimulatedTriangles);
		clothCompute.SetFloat("dragCoeffPerp", dragCoeffPerp);
		clothCompute.SetFloat("dragCoeffShear", dragCoeffShear);

		numStepsPerLoop.Add(substepsNative[numSimulatedCloths - 1]);
		substepsIndexPerLoop.Add((numSimulatedCloths - 1));
		for (int i = numSimulatedCloths - 1; i > 0; i--) {
			int skip = 1;
			int numSubsteps = substepsNative[i - skip] - substepsNative[i];
			while (numSubsteps == 0) {
				skip++;
				if (i - skip < 0) break;
				numSubsteps = substepsNative[i - skip] - substepsNative[i];
			}

			if (numSubsteps != 0) {
				numStepsPerLoop.Add(numSubsteps);
				substepsIndexPerLoop.Add(i - skip);
			}

			i -= skip - 1;
		}

		DisposeTempSimulationNatives();

		return true; // Successful creation of buffers
	}

	private bool UpdateRenderingBuffers() {

		textureGroupFirstIndex?.Clear();
		textureGroupLastIndex?.Clear();
		numSidesPerGroup?.Clear();
		
		ReleaseRenderingBuffers();

		if (numRenderedCloths == 0) return false; // Return if no cloths are active/initialized/visible

		textureSortedClothsNative = new NativeArray<int>(numRenderedCloths, Allocator.Temp);
		renderedToSortedNative = new NativeArray<int>(numRenderedCloths, Allocator.Temp);

		int numAddedCloths = 0;
		int numTextureGroups = 0;

		unsafe {
			void* ptr = textureSortedClothsNative.GetUnsafePtr();
			// Fill all bytes of the array with 0xFF.
			// For a signed 32-bit integer, 0xFFFFFFFF (all bits set) represents -1.
			UnsafeUtility.MemSet(ptr, 0xFF, numRenderedCloths * UnsafeUtility.SizeOf<int>());
		}

		for (int i = 0; i < numActiveCloths; i++) {
			int clothIndex = sortedClothIndicesNative[i];

			if (cloths[clothIndex].isRendering && !textureSortedClothsNative.Contains(clothIndex)) {
				textureGroupFirstIndex.Add(numAddedCloths);

				int currentTexture = (int)cloths[clothIndex].clothTexture;
				renderedToSortedNative[numAddedCloths] = i;
				textureSortedClothsNative[numAddedCloths++] = clothIndex;
				numSidesPerGroup.Add(cloths[clothIndex].isDoubleSided ? 2 : 1);

				for (int j = i + 1; j < numActiveCloths; j++) {

					if ((int)cloths[sortedClothIndicesNative[j]].clothTexture == currentTexture) {

						renderedToSortedNative[numAddedCloths] = j;
						textureSortedClothsNative[numAddedCloths++] = sortedClothIndicesNative[j];
						numSidesPerGroup[numTextureGroups] += cloths[sortedClothIndicesNative[j]].isDoubleSided ? 2 : 1;

					}
				}

				numTextureGroups++;
				textureGroupLastIndex.Add(numAddedCloths - 1);
			}
		}


		commandBuffers = new GraphicsBuffer[numTextureGroups];
		matProps = new MaterialPropertyBlock[numTextureGroups];
		renderedTriangleLocalStartIndexBuffers = new ComputeBuffer[numTextureGroups];
		renderedTriangleOffsetsBuffers = new ComputeBuffer[numTextureGroups];
		sideToInstanceBuffers = new ComputeBuffer[numTextureGroups];

		for (int i = 0; i < numTextureGroups; i++) {

			renderedTriangleLocalStartIndexNative = new NativeArray<int>(numSidesPerGroup[i], Allocator.Temp);
			renderedTriangleOffsetsNative = new NativeArray<int>(numSidesPerGroup[i], Allocator.Temp);
			sideToInstanceNative = new NativeArray<int>(numSidesPerGroup[i], Allocator.Temp);

			int numTrianglesInGroup = 0;

			int sideCount = 0;
			for (int j = textureGroupFirstIndex[i]; j < textureGroupLastIndex[i] + 1; j++) {
				int clothIndex = textureSortedClothsNative[j];

				sideToInstanceNative[sideCount] = renderedToSortedNative[j];
				renderedTriangleLocalStartIndexNative[sideCount] = numTrianglesInGroup;
				renderedTriangleOffsetsNative[sideCount] = activeTriangleStartIndexNative[renderedToSortedNative[j]];

				numTrianglesInGroup += cloths[clothIndex].numOneSidedTriangles * 3;

				sideCount++;

				if (cloths[clothIndex].isDoubleSided) {
					sideToInstanceNative[sideCount] = renderedToSortedNative[j];
					renderedTriangleLocalStartIndexNative[sideCount] = numTrianglesInGroup;
					renderedTriangleOffsetsNative[sideCount] = activeBackTriangleStartIndexNative[renderedToSortedNative[j]] + numOneSidedTrianglesTotal * 3;
					numTrianglesInGroup += cloths[clothIndex].numOneSidedTriangles * 3;

					sideCount++;
				}

			}

			renderedTriangleLocalStartIndexBuffers[i] = ComputeHelper.CreateStructuredBuffer<int>(numSidesPerGroup[i]);
			renderedTriangleLocalStartIndexBuffers[i].SetData(renderedTriangleLocalStartIndexNative);

			renderedTriangleOffsetsBuffers[i] = ComputeHelper.CreateStructuredBuffer<int>(numSidesPerGroup[i]);
			renderedTriangleOffsetsBuffers[i].SetData(renderedTriangleOffsetsNative);

			sideToInstanceBuffers[i] = ComputeHelper.CreateStructuredBuffer<int>(numSidesPerGroup[i]);
			sideToInstanceBuffers[i].SetData(sideToInstanceNative);

			matProps[i] = new MaterialPropertyBlock();
			matProps[i].SetTexture("_BaseMap", textures[(int)cloths[textureSortedClothsNative[textureGroupFirstIndex[i]]].clothTexture]);
			matProps[i].SetBuffer("TriangleLocalStartIndex", renderedTriangleLocalStartIndexBuffers[i]);
			matProps[i].SetBuffer("TriangleOffsets", renderedTriangleOffsetsBuffers[i]);
			matProps[i].SetBuffer("SideToInstance", sideToInstanceBuffers[i]);
			matProps[i].SetInt("numSides", sideCount);

			GraphicsBuffer.IndirectDrawArgs[] commandData = new GraphicsBuffer.IndirectDrawArgs[1];

			commandData[0].vertexCountPerInstance = (uint)numTrianglesInGroup;
			commandData[0].instanceCount = 1;
			commandData[0].startVertex = 0;
			commandData[0].startInstance = 0;

			commandBuffers[i] = new GraphicsBuffer(GraphicsBuffer.Target.IndirectArguments, 1, GraphicsBuffer.IndirectDrawArgs.size);
			commandBuffers[i].SetData(commandData);

		}

		transformMatrixBuffer = new GraphicsBuffer(GraphicsBuffer.Target.Structured, numActiveCloths, Marshal.SizeOf(typeof(Matrix4x4)));

		clothMaterial.SetInt("oneSidedNumTriangles", numOneSidedTrianglesTotal * 3);
		clothMaterial.SetTexture("_BaseMap", textures[0]);

		//Debug.Log("Vertex Start Index: " + string.Join(", ", activeVertexStartIndexNative));
		//Debug.Log("Simulated Vertex Start: " + string.Join(", ", simulatedVertexStartIndexNative));
		//Debug.Log("Vertex to Triangles: " + string.Join(", ", vertexToTrianglesNative));

		//Debug.Log("Triangles Start Index: " + string.Join(", ", activeTriangleStartIndexNative));
		//Debug.Log("Back Triangles Start: " + string.Join(", ", activeBackTriangleStartIndexNative));
		//Debug.Log("Triangles: " + string.Join(", ", trianglesNative));
		//Debug.Log("Triangles (1/4): " + string.Join(", ", trianglesNative.Take(numOneSidedTrianglesTotal * 3 / numActiveCloths)));
		//Debug.Log("Triangles (2/4): " + string.Join(", ", trianglesNative.Skip(numOneSidedTrianglesTotal * 3 / numActiveCloths).Take(numOneSidedTrianglesTotal * 3 / numActiveCloths)));
		//Debug.Log("Triangles (3/4): " + string.Join(", ", trianglesNative.Skip(numOneSidedTrianglesTotal * 3 / numActiveCloths * 2).Take(numOneSidedTrianglesTotal * 3 / numActiveCloths)));
		//Debug.Log("Triangles (4/4): " + string.Join(", ", trianglesNative.Skip(numOneSidedTrianglesTotal * 3 / numActiveCloths * 3).Take(numOneSidedTrianglesTotal * 3 / numActiveCloths)));

		//Debug.Log("Triangles (1/4): " + string.Join(", ", trianglesNative.Take(activeTriangleStartIndexNative[1])));
		//Debug.Log("Triangles (2/4): " + string.Join(", ", trianglesNative.Skip(activeTriangleStartIndexNative[1]).Take(activeTriangleStartIndexNative[1])));
		//Debug.Log("Triangles (3/4): " + string.Join(", ", trianglesNative.Skip(trianglesNative.Length / 2).Take(activeTriangleStartIndexNative[1])));
		//Debug.Log("Triangles (4/4): " + string.Join(", ", trianglesNative.Skip(trianglesNative.Length / 2 + activeTriangleStartIndexNative[1]).Take(activeTriangleStartIndexNative[1])));

		//Debug.Log("Triangle Offsets: " + string.Join(", ", renderedTriangleOffsetsNative));
		//Debug.Log("Triangle Local Start Index: " + string.Join(", ", renderedTriangleLocalStartIndexNative));

		/*int[] offsetIDs = new int[numTrianglesTotal * 3];
		int[] sideIDs = new int[numTrianglesTotal * 3];
		for (int i = 0; i < numTrianglesTotal * 3; i++) {

			if (numSidesPerGroup[0] > 0) {
				sideIDs[i] = 0;

				for (int j = 1; j < numSidesPerGroup[0]; j++) {
					if (i < renderedTriangleLocalStartIndexNative[j]) {
						sideIDs[i] = j - 1;
						break;
					} else if (j == numSidesPerGroup[0] - 1) {
						sideIDs[i] = j;
						break;
					}

				}
			}

			offsetIDs[i] = i + renderedTriangleOffsetsNative[sideIDs[i]] - renderedTriangleLocalStartIndexNative[sideIDs[i]];
		}

		Debug.Log("Side IDs: " + string.Join(", ", sideIDs));
		Debug.Log("Side To Instance IDs: " + string.Join(", ", sideToInstanceNative));
		Debug.Log("Offset Triangle IDs: " + string.Join(", ", offsetIDs));*/
		//Debug.Log("X: " + string.Join(", ", xNative));
		//Debug.Log("Loop Steps: " + string.Join(", ", numStepsPerLoop));
		//Debug.Log("Substeps Index Per Loop: " + string.Join(", ", substepsIndexPerLoop));
		//Debug.Log("Verts Per Substep: " + string.Join(", ", numVertsPerSubstepNative));

		DisposeTempRenderingNatives();

		return true;
	}

	void OnDestroy() {
		Release();
		DisposeNatives();
	}

	private void OnDisable() {
		Release();
		DisposeNatives();
	}

	private void OnEnable() {
		/*if (!buffersGenerated) {
			CreateBuffers();
		}*/
	}

	private void DisposeTempRenderingNatives() {
		if (textureSortedClothsNative.IsCreated) textureSortedClothsNative.Dispose();
		if (renderedToSortedNative.IsCreated) renderedToSortedNative.Dispose();
		if (sideToInstanceNative.IsCreated) sideToInstanceNative.Dispose();
		if (renderedTriangleLocalStartIndexNative.IsCreated) renderedTriangleLocalStartIndexNative.Dispose();
		if (renderedTriangleOffsetsNative.IsCreated) renderedTriangleOffsetsNative.Dispose();
	}

	private void DisposeTempSimulationNatives() {
		if (trianglesNative.IsCreated) trianglesNative.Dispose();
		if (uvNative.IsCreated) uvNative.Dispose();
		if (wNative.IsCreated) wNative.Dispose();
		if (vNative.IsCreated) vNative.Dispose();
		if (stretchingAlphaNative.IsCreated) stretchingAlphaNative.Dispose();
		if (bendingAlphaNative.IsCreated) bendingAlphaNative.Dispose();
		if (stepTimeNative.IsCreated) stepTimeNative.Dispose();
		if (dampingNative.IsCreated) dampingNative.Dispose();
		if (maxVelocityNative.IsCreated) maxVelocityNative.Dispose();
		if (handleCollisionsNative.IsCreated) handleCollisionsNative.Dispose();
		if (vertexToTrianglesNative.IsCreated) vertexToTrianglesNative.Dispose();
		if (triangleNormalsNative.IsCreated) triangleNormalsNative.Dispose();
		if (dragFactorNative.IsCreated) dragFactorNative.Dispose();
	}

	private void ReleaseSimulationBuffersAndNatives() {
		if (xNative.IsCreated) xNative.Dispose();
		if (normalsNative.IsCreated) normalsNative.Dispose();
		if (normalsInverseNative.IsCreated) normalsInverseNative.Dispose();
		if (activeVertexStartIndexNative.IsCreated) activeVertexStartIndexNative.Dispose();
		if (activeTriangleStartIndexNative.IsCreated) activeTriangleStartIndexNative.Dispose();
		if (activeBackTriangleStartIndexNative.IsCreated) activeBackTriangleStartIndexNative.Dispose();
		if (substepsNative.IsCreated) substepsNative.Dispose();
		if (applyGravityNative.IsCreated) applyGravityNative.Dispose();
		if (applyWindNative.IsCreated) applyWindNative.Dispose();
		if (numVertsPerSubstepNative.IsCreated) numVertsPerSubstepNative.Dispose();
		if (sortedClothIndicesNative.IsCreated) sortedClothIndicesNative.Dispose();
		if (stepVelocitiesNative.IsCreated) stepVelocitiesNative.Dispose();
		if (localGravityVectorsNative.IsCreated) localGravityVectorsNative.Dispose();
		if (localWindVectorsNative.IsCreated) localWindVectorsNative.Dispose();
		if (windForcesNative.IsCreated) windForcesNative.Dispose();

		ComputeHelper.Release(xBuffer, normalsBuffer, trianglesBuffer, uvBuffer, vBuffer, wBuffer, pBuffer, stepVelocityBuffer, gravityVectorBuffer, windVectorBuffer, startIndicesBuffer, substepsBuffer, stretchingAlphaBuffer, bendingAlphaBuffer, stepTimeBuffer, dampingBuffer, maxVelocityBuffer, vertexToTrianglesBuffer, triangleNormalsBuffer, dragFactorBuffer, windForceBuffer, pinnedVertBuffer);
		ComputeHelper.Release(stretchingIDsBuffers);
		ComputeHelper.Release(d0Buffers);
		ComputeHelper.Release(bendingIDsBuffers);
		ComputeHelper.Release(dihedral0Buffers);
	}

	private void ReleaseRenderingBuffers() {
		ComputeHelper.Release(renderedTriangleLocalStartIndexBuffers);
		ComputeHelper.Release(renderedTriangleOffsetsBuffers);
		ComputeHelper.Release(sideToInstanceBuffers);
		transformMatrixBuffer?.Release();

		ReleaseCommandBuffers();
	}

	void DisposeNatives() {
	}

	void Release() {
		ReleaseSimulationBuffersAndNatives();
		ReleaseRenderingBuffers();
		ReleaseCommandBuffers();
	}

	void ReleaseCommandBuffers() {
		if (commandBuffers != null) {
			for (int i = 0; i < commandBuffers.Length; i++) {
				if (commandBuffers[i] != null) {
					commandBuffers[i].Release();
				}
			}
		}
	}

	#region Cloth Sorting by Num Substeps
	void QuickSort(NativeArray<int> substeps, NativeArray<int> index, int low, int high)
	{
		if (low < high)
		{
			int pivotIndex = Partition(substeps, index, low, high);

			QuickSort(substeps, index, low, pivotIndex - 1);
			QuickSort(substeps, index, pivotIndex + 1, high);
		}
	}

	int Partition(NativeArray<int> substeps, NativeArray<int> index, int low, int high)
	{
		int pivot = substeps[high];
		int i = low - 1;

		for (int j = low; j < high; j++)
		{
			if (substeps[j] >= pivot) // Sort in descending order
			{
				i++;
				Swap(substeps, i, j);
				Swap(index, i, j);
			}
		}

		Swap(substeps, i + 1, high);
		Swap(index, i + 1, high);

		return i + 1;
	}

	void Swap<T>(NativeArray<T> array, int index1, int index2) where T : struct
	{
		T temp = array[index1];
		array[index1] = array[index2];
		array[index2] = temp;
	}
	#endregion
}