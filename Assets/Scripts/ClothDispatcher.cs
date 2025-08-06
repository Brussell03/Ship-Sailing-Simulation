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
using static ClothSimulation;
using UnityEditor.Search;

public class ClothDispatcher : MonoBehaviour
{
    public ComputeShader clothCompute;

	[Min(1)]
	public int projectionIterations = 3;
	[Range(0, 1)] public float windChaos = 0.1f;
	public bool isSimulating = true;
	public bool isRendering = true;
	public bool gravityActive = true;
	public bool windActive = true;
	public int maxLoadedVertices = 250000;
	public List<ClothMaterial> materials = new List<ClothMaterial>();
	public List<ClothTexture> textures = new List<ClothTexture>();
	private ClothSimulation[] cloths;
	private GameManager gameManager;

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
	const int solveConnectedKernel = 14;
	const int calculateTriangleNormalsKernel = 15;
	const int calculateVertexNormalsKernel = 16;
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
	ComputeBuffer[] renderedTriangleLocalStartIndexBuffers;
	ComputeBuffer[] renderedTriangleOffsetsBuffers;
	ComputeBuffer dragFactorBuffer;
	ComputeBuffer pinnedVertBuffer;
	ComputeBuffer vertToPinnedBuffer;
	ComputeBuffer clothWindForceBuffer;
	ComputeBuffer[] textureDoubleSidedBuffers;
	ComputeBuffer connectedVertsBuffer;
	ComputeBuffer textureUVBuffer;
	ComputeBuffer solveBendingBuffer;
	ComputeBuffer oneSidedWindBuffer;
	ComputeBuffer clothIndicesBuffer;
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
	NativeArray<bool> applyWindNative;
	NativeArray<int> numVertsPerSubstepNative;
	NativeArray<int> sortedClothIndicesNative;
	NativeArray<Vector3> normalsNative;
	NativeArray<int> trianglesNative;
	NativeArray<AdjacentTriangleIndices> vertexToTrianglesNative;
	NativeArray<Vector3> triangleNormalsNative;
	NativeArray<Vector2> uvNative;
	NativeArray<int> activeTriangleStartIndexNative;
	NativeArray<int> materialSortedClothsNative;
	NativeArray<int> textureSortedClothsNative;
	NativeArray<int> activeVertexStartIndexNative;
	NativeArray<int> renderedTriangleLocalStartIndexNative;
	NativeArray<int> renderedTriangleOffsetsNative;
	NativeArray<int> renderedToSortedNative;
	NativeArray<int> materialToSortedNative;
	NativeArray<int> activeBackTriangleStartIndexNative;
	NativeArray<Vector3> localGravityVectorsNative;
	NativeArray<Vector3> windVectorsNative;
	NativeArray<float> dragFactorNative;
	NativeArray<int3> clothWindForcesNative;
	NativeArray<Vector3> pinnedVertNative; // Pinned Vertices
	NativeArray<int> pinnedPerClothNativeBackup;
	NativeArray<Vector3> pinnedVertNativeBackup;
	NativeArray<int> vertToPinnedNative;
	NativeArray<Vector3> clothPositionDeltaNative;
	NativeArray<uint> textureDoubleSidedNative;
	NativeArray<int> connectedVertsNative;
	NativeArray<Vector2> textureUVNative;
	NativeArray<int> solveBendingNative;
	NativeArray<int> oneSidedWindNative;
	NativeArray<int> clothIndicesNative;
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
	List<int> materialGroupLastIndex = new List<int>();
	List<int> materialGroupFirstIndex = new List<int>();
	List<int> textureGroupLastIndex = new List<int>();
	List<int> textureGroupFirstIndex = new List<int>();
	List<int> numSidesPerGroup = new List<int>();
	List<Material> materialGroupMaterial = new List<Material>();
	List<int> textureGroupToMaterialIndex = new List<int>();

	[HideInInspector] public bool refreshAllQueued = false;
	[HideInInspector] public bool refreshRenderingQueued = false;
	[HideInInspector] public bool refreshPinnedQueued = false;
	[HideInInspector] public bool refreshConstraintLengthsQueued = false;
	[HideInInspector] public bool numClothsChanged = false;

	public bool buffersGenerated { get; private set; } = false;
	bool isInitialized = false;
	bool simulationGenerated = false;
	bool renderingGenerated = false;
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
	int numPinnedVertices = 0;
	int numConnectedVertices = 0;
	int fixedUpdateTicker = 0;
	bool readbackComplete = false;

	float dragCoeffPerp = 1.28f;
	float dragCoeffShear = 0.2f;

	MaterialPropertyBlock[] matProps;
	RenderParams rp;

	private AsyncGPUReadbackRequest xReadbackRequest;
	private AsyncGPUReadbackRequest vReadbackRequest;
	private AsyncGPUReadbackRequest normalsReadbackRequest;
	private AsyncGPUReadbackRequest windForceReadbackRequest;
	private bool pendingMeshReadback = false;
	private bool pendingForceReadback = false;
	private bool xReadbackDone = false;
	private bool vReadbackDone = false;
	private bool normalsReadbackDone = false;

	public float repeatIntervalSeconds = 1.0f; // How often to run the method
	private bool _isSolvingLODs = false;
	private CancellationTokenSource _cancellationTokenSource; // For graceful stopping

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
	private struct FillClothIndicesJob : IJobParallelFor {
		[ReadOnly] public int numCloths;
		[ReadOnly] public NativeArray<int> clothStartIndices;
		[WriteOnly]	public NativeArray<int> clothIndices;

		public void Execute(int index) {
			int clothIndex = 0;
			if (numCloths > 1) {
				for (int j = 1; j < numCloths; j++) {
					if (index >= clothStartIndices[j]) {
						clothIndex = j;
					} else {
						break;
					}
				}
			}

			clothIndices[index] = clothIndex;
		}
	}

	[BurstCompile]
	public struct TransformPointsJob : IJobParallelFor {
		public NativeArray<Vector3> positions;
		[ReadOnly] public NativeArray<int> clothStartIndices;
		[ReadOnly] public NativeArray<float4x4> transformMatrices;

		public void Execute(int index) {
			int clothIndex = 0;
			if (clothStartIndices.Length > 1) {
				for (int j = 1; j < clothStartIndices.Length; j++) {
					if (index >= clothStartIndices[j]) {
						clothIndex = j;
					} else {
						break;
					}
				}
			}

			positions[index] = math.mul(transformMatrices[clothIndex], new float4(positions[index], 1.0f)).xyz;
		}
	}

	// Start is called before the first frame update
	void Start()
    {
		//clothMaterial = new Material(clothMaterial);
		gameManager = FindAnyObjectByType<GameManager>();

		//rp = new RenderParams(clothMaterial);
		rp = new RenderParams();
		rp.worldBounds = new Bounds(Vector3.zero, 1000 * Vector3.one); // use tighter bounds
		refreshAllQueued = false;

		InitializeDispatcher();
	}

    // Update is called once per frame
    void FixedUpdate()
    {
		/*fixedUpdateTicker++;
		if (fixedUpdateTicker / (int)Time.timeScale < 1) return;
		fixedUpdateTicker = 0;*/

		if (!buffersGenerated) {
			return;
		}

		if (refreshPinnedQueued) {
			UpdatePinnedVertices();
		}

		if (refreshConstraintLengthsQueued) {
			UpdateConstraintLengths();
		}

		if (isSimulating && numSimulatedCloths != 0) {
			int numPinned = 0;
			for (int i = 0; i < numSimulatedCloths; i++) {
				if (refreshAllQueued && readbackComplete) {
					for (int j = 0; j < pinnedPerClothNativeBackup[i]; j++) {
						pinnedVertNative[numPinned] = cloths[sortedClothIndicesNative[i]].transform.TransformPoint(pinnedVertNativeBackup[numPinned++]);
					}
				} else {
					for (int j = 0; j < cloths[sortedClothIndicesNative[i]].pinnedVertLocalPos.Count; j++) {
						pinnedVertNative[numPinned++] = cloths[sortedClothIndicesNative[i]].transform.TransformPoint(cloths[sortedClothIndicesNative[i]].pinnedVertLocalPos[j]);
					}
				}
				
				// Wind Velocity
				if (windActive) {
					float time = Time.realtimeSinceStartup;
					float frequencyFactor = Mathf.PingPong(time, UnityEngine.Random.Range(-0.2f, 0.2f));
					float windSpeedVar = gameManager.windSpeed * (1f + Mathf.Cos(time * frequencyFactor / 8f) / 10f);
					float chaosMagnitude = Mathf.Sin(time * 1f * frequencyFactor) / 2f;
					float verticalFactor = Mathf.Sin(time * 1f * frequencyFactor + Mathf.PingPong(time, UnityEngine.Random.Range(5f, 20f))) / 4f;
					float horizontalFactor = Mathf.Sin(time * 1f * frequencyFactor + Mathf.PingPong(time, UnityEngine.Random.Range(5f, 20f)));
					windVectorsNative[i] = applyWindNative[i] ? (gameManager.windDirection * ((1f - windChaos / 4f) + chaosMagnitude * windChaos) + (transform.up * verticalFactor + transform.right * horizontalFactor) * chaosMagnitude * windChaos * 0.5f) * windSpeedVar : Vector3.zero;
					//windVectorsNative[i] = applyWindNative[i] ? (transform.forward * windSpeed) : Vector3.zero;

					/*if (cloths[sortedClothIndicesNative[i]].oneSidedWind) {
						float windForwardComp = Vector3.Dot(cloths[sortedClothIndicesNative[i]].transform.forward, windVectorsNative[i]);
						if (windForwardComp < 0) {
							// Wind force is against the forward direction of the cloth
							windVectorsNative[i] -= cloths[sortedClothIndicesNative[i]].transform.forward * windForwardComp;
						}
					}*/

				} else {
					windVectorsNative[i] = Vector3.zero;
				}
			}

			windVectorBuffer.SetData(windVectorsNative);

			if (numPinnedVertices > 0) {
				pinnedVertBuffer.SetData(pinnedVertNative);
			}


			/*if (handleCollisions) {
				hash.Create(x);
				//float maxTravelDist = maxVelocity * dt;
				float maxTravelDist = thickness;
				hash.QueryAll(x, maxTravelDist);
			}*/

			SimulationLoop();
		}

		if (!pendingForceReadback && !refreshAllQueued) {
			windForceReadbackRequest = AsyncGPUReadback.Request(clothWindForceBuffer);
			pendingForceReadback = true;
		}
	}

	void Update() {

		CheckRefresh();

		if (!buffersGenerated) return;

		if (pendingForceReadback) {
			if (windForceReadbackRequest.done) {
				if (windForceReadbackRequest.hasError) {
					//Debug.LogError("GPU Readback Error!");
				} else {
					clothWindForcesNative = windForceReadbackRequest.GetData<int3>();

					for (int i = 0; i < numSimulatedCloths; i++) {
						cloths[sortedClothIndicesNative[i]].windForce = new Vector3(clothWindForcesNative[i].x, clothWindForcesNative[i].y, clothWindForcesNative[i].z) / 10000.0f;
					}

					if (clothWindForcesNative.IsCreated) clothWindForcesNative.Dispose();
				}

				pendingForceReadback = false;
			}
		}

		// Render cloths
		if (isRendering && numRenderedCloths > 0) {
			for (int i = 0; i < commandBuffers.Length; i++) {
				Graphics.DrawProceduralIndirect(materialGroupMaterial[textureGroupToMaterialIndex[i]], new Bounds(Vector3.zero, 5000 * Vector3.one), MeshTopology.Triangles, commandBuffers[i], 0, null, matProps[i], ShadowCastingMode.On, true, gameObject.layer);
			}
		}

	}

	private void CheckRefresh() {
		if (refreshAllQueued) {
			if (buffersGenerated) {
				if (!readbackComplete) {
					if (!pendingMeshReadback) {
						// Initiate readback
						xReadbackRequest = AsyncGPUReadback.Request(xBuffer);
						vReadbackRequest = AsyncGPUReadback.Request(vBuffer);
						normalsReadbackRequest = AsyncGPUReadback.Request(normalsBuffer);
						pendingMeshReadback = true;

						for (int i = 0; i < numSimulatedCloths; i++) {
							cloths[sortedClothIndicesNative[i]].readbackComplete = false;
						}

					} else {
						// Copy X, V, and Normals back to cloths
						if (xReadbackRequest.done) {
							if (!xReadbackRequest.hasError) {

								xNative = xReadbackRequest.GetData<Vector3>();

								NativeArray<float4x4> transformMaticesNative = new NativeArray<float4x4>(numSimulatedCloths, Allocator.TempJob);

								for (int i = 0; i < numSimulatedCloths; i++) {

									int clothIndex = sortedClothIndicesNative[i];
									transformMaticesNative[i] = cloths[clothIndex].transform.worldToLocalMatrix;
								}

								TransformPointsJob transformJob = new TransformPointsJob {
									positions = xNative,
									clothStartIndices = activeVertexStartIndexNative,
									transformMatrices = transformMaticesNative
								};

								JobHandle jobHandle = transformJob.Schedule(numActiveVerts, 64);
								jobHandle.Complete();
								transformMaticesNative.Dispose();

								// Update positions for each cloth
								for (int i = 0; i < numSimulatedCloths; i++) {
									if (!cloths[sortedClothIndicesNative[i]].isActiveAndEnabled) continue;

									unsafe {
										// Copy Vertices
										int size = Marshal.SizeOf(typeof(Vector3)) * cloths[sortedClothIndicesNative[i]].numOneSidedVerts;
										IntPtr sourcePtr = (IntPtr)((Vector3*)xNative.GetUnsafePtr() + activeVertexStartIndexNative[i]);
										IntPtr destPtr = (IntPtr)((Vector3*)cloths[sortedClothIndicesNative[i]].x.GetUnsafePtr());
										UnsafeUtility.MemCpy((void*)destPtr, (void*)sourcePtr, size);
									}
								}

								xReadbackDone = true;
							}
						}
						if (vReadbackRequest.done) {
							if (!vReadbackRequest.hasError) {

								vNative = vReadbackRequest.GetData<Vector3>();

								// Update positions for each cloth
								for (int i = 0; i < numSimulatedCloths; i++) {
									if (!cloths[sortedClothIndicesNative[i]].isActiveAndEnabled) continue;

									unsafe {
										// Copy Velocities
										int size = Marshal.SizeOf(typeof(Vector3)) * cloths[sortedClothIndicesNative[i]].numOneSidedVerts;
										IntPtr sourcePtr = (IntPtr)((Vector3*)vNative.GetUnsafePtr() + activeVertexStartIndexNative[i]);
										IntPtr destPtr = (IntPtr)((Vector3*)cloths[sortedClothIndicesNative[i]].v.GetUnsafePtr());
										UnsafeUtility.MemCpy((void*)destPtr, (void*)sourcePtr, size);
									}
								}

								vReadbackDone = true;
							}
						}
						if (normalsReadbackRequest.done) {
							if (!normalsReadbackRequest.hasError) {

								normalsNative = normalsReadbackRequest.GetData<Vector3>();

								/*InvertNormalsJob invertJob = new InvertNormalsJob {
									normalsIn = normalsNative,
									normalsOut = normalsInverseNative
								};

								JobHandle jobHandle = invertJob.Schedule(numSimulatedVerts, 64);
								jobHandle.Complete();*/

								// Update positions for each cloth
								for (int i = 0; i < numSimulatedCloths; i++) {
									if (!cloths[sortedClothIndicesNative[i]].isActiveAndEnabled) continue;

									unsafe {
										// Copy Normals
										int size = Marshal.SizeOf(typeof(Vector3)) * cloths[sortedClothIndicesNative[i]].numOneSidedVerts;
										IntPtr sourcePtr = (IntPtr)((Vector3*)normalsNative.GetUnsafePtr() + activeVertexStartIndexNative[i]); // Calculate offset for the subset
										IntPtr destPtr = (IntPtr)((Vector3*)cloths[sortedClothIndicesNative[i]].normals.GetUnsafePtr());
										UnsafeUtility.MemCpy((void*)destPtr, (void*)sourcePtr, size);
									}
								}

								normalsReadbackDone = true;
							}
						}

						if (xReadbackDone && vReadbackDone && normalsReadbackDone) {

							pinnedVertNativeBackup = new NativeArray<Vector3>(pinnedVertNative.Length, Allocator.Persistent);
							pinnedPerClothNativeBackup = new NativeArray<int>(numSimulatedCloths, Allocator.Persistent);

							int numPinned = 0;
							for (int i = 0; i < numSimulatedCloths; i++) {
								pinnedPerClothNativeBackup[i] = cloths[sortedClothIndicesNative[i]].pinnedVertLocalPos.Count;
								for (int j = 0; j < cloths[sortedClothIndicesNative[i]].pinnedVertLocalPos.Count; j++) {
									pinnedVertNativeBackup[numPinned++] = cloths[sortedClothIndicesNative[i]].pinnedVertLocalPos[j];
								}
							}

							for (int i = 0; i < numSimulatedCloths; i++) {
								if (!cloths[sortedClothIndicesNative[i]].isActiveAndEnabled) continue;

								cloths[sortedClothIndicesNative[i]].readbackComplete = true;
							}

							readbackComplete = true;
							xReadbackDone = false;
							vReadbackDone = false;
							normalsReadbackDone = false;
							pendingMeshReadback = false;
						}
					}
				} else {

					bool lodUpdating = false;
					for (int i = 0; i < numSimulatedCloths; i++) {
						if (cloths[sortedClothIndicesNative[i]].isChangingLOD) {
							lodUpdating = true;
							break;
						}
					}

					if (!lodUpdating) {
						RefreshAll();

						readbackComplete = false;
						for (int i = 0; i < numSimulatedCloths; i++) {
							cloths[sortedClothIndicesNative[i]].readbackComplete = false;
						}

						pinnedVertNativeBackup.Dispose();
						pinnedPerClothNativeBackup.Dispose();
					}
				}

			} else {
				// Buffers not generated, no readback, perform first time generation

				bool lodUpdating = false;
				for (int i = 0; i < cloths.Length; i++) {
					if (cloths[i].isChangingLOD) {
						lodUpdating = true;
						break;
					}
				}

				if (!lodUpdating) {
					RefreshAll();
				}
			}
		}

		if (refreshRenderingQueued && !refreshAllQueued && buffersGenerated) {
			RefreshRendering();
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
				if (numD0PerSubstep[0][substepsIndexPerLoop[i]] > 0) ComputeHelper.Dispatch(clothCompute, numD0PerSubstep[0][substepsIndexPerLoop[i]], kernelIndex: solveStretchingKernel1);
				if (numD0PerSubstep[1][substepsIndexPerLoop[i]] > 0) ComputeHelper.Dispatch(clothCompute, numD0PerSubstep[1][substepsIndexPerLoop[i]], kernelIndex: solveStretchingKernel2);
				if (numD0PerSubstep[2][substepsIndexPerLoop[i]] > 0) ComputeHelper.Dispatch(clothCompute, numD0PerSubstep[2][substepsIndexPerLoop[i]], kernelIndex: solveStretchingKernel3);
				if (numD0PerSubstep[3][substepsIndexPerLoop[i]] > 0) ComputeHelper.Dispatch(clothCompute, numD0PerSubstep[3][substepsIndexPerLoop[i]], kernelIndex: solveStretchingKernel4);
				if (numD0PerSubstep[4][substepsIndexPerLoop[i]] > 0) ComputeHelper.Dispatch(clothCompute, numD0PerSubstep[4][substepsIndexPerLoop[i]], kernelIndex: solveStretchingKernel5);
				if (numD0PerSubstep[5][substepsIndexPerLoop[i]] > 0) ComputeHelper.Dispatch(clothCompute, numD0PerSubstep[5][substepsIndexPerLoop[i]], kernelIndex: solveStretchingKernel6);

				// Solve bending constraint for each batch of triangle pairs
				if (numDihedral0PerSubstep[0][substepsIndexPerLoop[i]] > 0) ComputeHelper.Dispatch(clothCompute, numDihedral0PerSubstep[0][substepsIndexPerLoop[i]], kernelIndex: solveBendingKernel1);
				if (numDihedral0PerSubstep[1][substepsIndexPerLoop[i]] > 0) ComputeHelper.Dispatch(clothCompute, numDihedral0PerSubstep[1][substepsIndexPerLoop[i]], kernelIndex: solveBendingKernel2);
				if (numDihedral0PerSubstep[2][substepsIndexPerLoop[i]] > 0) ComputeHelper.Dispatch(clothCompute, numDihedral0PerSubstep[2][substepsIndexPerLoop[i]], kernelIndex: solveBendingKernel3);
				if (numDihedral0PerSubstep[3][substepsIndexPerLoop[i]] > 0) ComputeHelper.Dispatch(clothCompute, numDihedral0PerSubstep[3][substepsIndexPerLoop[i]], kernelIndex: solveBendingKernel4);
				if (numDihedral0PerSubstep[4][substepsIndexPerLoop[i]] > 0) ComputeHelper.Dispatch(clothCompute, numDihedral0PerSubstep[4][substepsIndexPerLoop[i]], kernelIndex: solveBendingKernel5);
				if (numDihedral0PerSubstep[5][substepsIndexPerLoop[i]] > 0) ComputeHelper.Dispatch(clothCompute, numDihedral0PerSubstep[5][substepsIndexPerLoop[i]], kernelIndex: solveBendingKernel6);

				// Ensure connections
				if (numConnectedVertices > 0) ComputeHelper.Dispatch(clothCompute, numConnectedVertices, kernelIndex: solveConnectedKernel);

				// Update velocities
				ComputeHelper.Dispatch(clothCompute, numVertsPerSubstepNative[substepsIndexPerLoop[i]], kernelIndex: updateVelocityKernel);
			}
		}

		ComputeHelper.Dispatch(clothCompute, numOneSidedTrianglesTotal, kernelIndex: calculateTriangleNormalsKernel);
		ComputeHelper.Dispatch(clothCompute, numSimulatedVerts, kernelIndex: calculateVertexNormalsKernel);
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

		FindCloths();

		StartSolvingLODs();

		isInitialized = true;
	}

	async void RefreshAll() {
		buffersGenerated = false;
		simulationGenerated = false;
		renderingGenerated = false;
		pendingForceReadback = false;

		if (numClothsChanged || cloths == null) {
			FindCloths();

			numClothsChanged = false;
		}

		UpdateClothInfo();

		UpdateClothOrdering();

		Task simTask = UpdateSimulationBuffers();
		Task renderTask = UpdateRenderingBuffers();

		await simTask;
		await renderTask;

		if (!simulationGenerated) {
			Debug.Log("No simulated cloths");
			return;
		}

		if (!renderingGenerated) {
			Debug.Log("No rendered cloths");
			//return;
		}

		buffersGenerated = true;
		refreshAllQueued = false;
		refreshRenderingQueued = false;
		pendingMeshReadback = false;
	}

	async void RefreshRendering() {
		buffersGenerated = false;
		renderingGenerated = false;

		if (numClothsChanged || cloths == null) {
			FindCloths();
			RefreshAll();

			numClothsChanged = false;
			return;
		}

		UpdateClothInfo();

		await UpdateRenderingBuffers();

		if (!renderingGenerated) {
			Debug.Log("No rendered cloths");
			//return;
		}

		buffersGenerated = true;
		refreshRenderingQueued = false;
	}

	private void FindCloths() {
		// Finds all Cloths (Inactive included)
		cloths = Resources.FindObjectsOfTypeAll<ClothSimulation>();
	}

	private List<List<int>> GetConnectedIndices() {

		List<List<int>> connectedClothGroups = new List<List<int>>(capacity: cloths.Length);

		Dictionary<int, HashSet<int>> adjacencyMap = new Dictionary<int, HashSet<int>>();

		// Initialize map and populate direct connections
		for (int i = 0; i < cloths.Length; i++) {
			if (cloths[i].isInitialized && cloths[i].isActiveAndEnabled) {
				adjacencyMap[i] = new HashSet<int>();

				for (int j = 0; j < cloths[i].connectedVertices.Count; j++) {
					int connectedClothIndex = Array.IndexOf(cloths, cloths[i].connectedVertices[j].connectedCloth);
					adjacencyMap[i].Add(connectedClothIndex);

					// Add reverse connection from B to A (graph is undirected)
					// Ensure B is also in the map if it hasn't been processed yet
					if (!adjacencyMap.ContainsKey(connectedClothIndex)) {
						adjacencyMap[connectedClothIndex] = new HashSet<int>();
					}
					adjacencyMap[connectedClothIndex].Add(i);
				}
			}
		}

		// --- Step 2: Traverse the Graph (BFS) to Find Connected Components ---
		HashSet<int> visited = new HashSet<int>();

		for (int i = 0; i < cloths.Length; i++) {
			if (cloths[i].isInitialized && cloths[i].isActiveAndEnabled) {
				if (!visited.Contains(i)) { // If this cloth hasn't been visited yet, it starts a new group
					List<int> currentGroup = new List<int>();
					Queue<int> queue = new Queue<int>();

					queue.Enqueue(i);
					visited.Add(i);
					currentGroup.Add(i);

					while (queue.Count > 0) {
						int current = queue.Dequeue();

						// Get neighbors of current cloth (ensure it's in the map, otherwise it's isolated)
						if (adjacencyMap.TryGetValue(current, out HashSet<int> neighbors)) {
							foreach (int neighbor in neighbors) {
								if (!visited.Contains(neighbor)) {
									visited.Add(neighbor);
									queue.Enqueue(neighbor);
									currentGroup.Add(neighbor);
								}
							}
						}
					}
					connectedClothGroups.Add(currentGroup); // Add the complete group
				}
			}
		}

		return connectedClothGroups;
	}

	private void StartSolvingLODs() {
		if (_isSolvingLODs) return;
		_isSolvingLODs = true;
		_cancellationTokenSource = new CancellationTokenSource();

		RunLODSolveAsync(_cancellationTokenSource.Token);
	}

	private void StopSolvingLODs() {
		if (!_isSolvingLODs) return;
		_cancellationTokenSource?.Cancel(); // Signal the task to cancel
		_isSolvingLODs = false;
		_cancellationTokenSource?.Dispose(); // Dispose the source
		_cancellationTokenSource = null;
	}

	private async Task RunLODSolveAsync(CancellationToken cancellationToken) {
		while (!cancellationToken.IsCancellationRequested) {
			// --- Wait for the specified interval ---
			try {
				// Task.Delay runs on a background thread by default.
				// It's efficient because it doesn't block.
				await Task.Delay(TimeSpan.FromSeconds(repeatIntervalSeconds), cancellationToken);
			} catch (TaskCanceledException) {
				Debug.Log("[AsyncTimer] Periodic task cancelled.");
				break; // Exit loop if cancelled
			} catch (Exception ex) {
				Debug.LogError($"[AsyncTimer] An error occurred in periodic task: {ex.Message}");
				break; // Exit loop on unexpected errors
			}

			// --- Run your periodic action ---
			SolveLODs();
		}
		Debug.Log("[AsyncTimer] Periodic task stopped.");
	}

	private void SolveLODs() {

		if (refreshAllQueued) return;

		int totalVertices = 0;
		bool lodChanged = false;

		if (Time.timeScale > 1) {
			for (int i = 0; i < cloths.Length; i++) {
				if (cloths[i].isInitialized && cloths[i].isActiveAndEnabled) {
					totalVertices += cloths[i].LODs[0].numVertices;
					cloths[i].targetLODIndex = 0;

					if (0 != cloths[i].activeLODIndex) {
						lodChanged = true;

						cloths[i].isChangingLOD = true;
						if (!buffersGenerated) cloths[i].readbackComplete = true;

					} else {

						cloths[i].isChangingLOD = false;
						if (!buffersGenerated) cloths[i].readbackComplete = false;
					}
				}
			}

			if (lodChanged || !buffersGenerated) refreshAllQueued = true;

			return;
		}

		List<Tuple<float, int>> clothDistances = new List<Tuple<float, int>>();
		List<List<int>> connectedClothGroups = GetConnectedIndices();

		for (int i = 0; i < connectedClothGroups.Count; i++) {
			float sqrDistance = 0;

			for (int j = 0; j < connectedClothGroups[i].Count; j++) {
				sqrDistance += (cloths[connectedClothGroups[i][j]].transform.position - Camera.main.transform.position).sqrMagnitude;
			}

			sqrDistance /= connectedClothGroups[i].Count;

			clothDistances.Add(Tuple.Create(sqrDistance, i));

			float currentMax = 1000000f;
			int lodIndex = 0;
			for (int j = 0; j < cloths[connectedClothGroups[i][0]].LODs.Count; j++) {
				if (sqrDistance < cloths[connectedClothGroups[i][0]].LODs[j].maxDistanceSqr && cloths[connectedClothGroups[i][0]].LODs[j].maxDistance < currentMax) {
					currentMax = cloths[connectedClothGroups[i][0]].LODs[j].maxDistance;
					lodIndex = j;
				}
			}

			for (int j = 0; j < connectedClothGroups[i].Count; j++) {
				totalVertices += cloths[connectedClothGroups[i][j]].LODs[lodIndex].numVertices;

				if (lodIndex != cloths[connectedClothGroups[i][j]].activeLODIndex) {
					cloths[connectedClothGroups[i][j]].targetLODIndex = lodIndex;
					cloths[connectedClothGroups[i][j]].isChangingLOD = true;

					if (!buffersGenerated) cloths[connectedClothGroups[i][j]].readbackComplete = true;

					lodChanged = true;
				}
			}
		}

		if (totalVertices < maxLoadedVertices) {

			if (!lodChanged && buffersGenerated) return;

			refreshAllQueued = true;

		} else {

			List<Tuple<float, int>> distSortedCloths = clothDistances.OrderBy(tuple => tuple.Item1).ToList();

			totalVertices = 0;

			for (int i = 0; i < distSortedCloths.Count; i++) {
				for (int j = 0; j < connectedClothGroups[distSortedCloths[i].Item2].Count; j++) {
					totalVertices += cloths[connectedClothGroups[distSortedCloths[i].Item2][j]].LODs[0].numVertices;
					cloths[connectedClothGroups[distSortedCloths[i].Item2][j]].targetLODIndex = 0;
				}
			}

			int currentMaxDetailLevel = 0;
			int currentLimiter = 0;

			while (totalVertices <= maxLoadedVertices) {
				for (int i = currentLimiter; i < distSortedCloths.Count; i++) {

					var cloth = cloths[connectedClothGroups[distSortedCloths[i].Item2][0]];
					int nextLODIndex = cloth.targetLODIndex + 1;

					while (true) {
						// Increase LOD until caught up to closest's detail, at max distance, or at vertex limit

						if (nextLODIndex <= cloth.maxQualityLODIndex) {
							// Next LOD exists

							if (i == currentLimiter) {
								currentMaxDetailLevel = cloth.LODs[nextLODIndex].detailLevel;
							}

							if (distSortedCloths[i].Item1 < cloth.LODs[nextLODIndex].maxDistanceSqr) {
								// Next LOD max distance is greater than cloth distance

								int detailLevel = 0;

								for (int j = 0; j < connectedClothGroups[distSortedCloths[i].Item2].Count; j++) {
									if (cloths[connectedClothGroups[distSortedCloths[i].Item2][j]].LODs[nextLODIndex].detailLevel > detailLevel) {
										detailLevel = cloths[connectedClothGroups[distSortedCloths[i].Item2][j]].LODs[nextLODIndex].detailLevel;
									}
								}

								if (detailLevel <= currentMaxDetailLevel || i == currentLimiter) {
									// Next LOD detail level is under or equivalent to current allowed max

									int addedVerts = 0;

									for (int j = 0; j < connectedClothGroups[distSortedCloths[i].Item2].Count; j++) {
										addedVerts += cloths[connectedClothGroups[distSortedCloths[i].Item2][j]].LODs[nextLODIndex].numVertices - cloths[connectedClothGroups[distSortedCloths[i].Item2][j]].LODs[nextLODIndex - 1].numVertices;
									}

									if (totalVertices + addedVerts <= maxLoadedVertices) {
										// Vertices under maximum

										for (int j = 0; j < connectedClothGroups[distSortedCloths[i].Item2].Count; j++) {
											cloths[connectedClothGroups[distSortedCloths[i].Item2][j]].targetLODIndex = nextLODIndex;
										}
										totalVertices += addedVerts;

										if (i == currentLimiter) {
											break;
										}

										nextLODIndex++;
										continue;
									}
								}
							}
						}

						if (i == currentLimiter) {
							currentLimiter++;
						}

						break;
					}
				}

				if (currentLimiter >= distSortedCloths.Count) {
					break;
				}
			}
			
			lodChanged = false;

			for (int i = 0; i < distSortedCloths.Count; i++) {

				for (int j = 0; j < connectedClothGroups[distSortedCloths[i].Item2].Count; j++) {

					if (cloths[connectedClothGroups[distSortedCloths[i].Item2][j]].targetLODIndex != cloths[connectedClothGroups[distSortedCloths[i].Item2][j]].activeLODIndex) {
						lodChanged = true;

						cloths[connectedClothGroups[distSortedCloths[i].Item2][j]].isChangingLOD = true;
						if (!buffersGenerated) cloths[connectedClothGroups[distSortedCloths[i].Item2][j]].readbackComplete = true;

					} else {

						cloths[connectedClothGroups[distSortedCloths[i].Item2][j]].isChangingLOD = false;
						if (!buffersGenerated) cloths[connectedClothGroups[distSortedCloths[i].Item2][j]].readbackComplete = false;
					}

				}

				
			}

			if (lodChanged || !buffersGenerated) refreshAllQueued = true;
		}
	}

	private void UpdateClothInfo() {
		// Reset cloth information

		totalVerts = 0;
		numActiveCloths = 0;
		numSimulatedCloths = 0;
		numRenderedCloths = 0;

		simulatedClothIndices?.Clear();
		activeClothIndices?.Clear();

		numActiveVerts = 0;
		numSimulatedVerts = 0;
		numRenderedVerts = 0;
		numTrianglesTotal = 0;
		numOneSidedTrianglesTotal = 0;
		numPinnedVertices = 0;
		numConnectedVertices = 0;

		float totalArea = 0f;

		// Record cloth information

		for (int i = 0; i < cloths.Length; i++) {
			cloths[i].clothIndex = i;
		}

		for (int i = 0; i < cloths.Length; i++) {

			// Add the number of vertices representing a single side of the cloth
			totalVerts += cloths[i].numOneSidedVerts;

			if (cloths[i].isInitialized && cloths[i].isActiveAndEnabled) {
				activeClothIndices.Add(i);
				totalArea += cloths[i].mass / materials[(int)cloths[i].clothMaterial].surfaceDensity;

				// Add the number of vertices representing a single side of the cloth
				numActiveVerts += cloths[i].numOneSidedVerts;
				numOneSidedTrianglesTotal += cloths[i].numOneSidedTriangles;
				numTrianglesTotal += cloths[i].triangles.Length / 3;

				if (cloths[i].isSimulating) {
					simulatedClothIndices.Add(i);
					numSimulatedVerts += cloths[i].numOneSidedVerts;
					numSimulatedTriangles += cloths[i].numOneSidedTriangles;
					numPinnedVertices += cloths[i].pinnedVertLocalPos.Count;

					for (int j = 0; j < cloths[i].connectedVertices.Count; j++) {
						if (cloths[i].connectedVertices[j].connectedCloth == null) continue;

						ClothSimulation otherCloth = cloths[i].connectedVertices[j].connectedCloth;
						if (otherCloth.isSimulating && cloths[i].clothIndex < otherCloth.clothIndex && cloths[i].connectedVertices[j].isSet && otherCloth.connectedVertices[cloths[i].connectedVertices[j].targetListIndex].isSet) {
							numConnectedVertices += 2;
						}
					}
				}

				if (cloths[i].isRendering) {
					numRenderedCloths++;
					numRenderedVerts += cloths[i].numOneSidedVerts;
				}

				
			}
		}

		Debug.Log(totalArea);
		Debug.Log("Total Vertices: " + totalVerts);

		numActiveCloths = activeClothIndices.Count;
		numSimulatedCloths = simulatedClothIndices.Count;
	}

	private bool UpdateClothOrdering() {

		DisposeOrderingNatives();

		if (numActiveCloths == 0) {
			return false; // Return if no cloths are active/initialized/visible
		}

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
		activeVertexStartIndexNative = new NativeArray<int>(numActiveCloths, Allocator.Persistent);
		activeTriangleStartIndexNative = new NativeArray<int>(numActiveCloths, Allocator.Persistent);
		activeBackTriangleStartIndexNative = new NativeArray<int>(numActiveCloths, Allocator.Persistent);
		sortedClothIndicesNative = new NativeArray<int>(numActiveCloths, Allocator.Persistent);
		substepsNative = new NativeArray<int>(numActiveCloths, Allocator.Persistent);

		// Sort simulated cloths by number of substeps

		int addedCloths = 0;
		for (int i = 0; i < numSimulatedCloths; i++) {
			sortedClothIndicesNative[addedCloths] = simulatedClothIndices[i];
			substepsNative[addedCloths++] = cloths[simulatedClothIndices[i]].LODs[cloths[simulatedClothIndices[i]].activeLODIndex].substeps;
		}
		for (int i = 0; i < numActiveCloths; i++) {
			if (!cloths[activeClothIndices[i]].isSimulating) {
				sortedClothIndicesNative[addedCloths] = activeClothIndices[i];
				substepsNative[addedCloths++] = 0;
			}
		}

		QuickSort(substepsNative, sortedClothIndicesNative, 0, substepsNative.Length - 1);

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
		}

		return true;
	}

	async Task UpdateSimulationBuffers() {
		// New dispose function for just simulating natives & buffers

		ReleaseSimulationBuffersAndNatives();
		simulationGenerated = false;

		if (numActiveCloths == 0) {
			return; // Return if no cloths are active/initialized/visible
		}

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
		xNative = new NativeArray<Vector3>(numActiveVerts, Allocator.TempJob);
		normalsNative = new NativeArray<Vector3>(numActiveVerts, Allocator.Temp);
		trianglesNative = new NativeArray<int>(numTrianglesTotal * 3, Allocator.Temp);
		uvNative = new NativeArray<Vector2>(numActiveVerts, Allocator.Temp);
		wNative = new NativeArray<float>(numSimulatedVerts, Allocator.Temp);
		vNative = new NativeArray<Vector3>(numSimulatedVerts, Allocator.Temp);
		stretchingAlphaNative = new NativeArray<float>(numSimulatedCloths, Allocator.Temp);
		bendingAlphaNative = new NativeArray<float>(numSimulatedCloths, Allocator.Temp);
		stepTimeNative = new NativeArray<float>(numSimulatedCloths, Allocator.Temp);
		dampingNative = new NativeArray<float>(numSimulatedCloths, Allocator.Temp);
		maxVelocityNative = new NativeArray<float>(numSimulatedCloths, Allocator.Temp);
		vertexToTrianglesNative = new NativeArray<AdjacentTriangleIndices>(numSimulatedVerts, Allocator.Temp);
		triangleNormalsNative = new NativeArray<Vector3>(numSimulatedTriangles, Allocator.Temp);
		dragFactorNative = new NativeArray<float>(numSimulatedVerts, Allocator.Temp);
		localGravityVectorsNative = new NativeArray<Vector3>(numSimulatedCloths, Allocator.Temp);
		vertToPinnedNative = new NativeArray<int>(numSimulatedVerts, Allocator.Temp);
		clothPositionDeltaNative = new NativeArray<Vector3>(numActiveCloths, Allocator.TempJob);
		connectedVertsNative = new NativeArray<int>(numConnectedVertices, Allocator.Temp);
		textureUVNative = new NativeArray<Vector2>(numActiveVerts, Allocator.Temp);
		solveBendingNative = new NativeArray<int>(numSimulatedCloths, Allocator.Temp);
		oneSidedWindNative = new NativeArray<int>(numSimulatedCloths, Allocator.Temp);
		clothIndicesNative = new NativeArray<int>(numSimulatedVerts, Allocator.TempJob);

		pinnedVertNative = new NativeArray<Vector3>(numPinnedVertices, Allocator.Persistent);
		windVectorsNative = new NativeArray<Vector3>(numSimulatedCloths, Allocator.Persistent);
		applyWindNative = new NativeArray<bool>(numSimulatedCloths, Allocator.Persistent);
		numVertsPerSubstepNative = new NativeArray<int>(numSimulatedCloths, Allocator.Persistent);

		for (int i = 0; i < numActiveCloths; i++) {
			cloths[sortedClothIndicesNative[i]].clothIndex = i;
		}

		int numPinned = 0;
		int numConnected = 0;

		for (int i = 0; i < numActiveCloths; i++) {

			int clothIndex = sortedClothIndicesNative[i];

			// Copy vertices, triangle indices, normals, and uv's from individual cloths to combined array
			unsafe {
				// Copy Vertices
				int size = Marshal.SizeOf(typeof(Vector3)) * cloths[clothIndex].numOneSidedVerts;
				IntPtr sourcePtr = (IntPtr)((Vector3*)cloths[clothIndex].x.GetUnsafePtr());
				IntPtr destPtr = (IntPtr)((Vector3*)xNative.GetUnsafePtr() + activeVertexStartIndexNative[i]); // Calculate offset for the subset
				UnsafeUtility.MemCpy((void*)destPtr, (void*)sourcePtr, size);

				// Copy Frontside Normals
				size = Marshal.SizeOf(typeof(Vector3)) * cloths[clothIndex].numOneSidedVerts;
				sourcePtr = (IntPtr)((Vector3*)cloths[clothIndex].normals.GetUnsafePtr());
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
				sourcePtr = (IntPtr)((Vector2*)cloths[clothIndex].uv.GetUnsafePtr());
				destPtr = (IntPtr)((Vector2*)uvNative.GetUnsafePtr() + activeVertexStartIndexNative[i]); // Calculate offset for the subset
				UnsafeUtility.MemCpy((void*)destPtr, (void*)sourcePtr, size);

				// Copy Texture UVs
				sourcePtr = (IntPtr)((Vector2*)cloths[clothIndex].textureUV.GetUnsafePtr());
				destPtr = (IntPtr)((Vector2*)textureUVNative.GetUnsafePtr() + activeVertexStartIndexNative[i]); // Calculate offset for the subset
				UnsafeUtility.MemCpy((void*)destPtr, (void*)sourcePtr, size);
			}

			// If cloth is being simulated
			if (i < numSimulatedCloths) {
				float dt_step = Time.fixedDeltaTime / substepsNative[i];
				stretchingAlphaNative[i] = cloths[clothIndex].stretchingCompliance / (dt_step * dt_step);
				bendingAlphaNative[i] = cloths[clothIndex].bendingCompliance / (dt_step * dt_step);
				stepTimeNative[i] = dt_step;
				dampingNative[i] = cloths[clothIndex].damping;
				//maxVelocityNative[i] = 1;// * substepsNative[i];
				maxVelocityNative[i] = 1 / stepTimeNative[i];
				applyWindNative[i] = cloths[clothIndex].simulateWind;
				solveBendingNative[i] = cloths[clothIndex].solveBending ? 1 : 0;
				oneSidedWindNative[i] = cloths[clothIndex].oneSidedWind ? 1 : 0;

				// Acceleration Due to Gravity, Acceleration
				localGravityVectorsNative[i] = (gravityActive && cloths[clothIndex].simulateGravity) ? Physics.gravity : Vector3.zero;

				// Map pinned vertices locations
				for (int j = 0; j < cloths[clothIndex].pinnedVertLocalPos.Count; j++) {
					vertToPinnedNative[cloths[clothIndex].LODs[cloths[clothIndex].activeLODIndex].pinIndices[j] + activeVertexStartIndexNative[i]] = numPinned++;
				}

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

				// Copy from individual cloths to combined array
				unsafe {
					// Copy Inverse Masses
					int size = Marshal.SizeOf(typeof(float)) * cloths[clothIndex].numOneSidedVerts;
					IntPtr sourcePtr = Marshal.UnsafeAddrOfPinnedArrayElement(cloths[clothIndex].w, 0);
					IntPtr destPtr = (IntPtr)((float*)wNative.GetUnsafePtr() + activeVertexStartIndexNative[i]); // Calculate offset for the subset
					UnsafeUtility.MemCpy((void*)destPtr, (void*)sourcePtr, size);

					// Copy Velocities
					size = Marshal.SizeOf(typeof(Vector3)) * cloths[clothIndex].numOneSidedVerts;
					sourcePtr = (IntPtr)((Vector3*)cloths[clothIndex].v.GetUnsafePtr());
					destPtr = (IntPtr)((Vector3*)vNative.GetUnsafePtr() + activeVertexStartIndexNative[i]); // Calculate offset for the subset
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

				// Connected vertices
				for (int j = 0; j < cloths[clothIndex].connectedVertices.Count; j++) {
					if (cloths[clothIndex].connectedVertices[j].connectedCloth == null) continue;

					ClothSimulation otherCloth = cloths[clothIndex].connectedVertices[j].connectedCloth;
					if (cloths[clothIndex].clothIndex < otherCloth.clothIndex && cloths[clothIndex].connectedVertices[j].isSet && otherCloth.connectedVertices[cloths[clothIndex].connectedVertices[j].targetListIndex].isSet) {
						
						if (cloths[clothIndex].activeLODIndex == cloths[clothIndex].maxQualityLODIndex) {
							// Using max quality LOD, indices do not need to be adjusted

							connectedVertsNative[numConnected++] = cloths[clothIndex].connectedVertices[j].vertIndex + activeVertexStartIndexNative[i];
							connectedVertsNative[numConnected++] = otherCloth.connectedVertices[cloths[clothIndex].connectedVertices[j].targetListIndex].vertIndex + activeVertexStartIndexNative[otherCloth.clothIndex];
						} else {
							// Active LOD is lower than max, indices need to be adjusted

							int rowIndex1 = 0;
							int colIndex1 = 0;
							int rowIndex2 = 0;
							int colIndex2 = 0;

							if (cloths[clothIndex].clothShape == ClothShape.Trapezoidal) {
								int passed = 0;
								for (int k = 0; k <= cloths[clothIndex].maxNumRows; k++) {
									int colToRowDiff = cloths[clothIndex].maxNumColumns - cloths[clothIndex].maxNumRows;
									int numOnRow = (colToRowDiff > 0 ? colToRowDiff : 0) + k + 1;
									numOnRow = numOnRow > cloths[clothIndex].maxNumColumns + 1 ? cloths[clothIndex].maxNumColumns + 1 : numOnRow;

									if (cloths[clothIndex].connectedVertices[j].vertIndex > numOnRow - 1 + passed) {
										passed += numOnRow;
									} else {
										rowIndex1 = k;
										colIndex1 = cloths[clothIndex].maxNumColumns - numOnRow + 1 + (cloths[clothIndex].connectedVertices[j].vertIndex - passed) % numOnRow;
										break;
									}
								}
							} else {
								rowIndex1 = cloths[clothIndex].connectedVertices[j].vertIndex / (cloths[clothIndex].maxNumColumns + 1);
								colIndex1 = cloths[clothIndex].connectedVertices[j].vertIndex % (cloths[clothIndex].maxNumColumns + 1);
							}

							if (otherCloth.clothShape == ClothShape.Trapezoidal) {
								int passed = 0;
								for (int k = 0; k <= otherCloth.maxNumRows; k++) {
									int colToRowDiff = otherCloth.maxNumColumns - otherCloth.maxNumRows;
									int numOnRow = (colToRowDiff > 0 ? colToRowDiff : 0) + k + 1;
									numOnRow = numOnRow > otherCloth.maxNumColumns + 1 ? otherCloth.maxNumColumns + 1 : numOnRow;

									if (otherCloth.connectedVertices[cloths[clothIndex].connectedVertices[j].targetListIndex].vertIndex > numOnRow - 1 + passed) {
										passed += numOnRow;
									} else {
										rowIndex2 = k;
										colIndex2 = otherCloth.maxNumColumns - numOnRow + 1 + (otherCloth.connectedVertices[cloths[clothIndex].connectedVertices[j].targetListIndex].vertIndex - passed) % numOnRow;
										break;
									}
								}
							} else {
								rowIndex2 = otherCloth.connectedVertices[cloths[clothIndex].connectedVertices[j].targetListIndex].vertIndex / (otherCloth.maxNumColumns + 1);
								colIndex2 = otherCloth.connectedVertices[cloths[clothIndex].connectedVertices[j].targetListIndex].vertIndex % (otherCloth.maxNumColumns + 1);
							}

							int every = (int)Mathf.Pow(2, cloths[clothIndex].maxSubdivisions - cloths[clothIndex].LODs[cloths[clothIndex].activeLODIndex].subdivisions);

							if (colIndex1 % every == 0 && rowIndex1 % every == 0) {
								// Index is on this quality level
								int adjustedGridIndex1 = rowIndex1 / every * (cloths[clothIndex].LODs[cloths[clothIndex].activeLODIndex].numColumns + 1) + colIndex1 / every;
								int adjustedGridIndex2 = rowIndex2 / every * (otherCloth.LODs[cloths[clothIndex].activeLODIndex].numColumns + 1) + colIndex2 / every;

								if (cloths[clothIndex].clothShape == ClothShape.Trapezoidal) {
									int minSubdivision = (int)Mathf.Min(cloths[clothIndex].LODs[cloths[clothIndex].activeLODIndex].numRows, cloths[clothIndex].LODs[cloths[clothIndex].activeLODIndex].numColumns);

									int minSubdivisionMinAdjustedRowIndex = minSubdivision - rowIndex1 / every;
									adjustedGridIndex1 -= minSubdivision * (minSubdivision + 1) / 2;

									if (minSubdivisionMinAdjustedRowIndex - 1 > 0) {
										adjustedGridIndex1 += (minSubdivisionMinAdjustedRowIndex - 1) * (minSubdivisionMinAdjustedRowIndex) / 2;
									}
								}

								if (otherCloth.clothShape == ClothShape.Trapezoidal) {
									int minSubdivision = (int)Mathf.Min(otherCloth.LODs[cloths[clothIndex].activeLODIndex].numRows, otherCloth.LODs[cloths[clothIndex].activeLODIndex].numColumns);

									int minSubdivisionMinAdjustedRowIndex = minSubdivision - rowIndex2 / every;
									adjustedGridIndex2 -= minSubdivision * (minSubdivision + 1) / 2;

									if (minSubdivisionMinAdjustedRowIndex - 1 > 0) {
										adjustedGridIndex2 += (minSubdivisionMinAdjustedRowIndex - 1) * (minSubdivisionMinAdjustedRowIndex) / 2;
									}
								}

								connectedVertsNative[numConnected++] = adjustedGridIndex1 + activeVertexStartIndexNative[i];
								connectedVertsNative[numConnected++] = adjustedGridIndex2 + activeVertexStartIndexNative[otherCloth.clothIndex];
							}
						}
					}
				}
			}
		}

		// Set Mesh Data Buffers

		normalsBuffer = ComputeHelper.CreateStructuredBuffer<Vector3>(normalsNative.Length);
		normalsBuffer.SetData(normalsNative);
		clothCompute.SetBuffer(predictPositionKernel, "normals", normalsBuffer);
		clothCompute.SetBuffer(calculateVertexNormalsKernel, "normals", normalsBuffer);

		trianglesBuffer = ComputeHelper.CreateStructuredBuffer<int>(numTrianglesTotal * 3);
		trianglesBuffer.SetData(trianglesNative);
		clothCompute.SetBuffer(calculateTriangleNormalsKernel, "triangles", trianglesBuffer);

		uvBuffer = ComputeHelper.CreateStructuredBuffer<Vector2>(numActiveVerts);
		uvBuffer.SetData(uvNative);

		textureUVBuffer = ComputeHelper.CreateStructuredBuffer<Vector2>(numActiveVerts);
		textureUVBuffer.SetData(textureUVNative);

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
		clothCompute.SetBuffer(solveConnectedKernel, "substeps", substepsBuffer);

		clothCompute.SetInt("numCloths", numActiveCloths);
		clothCompute.SetFloat("projectionIterations", projectionIterations);

		// Set Simulation Buffers

		if (numSimulatedCloths == 0) {
			simulationGenerated = true;
			return;
		}

		vBuffer = ComputeHelper.CreateStructuredBuffer<Vector3>(numSimulatedVerts);
		vBuffer.SetData(vNative);
		clothCompute.SetBuffer(updateVelocityKernel, "v", vBuffer);
		clothCompute.SetBuffer(predictPositionKernel, "v", vBuffer);

		vertexToTrianglesBuffer = ComputeHelper.CreateStructuredBuffer<AdjacentTriangleIndices>(numSimulatedVerts);
		vertexToTrianglesBuffer.SetData(vertexToTrianglesNative);
		clothCompute.SetBuffer(calculateVertexNormalsKernel, "vertexToTriangle", vertexToTrianglesBuffer);

		triangleNormalsBuffer = ComputeHelper.CreateStructuredBuffer<Vector3>(numSimulatedTriangles);
		triangleNormalsBuffer.SetData(triangleNormalsNative);
		clothCompute.SetBuffer(calculateVertexNormalsKernel, "triangleNormals", triangleNormalsBuffer);
		clothCompute.SetBuffer(calculateTriangleNormalsKernel, "triangleNormals", triangleNormalsBuffer);

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
		clothCompute.SetBuffer(solveConnectedKernel, "p", pBuffer);

		dragFactorBuffer = ComputeHelper.CreateStructuredBuffer<float>(numSimulatedVerts);
		dragFactorBuffer.SetData(dragFactorNative);
		clothCompute.SetBuffer(predictPositionKernel, "dragFactor", dragFactorBuffer);

		clothWindForceBuffer = ComputeHelper.CreateStructuredBuffer<int3>(numSimulatedCloths);
		clothCompute.SetBuffer(predictPositionKernel, "clothWindForce", clothWindForceBuffer);

		if (numPinnedVertices > 0) {
			pinnedVertBuffer = ComputeHelper.CreateStructuredBuffer<Vector3>(numPinnedVertices);
			clothCompute.SetBuffer(predictPositionKernel, "pinnedVertPos", pinnedVertBuffer);
		}

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
		clothCompute.SetBuffer(updateVelocityKernel, "stepVelocityAndPrevX", stepVelocityBuffer);
		clothCompute.SetBuffer(predictPositionKernel, "stepVelocityAndPrevX", stepVelocityBuffer);

		gravityVectorBuffer = ComputeHelper.CreateStructuredBuffer<Vector3>(numSimulatedCloths);
		gravityVectorBuffer.SetData(localGravityVectorsNative);
		clothCompute.SetBuffer(predictPositionKernel, "gravityVector", gravityVectorBuffer);

		windVectorBuffer = ComputeHelper.CreateStructuredBuffer<Vector3>(numSimulatedCloths);
		clothCompute.SetBuffer(predictPositionKernel, "windVector", windVectorBuffer);

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

		vertToPinnedBuffer = ComputeHelper.CreateStructuredBuffer<int>(numSimulatedVerts);
		vertToPinnedBuffer.SetData(vertToPinnedNative);
		clothCompute.SetBuffer(predictPositionKernel, "vertToPinned", vertToPinnedBuffer);

		clothCompute.SetInt("numTriangles", numSimulatedTriangles);
		clothCompute.SetInt("numVertices", numSimulatedVerts);
		UpdateDragCoeff();

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

		if (numConnectedVertices > 0) {
			connectedVertsBuffer = ComputeHelper.CreateStructuredBuffer<int>(numConnectedVertices);
			connectedVertsBuffer.SetData(connectedVertsNative);
			clothCompute.SetBuffer(solveConnectedKernel, "connectedIDs", connectedVertsBuffer);
		}

		solveBendingBuffer = ComputeHelper.CreateStructuredBuffer<int>(numSimulatedCloths);
		solveBendingBuffer.SetData(solveBendingNative);
		clothCompute.SetBuffer(solveBendingKernel1, "solveBending", solveBendingBuffer);
		clothCompute.SetBuffer(solveBendingKernel2, "solveBending", solveBendingBuffer);
		clothCompute.SetBuffer(solveBendingKernel3, "solveBending", solveBendingBuffer);
		clothCompute.SetBuffer(solveBendingKernel4, "solveBending", solveBendingBuffer);
		clothCompute.SetBuffer(solveBendingKernel5, "solveBending", solveBendingBuffer);
		clothCompute.SetBuffer(solveBendingKernel6, "solveBending", solveBendingBuffer);

		oneSidedWindBuffer = ComputeHelper.CreateStructuredBuffer<int>(numSimulatedCloths);
		oneSidedWindBuffer.SetData(oneSidedWindNative);
		clothCompute.SetBuffer(predictPositionKernel, "oneSidedWind", oneSidedWindBuffer);

		NativeArray<float4x4> transformMaticesNative = new NativeArray<float4x4>(numActiveCloths, Allocator.TempJob);

		for (int i = 0; i < numActiveCloths; i++) {
			int clothIndex = sortedClothIndicesNative[i];
			transformMaticesNative[i] = cloths[clothIndex].transform.localToWorldMatrix;
		}

		TransformPointsJob transformJob = new TransformPointsJob {
			positions = xNative,
			clothStartIndices = activeVertexStartIndexNative,
			transformMatrices = transformMaticesNative
		};

		JobHandle transformJobHandle = transformJob.Schedule(numActiveVerts, 64);

		FillClothIndicesJob clothIndicesJob = new FillClothIndicesJob {
			numCloths = numSimulatedCloths,
			clothStartIndices = activeVertexStartIndexNative,
			clothIndices = clothIndicesNative
		};

		JobHandle clothIndicesJobHandle = clothIndicesJob.Schedule(numSimulatedVerts, 64);

		transformJobHandle.Complete();
		clothIndicesJobHandle.Complete();

		transformMaticesNative.Dispose();

		xBuffer = ComputeHelper.CreateStructuredBuffer<Vector3>(numActiveVerts);
		xBuffer.SetData(xNative);
		clothCompute.SetBuffer(calculateTriangleNormalsKernel, "x", xBuffer);
		clothCompute.SetBuffer(updateVelocityKernel, "x", xBuffer);
		clothCompute.SetBuffer(predictPositionKernel, "x", xBuffer);

		clothIndicesBuffer = ComputeHelper.CreateStructuredBuffer<int>(numSimulatedVerts);
		clothIndicesBuffer.SetData(clothIndicesNative);
		clothCompute.SetBuffer(updateVelocityKernel, "clothIDs", clothIndicesBuffer);
		clothCompute.SetBuffer(predictPositionKernel, "clothIDs", clothIndicesBuffer);
		clothCompute.SetBuffer(solveStretchingKernel1, "clothIDs", clothIndicesBuffer);
		clothCompute.SetBuffer(solveStretchingKernel2, "clothIDs", clothIndicesBuffer);
		clothCompute.SetBuffer(solveStretchingKernel3, "clothIDs", clothIndicesBuffer);
		clothCompute.SetBuffer(solveStretchingKernel4, "clothIDs", clothIndicesBuffer);
		clothCompute.SetBuffer(solveStretchingKernel5, "clothIDs", clothIndicesBuffer);
		clothCompute.SetBuffer(solveStretchingKernel6, "clothIDs", clothIndicesBuffer);
		clothCompute.SetBuffer(solveBendingKernel1, "clothIDs", clothIndicesBuffer);
		clothCompute.SetBuffer(solveBendingKernel2, "clothIDs", clothIndicesBuffer);
		clothCompute.SetBuffer(solveBendingKernel3, "clothIDs", clothIndicesBuffer);
		clothCompute.SetBuffer(solveBendingKernel4, "clothIDs", clothIndicesBuffer);
		clothCompute.SetBuffer(solveBendingKernel5, "clothIDs", clothIndicesBuffer);
		clothCompute.SetBuffer(solveBendingKernel6, "clothIDs", clothIndicesBuffer);
		clothCompute.SetBuffer(solveConnectedKernel, "clothIDs", clothIndicesBuffer);

		DisposeTempSimulationNatives();

		simulationGenerated = true;
		return; // Successful creation of buffers
	}

	async Task UpdateRenderingBuffers() {

		materialGroupFirstIndex?.Clear();
		materialGroupLastIndex?.Clear();
		textureGroupFirstIndex?.Clear();
		textureGroupLastIndex?.Clear();
		numSidesPerGroup?.Clear();
		materialGroupMaterial?.Clear();
		textureGroupToMaterialIndex?.Clear();
		renderingGenerated = false;

		ReleaseRenderingBuffers();

		if (numRenderedCloths == 0) {
			renderingGenerated = false;
			return; // Return if no cloths are active/initialized/visible
		}

		materialSortedClothsNative = new NativeArray<int>(numRenderedCloths, Allocator.Temp);
		textureSortedClothsNative = new NativeArray<int>(numRenderedCloths, Allocator.Temp);
		materialToSortedNative = new NativeArray<int>(numRenderedCloths, Allocator.Temp);
		renderedToSortedNative = new NativeArray<int>(numRenderedCloths, Allocator.Temp);

		int numAddedCloths = 0;
		int numMaterialGroups = 0;

		unsafe {
			void* ptr = materialSortedClothsNative.GetUnsafePtr();
			// Fill all bytes of the array with 0xFF.
			// For a signed 32-bit integer, 0xFFFFFFFF (all bits set) represents -1.
			UnsafeUtility.MemSet(ptr, 0xFF, numRenderedCloths * UnsafeUtility.SizeOf<int>());

			ptr = textureSortedClothsNative.GetUnsafePtr();
			UnsafeUtility.MemSet(ptr, 0xFF, numRenderedCloths * UnsafeUtility.SizeOf<int>());
		}

		for (int i = 0; i < numActiveCloths; i++) {
			int clothIndex = sortedClothIndicesNative[i];

			if (cloths[clothIndex].isRendering && !materialSortedClothsNative.Contains(clothIndex)) {
				materialGroupFirstIndex.Add(numAddedCloths);

				int currentMaterial = (int)cloths[clothIndex].clothMaterial;
				materialToSortedNative[numAddedCloths] = i;
				materialSortedClothsNative[numAddedCloths++] = clothIndex;

				for (int j = i + 1; j < numActiveCloths; j++) {

					if (cloths[sortedClothIndicesNative[j]].isRendering && (int)cloths[sortedClothIndicesNative[j]].clothMaterial == currentMaterial) {

						materialToSortedNative[numAddedCloths] = j;
						materialSortedClothsNative[numAddedCloths++] = sortedClothIndicesNative[j];

					}
				}

				numMaterialGroups++;
				materialGroupLastIndex.Add(numAddedCloths - 1);
			}
		}

		numAddedCloths = 0;
		int numTextureGroups = 0;
		for (int i = 0; i < numMaterialGroups; i++) {
			for (int j = materialGroupFirstIndex[i]; j < materialGroupLastIndex[i] + 1; j++) {

				int clothIndex = materialSortedClothsNative[j];

				if (!textureSortedClothsNative.Contains(clothIndex)) {
					textureGroupFirstIndex.Add(numAddedCloths);
					textureGroupToMaterialIndex.Add(i);

					int currentTexture = (int)cloths[clothIndex].clothTexture;
					renderedToSortedNative[numAddedCloths] = materialToSortedNative[j];
					textureSortedClothsNative[numAddedCloths++] = clothIndex;
					numSidesPerGroup.Add(cloths[clothIndex].isDoubleSided ? 2 : 1);

					for (int k = j + 1; k < materialGroupLastIndex[i] + 1; k++) {

						if ((int)cloths[materialSortedClothsNative[k]].clothTexture == currentTexture) {

							renderedToSortedNative[numAddedCloths] = materialToSortedNative[k];
							textureSortedClothsNative[numAddedCloths++] = materialSortedClothsNative[k];
							numSidesPerGroup[numTextureGroups] += cloths[materialSortedClothsNative[k]].isDoubleSided ? 2 : 1;

						}
					}

					numTextureGroups++;
					textureGroupLastIndex.Add(numAddedCloths - 1);
				}
			}

			int matIndex = (int)cloths[textureSortedClothsNative[materialGroupFirstIndex[i]]].clothMaterial;
			materialGroupMaterial.Add(materials[matIndex].material);
			materialGroupMaterial[i].SetBuffer("Vertices", xBuffer);
			materialGroupMaterial[i].SetBuffer("Triangles", trianglesBuffer);
			materialGroupMaterial[i].SetBuffer("Normals", normalsBuffer);
			materialGroupMaterial[i].SetBuffer("UVs", uvBuffer);
			materialGroupMaterial[i].SetBuffer("TextureUVs", textureUVBuffer);
			materialGroupMaterial[i].SetInt("oneSidedNumTriangles", numOneSidedTrianglesTotal * 3);
			materialGroupMaterial[i].SetInt("_InEditor", 0);
		}

		commandBuffers = new GraphicsBuffer[numTextureGroups];
		matProps = new MaterialPropertyBlock[numTextureGroups];
		renderedTriangleLocalStartIndexBuffers = new ComputeBuffer[numTextureGroups];
		renderedTriangleOffsetsBuffers = new ComputeBuffer[numTextureGroups];
		textureDoubleSidedBuffers = new ComputeBuffer[numTextureGroups];

		for (int i = 0; i < numTextureGroups; i++) {

			renderedTriangleLocalStartIndexNative = new NativeArray<int>(numSidesPerGroup[i], Allocator.Temp);
			renderedTriangleOffsetsNative = new NativeArray<int>(numSidesPerGroup[i], Allocator.Temp);
			textureDoubleSidedNative = new NativeArray<uint>(numSidesPerGroup[i], Allocator.Temp);

			int numTrianglesInGroup = 0;

			int sideCount = 0;
			for (int j = textureGroupFirstIndex[i]; j < textureGroupLastIndex[i] + 1; j++) {
				int clothIndex = textureSortedClothsNative[j];

				renderedTriangleLocalStartIndexNative[sideCount] = numTrianglesInGroup;
				renderedTriangleOffsetsNative[sideCount] = activeTriangleStartIndexNative[renderedToSortedNative[j]];
				textureDoubleSidedNative[sideCount] = (uint)(cloths[clothIndex].textureSide == ClothSimulation.TextureSide.Both || cloths[clothIndex].textureSide == ClothSimulation.TextureSide.Front ? 1 : 0);

				numTrianglesInGroup += cloths[clothIndex].numOneSidedTriangles * 3;

				sideCount++;

				if (cloths[clothIndex].isDoubleSided) {
					renderedTriangleLocalStartIndexNative[sideCount] = numTrianglesInGroup;
					renderedTriangleOffsetsNative[sideCount] = activeBackTriangleStartIndexNative[renderedToSortedNative[j]] + numOneSidedTrianglesTotal * 3;
					textureDoubleSidedNative[sideCount] = (uint)(cloths[clothIndex].textureSide == ClothSimulation.TextureSide.Both || cloths[clothIndex].textureSide == ClothSimulation.TextureSide.Back ? 1 : 0);

					numTrianglesInGroup += cloths[clothIndex].numOneSidedTriangles * 3;
					sideCount++;
				}

			}

			renderedTriangleLocalStartIndexBuffers[i] = ComputeHelper.CreateStructuredBuffer<int>(numSidesPerGroup[i]);
			renderedTriangleLocalStartIndexBuffers[i].SetData(renderedTriangleLocalStartIndexNative);

			renderedTriangleOffsetsBuffers[i] = ComputeHelper.CreateStructuredBuffer<int>(numSidesPerGroup[i]);
			renderedTriangleOffsetsBuffers[i].SetData(renderedTriangleOffsetsNative);

			textureDoubleSidedBuffers[i] = ComputeHelper.CreateStructuredBuffer<uint>(numSidesPerGroup[i]);
			textureDoubleSidedBuffers[i].SetData(textureDoubleSidedNative);

			matProps[i] = new MaterialPropertyBlock();
			matProps[i].SetBuffer("TriangleLocalStartIndex", renderedTriangleLocalStartIndexBuffers[i]);
			matProps[i].SetBuffer("TriangleOffsets", renderedTriangleOffsetsBuffers[i]);
			matProps[i].SetBuffer("TextureDoubleSided", textureDoubleSidedBuffers[i]);
			matProps[i].SetInt("numSides", sideCount);

			if ((int)cloths[textureSortedClothsNative[textureGroupFirstIndex[i]]].clothTexture > 0) {
				matProps[i].SetInt("_UseTexture", 1);
				matProps[i].SetTexture("_Texture", textures[(int)cloths[textureSortedClothsNative[textureGroupFirstIndex[i]]].clothTexture].texture);
			}

			GraphicsBuffer.IndirectDrawArgs[] commandData = new GraphicsBuffer.IndirectDrawArgs[1];

			commandData[0].vertexCountPerInstance = (uint)numTrianglesInGroup;
			commandData[0].instanceCount = 1;
			commandData[0].startVertex = 0;
			commandData[0].startInstance = 0;

			commandBuffers[i] = new GraphicsBuffer(GraphicsBuffer.Target.IndirectArguments, 1, GraphicsBuffer.IndirectDrawArgs.size);
			commandBuffers[i].SetData(commandData);

		}

		DisposeTempRenderingNatives();

		renderingGenerated = true;
		return;
	}

	public void UpdateMasses() {
		if (!buffersGenerated || numSimulatedVerts == 0) return;

		wBuffer?.Release();

		wNative = new NativeArray<float>(numSimulatedVerts, Allocator.Temp);

		for (int i = 0; i < numSimulatedCloths; i++) {
			int clothIndex = sortedClothIndicesNative[i];

			unsafe {
				// Copy Inverse Masses
				int size = Marshal.SizeOf(typeof(float)) * cloths[clothIndex].numOneSidedVerts;
				IntPtr sourcePtr = Marshal.UnsafeAddrOfPinnedArrayElement(cloths[clothIndex].w, 0);
				IntPtr destPtr = (IntPtr)((float*)wNative.GetUnsafePtr() + activeVertexStartIndexNative[i]); // Calculate offset for the subset
				UnsafeUtility.MemCpy((void*)destPtr, (void*)sourcePtr, size);
			}
		}

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

		wNative.Dispose();
	}

	private void UpdatePinnedVertices() {
		if (!buffersGenerated || numSimulatedVerts == 0) return;

		pinnedVertBuffer?.Release();

		if (pinnedVertNative.IsCreated) pinnedVertNative.Dispose();

		numPinnedVertices = 0;

		for (int i = 0; i < activeClothIndices.Count; i++) {
			if (cloths[activeClothIndices[i]].isSimulating) {
				numPinnedVertices += cloths[activeClothIndices[i]].pinnedVertLocalPos.Count;
			}
		}

		wNative = new NativeArray<float>(numSimulatedVerts, Allocator.Temp);
		vertToPinnedNative = new NativeArray<int>(numSimulatedVerts, Allocator.Temp);

		pinnedVertNative = new NativeArray<Vector3>(numPinnedVertices, Allocator.Persistent);

		int numPinned = 0;

		for (int i = 0; i < numSimulatedCloths; i++) {
			int clothIndex = sortedClothIndicesNative[i];

			unsafe {
				// Copy Inverse Masses
				int size = Marshal.SizeOf(typeof(float)) * cloths[clothIndex].numOneSidedVerts;
				IntPtr sourcePtr = Marshal.UnsafeAddrOfPinnedArrayElement(cloths[clothIndex].w, 0);
				IntPtr destPtr = (IntPtr)((float*)wNative.GetUnsafePtr() + activeVertexStartIndexNative[i]); // Calculate offset for the subset
				UnsafeUtility.MemCpy((void*)destPtr, (void*)sourcePtr, size);
			}

			// Map pinned vertices locations
			for (int j = 0; j < cloths[clothIndex].pinnedVertLocalPos.Count; j++) {
				vertToPinnedNative[cloths[clothIndex].LODs[cloths[clothIndex].activeLODIndex].pinIndices[j] + activeVertexStartIndexNative[i]] = numPinned++;
				//pinnedVertNative[numPinned++] = cloths[clothIndex].transform.TransformPoint(cloths[clothIndex].pinnedVertLocalPos[j]);
			}
		}

		wBuffer.SetData(wNative);

		vertToPinnedBuffer.SetData(vertToPinnedNative);

		if (numPinnedVertices > 0) {
			pinnedVertBuffer = ComputeHelper.CreateStructuredBuffer<Vector3>(numPinnedVertices);
			clothCompute.SetBuffer(predictPositionKernel, "pinnedVertPos", pinnedVertBuffer);
		}
		

		wNative.Dispose();
		vertToPinnedNative.Dispose();
		refreshPinnedQueued = false;
	}

	public void UpdateConstraintAlphas() {
		if (!buffersGenerated || numSimulatedVerts == 0) return;

		stretchingAlphaBuffer?.Release();
		bendingAlphaBuffer?.Release();

		stretchingAlphaNative = new NativeArray<float>(numSimulatedCloths, Allocator.Temp);
		bendingAlphaNative = new NativeArray<float>(numSimulatedCloths, Allocator.Temp);

		for (int i = 0; i < numSimulatedCloths; i++) {
			int clothIndex = sortedClothIndicesNative[i];

			float dt_step = Time.fixedDeltaTime / substepsNative[i];
			stretchingAlphaNative[i] = cloths[clothIndex].stretchingCompliance / (dt_step * dt_step);
			bendingAlphaNative[i] = cloths[clothIndex].bendingCompliance / (dt_step * dt_step);
		}

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

		stretchingAlphaNative.Dispose();
		bendingAlphaNative.Dispose();
	}

	private void UpdateConstraintLengths() {
		if (!buffersGenerated || numSimulatedVerts == 0) return;

		for (int i = 0; i < stretchBatches; i++) {
			d0[i].Clear();
		}

		for (int i = 0; i < bendingBatches; i++) {
			dihedral0[i].Clear();
		};

		stepTimeNative = new NativeArray<float>(numSimulatedCloths, Allocator.Temp);
		stretchingAlphaNative = new NativeArray<float>(numSimulatedCloths, Allocator.Temp);
		bendingAlphaNative = new NativeArray<float>(numSimulatedCloths, Allocator.Temp);
		solveBendingNative = new NativeArray<int>(numSimulatedCloths, Allocator.Temp);

		for (int i = 0; i < numActiveCloths; i++) {
			if (i < numSimulatedCloths) {
				int clothIndex = sortedClothIndicesNative[i];

				float dt_step = Time.fixedDeltaTime / substepsNative[i];
				stretchingAlphaNative[i] = cloths[clothIndex].stretchingCompliance / (dt_step * dt_step);
				bendingAlphaNative[i] = cloths[clothIndex].bendingCompliance / (dt_step * dt_step);
				stepTimeNative[i] = dt_step;
				solveBendingNative[i] = cloths[clothIndex].solveBending ? 1 : 0;

				// Create combined constraint information
				for (int j = 0; j < stretchBatches; j++) {
					d0[j].AddRange(cloths[clothIndex].d0[j]);

				}

				for (int j = 0; j < bendingBatches; j++) {
					dihedral0[j].AddRange(cloths[clothIndex].dihedral0[j]);

				}
			}
		}

		for (int i = 0; i < stretchBatches; i++) {
			d0Buffers[i].SetData(d0[i]);
		}

		for (int i = 0; i < bendingBatches; i++) {
			dihedral0Buffers[i].SetData(dihedral0[i]);
		}

		stretchingAlphaBuffer.SetData(stretchingAlphaNative);
		bendingAlphaBuffer.SetData(bendingAlphaNative);
		solveBendingBuffer.SetData(solveBendingNative);
		stepTimeBuffer.SetData(stepTimeNative);

		stepTimeNative.Dispose();
		stretchingAlphaNative.Dispose();
		bendingAlphaNative.Dispose();
		solveBendingNative.Dispose();

		refreshConstraintLengthsQueued = false;
	}

	public void UpdateTimestep() {
		if (numActiveCloths == 0) {
			return; // Return if no cloths are active/initialized/visible
		}

		stretchingAlphaBuffer?.Dispose();
		bendingAlphaBuffer?.Dispose();
		stepTimeBuffer?.Dispose();
		maxVelocityBuffer?.Dispose();

		// Create Arrays
		stretchingAlphaNative = new NativeArray<float>(numSimulatedCloths, Allocator.Temp);
		bendingAlphaNative = new NativeArray<float>(numSimulatedCloths, Allocator.Temp);
		stepTimeNative = new NativeArray<float>(numSimulatedCloths, Allocator.Temp);
		maxVelocityNative = new NativeArray<float>(numSimulatedCloths, Allocator.Temp);

		for (int i = 0; i < numActiveCloths; i++) {

			int clothIndex = sortedClothIndicesNative[i];

			// If cloth is being simulated
			if (i < numSimulatedCloths) {
				float dt_step = Time.fixedDeltaTime * Time.timeScale / substepsNative[i];
				stretchingAlphaNative[i] = cloths[clothIndex].stretchingCompliance / (dt_step * dt_step);
				bendingAlphaNative[i] = cloths[clothIndex].bendingCompliance / (dt_step * dt_step);
				stepTimeNative[i] = dt_step;
				maxVelocityNative[i] = 1 / stepTimeNative[i];
			}
		}

		if (numSimulatedCloths == 0) {
			return;
		}

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

		maxVelocityBuffer = ComputeHelper.CreateStructuredBuffer<float>(numSimulatedCloths);
		maxVelocityBuffer.SetData(maxVelocityNative);
		clothCompute.SetBuffer(predictPositionKernel, "maxVelocity", maxVelocityBuffer);

		stretchingAlphaNative.Dispose();
		bendingAlphaNative.Dispose();
		stepTimeNative.Dispose();
		maxVelocityNative.Dispose();
	}

	public void UpdateDragCoeff() {
		clothCompute.SetFloat("dragCoeffPerp", dragCoeffPerp);
		clothCompute.SetFloat("dragCoeffShear", dragCoeffShear);
	}

	void OnDestroy() {
		ForceRelease();
	}

	private void OnDisable() {
		ForceRelease();

		StopSolvingLODs();
	}

	private void OnEnable() {
		if (isInitialized) RefreshAll();
	}

	private void ForceRelease() {
		try {
			if (buffersGenerated) {
				Vector3[] x = new Vector3[numActiveVerts];
				Vector3[] normals = new Vector3[numActiveVerts];
				xBuffer.GetData(x);
				normalsBuffer.GetData(normals);

				if (normalsNative.IsCreated) normalsNative.Dispose();
				normalsNative = new NativeArray<Vector3>(normals, Allocator.TempJob);

				//normalsInverse = normals.Select(x => -x).ToArray();
				/*InvertNormalsJob invertJob = new InvertNormalsJob {
					normalsIn = normalsNative,
					normalsOut = normalsInverseNative
				};

				JobHandle jobHandle = invertJob.Schedule(numSimulatedVerts, 64);
				jobHandle.Complete();*/

				// Update positions for each cloth
				for (int i = 0; i < numSimulatedCloths; i++) {
					// Copy specific cloth's data from global arrays
					if (cloths[sortedClothIndicesNative[i]] != null && cloths[sortedClothIndicesNative[i]].isActiveAndEnabled) {
						unsafe {
							// Copy Vertices
							int size = Marshal.SizeOf(typeof(Vector3)) * cloths[sortedClothIndicesNative[i]].numOneSidedVerts;
							IntPtr sourcePtr = Marshal.UnsafeAddrOfPinnedArrayElement(x, activeVertexStartIndexNative[i]);
							IntPtr destPtr = (IntPtr)((Vector3*)cloths[sortedClothIndicesNative[i]].x.GetUnsafePtr());
							UnsafeUtility.MemCpy((void*)destPtr, (void*)sourcePtr, size);

							// Copy Normals
							size = Marshal.SizeOf(typeof(Vector3)) * cloths[sortedClothIndicesNative[i]].numOneSidedVerts;
							sourcePtr = (IntPtr)((Vector3*)normalsNative.GetUnsafePtr() + activeVertexStartIndexNative[i]); // Calculate offset for the subset
							destPtr = (IntPtr)((Vector3*)cloths[sortedClothIndicesNative[i]].normals.GetUnsafePtr());
							UnsafeUtility.MemCpy((void*)destPtr, (void*)sourcePtr, size);

						}
					}
				}

				normalsNative.Dispose();
			}
		} finally {
			Release();
			buffersGenerated = false;
		}

	}

	private void DisposeTempRenderingNatives() {
		if (textureSortedClothsNative.IsCreated) textureSortedClothsNative.Dispose();
		if (materialSortedClothsNative.IsCreated) materialSortedClothsNative.Dispose();
		if (renderedToSortedNative.IsCreated) renderedToSortedNative.Dispose();
		if (renderedTriangleLocalStartIndexNative.IsCreated) renderedTriangleLocalStartIndexNative.Dispose();
		if (renderedTriangleOffsetsNative.IsCreated) renderedTriangleOffsetsNative.Dispose();
		if (textureDoubleSidedNative.IsCreated) textureDoubleSidedNative.Dispose();
		if (materialToSortedNative.IsCreated) materialToSortedNative.Dispose();
	}

	private void DisposeOrderingNatives() {
		if (activeVertexStartIndexNative.IsCreated) activeVertexStartIndexNative.Dispose();
		if (activeTriangleStartIndexNative.IsCreated) activeTriangleStartIndexNative.Dispose();
		if (activeBackTriangleStartIndexNative.IsCreated) activeBackTriangleStartIndexNative.Dispose();
		if (sortedClothIndicesNative.IsCreated) sortedClothIndicesNative.Dispose();
		if (substepsNative.IsCreated) substepsNative.Dispose();
	}

	private void DisposeTempSimulationNatives() {
		if (xNative.IsCreated) xNative.Dispose();
		if (normalsNative.IsCreated) normalsNative.Dispose();
		if (trianglesNative.IsCreated) trianglesNative.Dispose();
		if (uvNative.IsCreated) uvNative.Dispose();
		if (wNative.IsCreated) wNative.Dispose();
		if (vNative.IsCreated) vNative.Dispose();
		if (stretchingAlphaNative.IsCreated) stretchingAlphaNative.Dispose();
		if (bendingAlphaNative.IsCreated) bendingAlphaNative.Dispose();
		if (stepTimeNative.IsCreated) stepTimeNative.Dispose();
		if (dampingNative.IsCreated) dampingNative.Dispose();
		if (maxVelocityNative.IsCreated) maxVelocityNative.Dispose();
		if (vertexToTrianglesNative.IsCreated) vertexToTrianglesNative.Dispose();
		if (triangleNormalsNative.IsCreated) triangleNormalsNative.Dispose();
		if (dragFactorNative.IsCreated) dragFactorNative.Dispose();
		if (localGravityVectorsNative.IsCreated) localGravityVectorsNative.Dispose();
		if (vertToPinnedNative.IsCreated) vertToPinnedNative.Dispose();
		if (clothPositionDeltaNative.IsCreated) clothPositionDeltaNative.Dispose();
		if (connectedVertsNative.IsCreated) connectedVertsNative.Dispose();
		if (textureUVNative.IsCreated) textureUVNative.Dispose();
		if (solveBendingNative.IsCreated) solveBendingNative.Dispose();
		if (oneSidedWindNative.IsCreated) oneSidedWindNative.Dispose();
		if (clothIndicesNative.IsCreated) clothIndicesNative.Dispose();
	}

	private void ReleaseSimulationBuffersAndNatives() {
		if (applyWindNative.IsCreated) applyWindNative.Dispose();
		if (numVertsPerSubstepNative.IsCreated) numVertsPerSubstepNative.Dispose();
		if (windVectorsNative.IsCreated) windVectorsNative.Dispose();
		if (pinnedVertNative.IsCreated) pinnedVertNative.Dispose();

		ComputeHelper.Release(xBuffer, normalsBuffer, trianglesBuffer, uvBuffer, vBuffer, wBuffer, pBuffer, stepVelocityBuffer, gravityVectorBuffer, windVectorBuffer, substepsBuffer, stretchingAlphaBuffer, bendingAlphaBuffer, stepTimeBuffer, dampingBuffer, maxVelocityBuffer, vertexToTrianglesBuffer, triangleNormalsBuffer, dragFactorBuffer, pinnedVertBuffer, vertToPinnedBuffer, clothWindForceBuffer, connectedVertsBuffer, textureUVBuffer, solveBendingBuffer, oneSidedWindBuffer, clothIndicesBuffer);
		ComputeHelper.Release(stretchingIDsBuffers);
		ComputeHelper.Release(d0Buffers);
		ComputeHelper.Release(bendingIDsBuffers);
		ComputeHelper.Release(dihedral0Buffers);
	}

	private void ReleaseRenderingBuffers() {
		ComputeHelper.Release(renderedTriangleLocalStartIndexBuffers);
		ComputeHelper.Release(renderedTriangleOffsetsBuffers);
		ComputeHelper.Release(textureDoubleSidedBuffers);

		ReleaseCommandBuffers();
	}

	void Release() {
		DisposeOrderingNatives();
		ReleaseSimulationBuffersAndNatives();
		ReleaseRenderingBuffers();
		ReleaseCommandBuffers();

		if (pinnedVertNativeBackup.IsCreated) pinnedVertNativeBackup.Dispose();
		if (pinnedPerClothNativeBackup.IsCreated) pinnedPerClothNativeBackup.Dispose();
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

[System.Serializable]
public struct ClothMaterial {
	public string name;
	public Material material;
	public float surfaceDensity;
}

[Serializable] public struct ClothTexture {
	public string name;
	public Texture2D texture;
}