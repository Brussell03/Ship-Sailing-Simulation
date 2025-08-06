using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Threading.Tasks;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.VisualScripting;
using UnityEngine;

[RequireComponent(typeof(MeshFilter)), RequireComponent(typeof(MeshRenderer)), RequireComponent(typeof(BoxCollider)), ExecuteAlways]
public class ClothSimulation : MonoBehaviour {

	[Header("Cloth Properties")]
	public ClothMaterial clothMaterial;
	public Vector2 compression = Vector2.zero;
	[Tooltip("Effects how much bending constraints can be tightened.")]
	[Range(0, 1)] public float wrinkliness = 0f;
	[Tooltip("Effects how much vertices are scrambled away from their grid position.")]
	[Range(0, 1)] public float randomness = 0.5f;
	[Tooltip("Vertex randomness seed.")]
	[SerializeField] private int seed;
	[Tooltip("What rigidbody the force is applied to.")] public Rigidbody forceRigidbody;
	public bool isDoubleSided = true;
	public bool applyGravity = true;
	public bool applyWind = true;
	public bool oneSidedWind = true;

	[Header("Shape Settings")]
	public bool liveEdit = false;
	public ClothShape clothShape = ClothShape.Quadrilateral;
	[Min(2)] public int baseNumRows = 15;
	[Min(2)] public int baseNumColumns = 15;
	[Min(0.0000001f)] public float topWidth = 10f;
	[Min(0.0000001f)] public float bottomWidth = 10f;
	[Min(0.0000001f)] public float leftHeight = 10f;
	[Min(0.0000001f)] public float rightHeight = 10f;
	public float topCenter = 0f;
	public float bottomCenter = 0f;
	public float leftCenter = 0f;
	public float rightCenter = 0f;

	[Header("Simulation Settings")]
	[Range(0, 0.0001f)] public float stretchingCompliance = 0f;
	[Range(0, 10)] public float bendingCompliance = 0f;
	[Range(0, 1)] public float damping = 0.03f;
	[Min(0.0000001f)] public float wScale = 1f;
	public bool simulateGravity = true;
	public bool simulateWind = true;
	public bool solveBending = true;
	[SerializeField] private bool _isSimulating = true;
	public bool isSimulating {
		get { return _isSimulating; }
		set {
			if (_isSimulating != value) {
				_isSimulating = value;

				Debug.Log($"isSimulating changed to: {_isSimulating}");
				if (dispatcher != null) dispatcher.refreshAllQueued = true;

#if UNITY_EDITOR
				UnityEditor.EditorUtility.SetDirty(this);
#endif
			}
		}
	}
	[SerializeField] private bool _isRendering = true;
	public bool isRendering {
		get { return _isRendering; }
		set {
			if (_isRendering != value) {
				_isRendering = value;

				Debug.Log($"isRendering changed to: {_isRendering}");
				if (dispatcher != null) dispatcher.refreshRenderingQueued = true;

#if UNITY_EDITOR
				UnityEditor.EditorUtility.SetDirty(this);
#endif
			}
		}
	}

	[Header("Texture Settings")]
	[SerializeField] private ClothTexture _clothTexture;
	public ClothTexture clothTexture {
		get { return _clothTexture; }
		set {
			if (_clothTexture != value) {
				_clothTexture = value;

				Debug.Log($"clothTexture changed to: {_clothTexture}");
				if (dispatcher != null) dispatcher.refreshRenderingQueued = true;
#if UNITY_EDITOR
				UnityEditor.EditorUtility.SetDirty(this);
#endif
			}
		}
	}
	public TextureSide textureSide = TextureSide.Both;
	public Vector2 textureSize = Vector2.one;
	public Vector2 textureOffset = Vector2.zero;
	[Range(0, 360)] public float textureRotation = 0;

	[Header("Pin Cutting")]
	[Tooltip("Can pinned vertices be cut.")]
	public bool cuttablePins = false;
	[Tooltip("Momentum required to cut pins.")]
	public float minCuttingMomentum = 10;
	[SerializeField] public List<string> collisionTags = new List<string>();

	// Simulation and Mesh Data
	public NativeArray<Vector3> x;
	public NativeArray<Vector3> v;
	public NativeArray<Vector3> normals;
	public NativeArray<Vector2> uv;
	public NativeArray<Vector2> textureUV;
	public float[] w { get; set; }
	public int[] triangles { get; set; }
	public float[][] d0 { get; set; }
	public float[][] dihedral0 { get; set; }
	public int[][] bendingIDs { get; set; }
	private int[] neighbors;
	public int[][] stretchingIDs { get; set; }
	public float[] dragFactor { get; set; }

	// Booleans
	public bool isInitialized { get; set; } = false;
	public bool showShapingGizmos { get; set; } = false;
	public bool isChangingLOD { get; set; } = false;
	public bool readbackComplete { get; set; } = false;

	// Integers
	public int clothIndex { get; set; }
	public int maxNumRows { get; private set; }
	public int maxNumColumns { get; private set; }
	public int maxSubdivisions { get; private set; }
	public int maxQualityLODIndex { get; private set; }
	public int numOneSidedVerts { get; private set; }
	public int numOneSidedTriangles { get; private set; }
	public int targetLODIndex { get; set; }
	public int activeLODIndex { get; private set; }
	private int cutPinStartIndex = -1;

	// Floats
	public float distBetweenHandles { get; private set; }
	[field: SerializeField] public float mass { get; private set; }
	private float hashSpacing = 1f;
	private float airDensity = 1.287f; // kg/m^3

	// Objects
	public MeshFilter meshFilter { get; set; }
	public MeshRenderer meshRenderer { get; private set; }
	public Texture2D alphaMap { get; private set; }
	public ClothDispatcher dispatcher { get; private set; }
	private BoxCollider collider;
	private BuoyantHull hull;
	private Hash pinnedVerticesHash;

	// Vectors
	public Vector3 windForce { get; set; }
	private Vector3 pinnedCenter;

	// Lists
	[HideInInspector] public List<int>[] vertexToTriangles { get; private set; }

	[HideInInspector, SerializeField] private List<IndexAndVector3> _pinnedVertices = new List<IndexAndVector3>();
	public List<IndexAndVector3> pinnedVertices => _pinnedVertices; // Read-only reference variable to _pinnedVertices
	public List<Vector3> pinnedVertLocalPos { get; private set; } = new List<Vector3>();
	private List<float> pinnedVertInvMass = new List<float>();

	[HideInInspector] public List<int> selectedVertices = new List<int>();

	[SerializeField] public List<ClothLOD> LODs = new List<ClothLOD>();

	[SerializeField] public List<ConnectedVertex> connectedVertices = new List<ConnectedVertex>();

	// Pin Cutting
	private HashSet<Collider> _currentTriggers = new HashSet<Collider>();
	List<Vector3> collisionTestPoints = new List<Vector3>();
	List<int> collisionCutIndices = new List<int>();
	public List<IndexAndVector3> cutPins = new List<IndexAndVector3>();

	public enum ClothMaterial : int { };

	public enum ClothShape {
		Quadrilateral,
		Trapezoidal
	};

	public enum TextureSide : int {
		Front,
		Back,
		Both
	};

	public enum ClothTexture : int { };

	/*private NativeArray<Vector3> partialSums;

	[BurstCompile]
	public struct ParallelPartialSumJob : IJobParallelFor {
		[ReadOnly]
		public NativeArray<Vector3> InputVectors;

		[NativeDisableParallelForRestriction] // Required to allow different `Execute` calls to write to distinct indices.
		public NativeArray<Vector3> PartialSums;

		public void Execute(int index) {
			PartialSums[index / 64] += InputVectors[index];
		}
	}*/

	[Serializable]
	public struct ConnectedVertex {
		[Tooltip("What cloth the vertex is connected to.")]
		public ClothSimulation connectedCloth;
		[Tooltip("Index of the connected vertices list on the connected cloth.")]
		public int targetListIndex;
		[Tooltip("Index of this cloth's vertices that is connected.")]
		public int vertIndex;
		public bool isSet;
	}

	[Serializable]
	public struct ClothLOD {
		[Min(0)] public float maxDistance;
		[Min(0)] public int subdivisions;
		[Range(1, 30)] public int substeps;

		public int numRows { get; set; }
		public int numColumns { get; set; }
		public int detailLevel { get; set; }
		public int numVertices { get; set; }
		public float maxDistanceSqr { get; set; }
		public List<int> pinIndices { get; set; }
		public List<int> pinListIndices { get; set; }
	}

	// Start is called before the first frame update
	void Start()
	{
		meshFilter = GetComponent<MeshFilter>();
		meshRenderer = GetComponent<MeshRenderer>();
		collider = GetComponent<BoxCollider>();
		dispatcher = FindAnyObjectByType<ClothDispatcher>();
		hull = GetComponentInParent<BuoyantHull>();

		if (dispatcher != null ) {
			dispatcher.numClothsChanged = true;
		}

		liveEdit = false;
		targetLODIndex = 0;
		activeLODIndex = 0;

		InitLODs();

		Init();

		if (Application.isPlaying) {
			UpdatePinnedBounds();
			collider.enabled = true;

			if (cuttablePins) pinnedVerticesHash = new Hash(hashSpacing, pinnedVertices.Count);

			meshRenderer.enabled = false;

		} else {
			collider.enabled = false;
			collider.size = Vector3.zero;
			dispatcher.materials[(int)clothMaterial].material.SetInt("_InEditor", 1);

			if ((int)clothTexture > 0) {
				Material mat = new Material(dispatcher.materials[(int)clothMaterial].material);
				mat.SetInt("_UseTexture", 1);
				mat.SetTexture("_Texture", dispatcher.textures[(int)clothTexture].texture);
				meshRenderer.sharedMaterial = mat;
			}
		}

#if UNITY_EDITOR
		if (false && LODs[activeLODIndex].numRows <= 40 && LODs[activeLODIndex].numColumns <= 40) {
			for (int i = 0; i < d0.Length; i++) {
				for (int j = 0; j < d0[i].Length; j++) {
					int id0 = stretchingIDs[i][j * 2];
					int id1 = stretchingIDs[i][j * 2 + 1];

					Vector3 p0 = x[id0];
					Vector3 p1 = x[id1];

					if (i == 0) Debug.DrawLine(p0, p1, Color.yellow);
					else if (i == 1) Debug.DrawLine(p0, p1, Color.red);
					else if (i == 2) Debug.DrawLine(p0, p1, Color.blue);
					else if (i == 3) Debug.DrawLine(p0, p1, Color.green);
					else if (i == 4) Debug.DrawLine(p0, p1, Color.magenta);
					else if (i == 5) Debug.DrawLine(p0, p1, Color.black);
				}
			}

			for (int i = 0; i < dihedral0.Length; i++) {
				for (int j = 0; j < dihedral0[i].Length; j++) {
					int id0 = bendingIDs[i][j * 2];
					int id1 = bendingIDs[i][j * 2 + 1];

					Vector3 p0 = x[id0];
					Vector3 p1 = x[id1];

					if (i == 0) Debug.DrawLine(p0, p1, Color.white);
					else if (i == 1) Debug.DrawLine(p0, p1, Color.cyan);
					else if (i == 2) Debug.DrawLine(p0, p1, Color.green);
					else if (i == 3) Debug.DrawLine(p0, p1, Color.red);
					else if (i == 4) Debug.DrawLine(p0, p1, Color.yellow);
					else if (i == 5) Debug.DrawLine(p0, p1, Color.magenta);
				}
			}
		}
#endif
	}
	void Update()
	{
		if (!Application.isPlaying) return;

		if (cuttablePins && isInitialized && pinnedVertices.Count > 0) {
			if (pinnedVerticesHash == null) pinnedVerticesHash = new Hash(hashSpacing, pinnedVertices.Count);

			pinnedVerticesHash.Create(pinnedVertices);
		}

		if (readbackComplete) {
			readbackComplete = false;
			ChangeLOD(targetLODIndex);
		}
	}

	private void FixedUpdate() {

		/*int batchSize = 64;
		int numPartialSums = (numOneSidedVerts + batchSize - 1) / batchSize; // Ceiling division

		partialSums = new NativeArray<Vector3>(numPartialSums, Allocator.TempJob);

		ParallelPartialSumJob parallelSumJob = new ParallelPartialSumJob {
			InputVectors = windForcesNative,
			PartialSums = partialSums
		};

		JobHandle parallelJobHandle = parallelSumJob.Schedule(numOneSidedVerts, batchSize);
		parallelJobHandle.Complete();

		Vector3 windForceSum = Vector3.zero;
		for (int i = 0; i < partialSums.Length; i++) {
			windForceSum += partialSums[i];
		}

		if (partialSums.IsCreated) partialSums.Dispose();

		Debug.Log("Total Wind Force: " + windForceSum);*/

		//Debug.Log("Total Wind Force: " + windForce);

		if (forceRigidbody != null) {
			if (applyWind && pinnedVertices != null && pinnedVertices.Count > 0 && applyGravity) {

				forceRigidbody.AddForceAtPosition(windForce + mass * Physics.gravity, transform.TransformPoint(pinnedCenter));

				Debug.DrawRay(transform.TransformPoint(pinnedCenter), (windForce + mass * Physics.gravity) / (hull != null ? hull.debugForceScaling : 1000), Color.red);

			} else {
				if (applyGravity) {
					forceRigidbody.AddForceAtPosition(mass * Physics.gravity, transform.TransformPoint(pinnedCenter));

					Debug.DrawRay(transform.TransformPoint(pinnedCenter), (mass * Physics.gravity) / (hull != null ? hull.debugForceScaling : 1000), Color.red);

				}
				if (applyWind && pinnedVertices != null && pinnedVertices.Count > 0) {
					forceRigidbody.AddForceAtPosition(windForce, transform.TransformPoint(pinnedCenter));

					Debug.DrawRay(transform.TransformPoint(pinnedCenter), (windForce) / (hull != null ? hull.debugForceScaling : 1000), Color.red);
				}
			}
		}
		
	}

	private void LateUpdate() {
		if (_currentTriggers.Count > 0) {
			ProcessColliders();
			_currentTriggers.Clear();
		}
	}

	private void SolveCollisions(Vector3[] p) {
		/*float thickness2 = thickness * thickness;

		for (int i = 0; i < x.Length; i++) {
			if (w[i] == 0.0)
				continue;
			int id0 = i;
			int first = hash.firstAdjId[i];
			int last = hash.firstAdjId[i + 1];
			//Debug.Log(first);
			//Debug.Log(last);

			for (int j = first; j < last; j++) {

				int id1 = hash.adjIds[j];
				if (w[id1] == 0.0)
					continue;

				Vector3 vec = p[id1] - p[id0];

				float dist2 = vec.sqrMagnitude;//vecLengthSquared(vecs, 0);
				if (dist2 > thickness2 || dist2 == 0.0)
					continue;
				float restDist2 = (restPos[id0] - restPos[id1]).sqrMagnitude;//vecDistSquared(restPos, id0, restPos, id1);

				float minDist = thickness;
				if (dist2 > restDist2)
					continue;
				if (restDist2 < thickness2)
					minDist = Mathf.Sqrt(restDist2);

				// position correction
				float dist = Mathf.Sqrt(dist2);
				//vecScale(vecs, 0, (minDist - dist) / dist);
				//vecAdd(pos, id0, vecs, 0, -0.5);
				//vecAdd(pos, id1, vecs, 0, 0.5);

				vec *= (minDist - dist) / dist;
				p[id0] += vec * -0.5f;
				p[id1] += vec * 0.5f;

				// velocities
				//vecSetDiff(this.vecs, 0, this.pos, id0, this.prevPos, id0);
				//vecSetDiff(this.vecs, 1, this.pos, id1, this.prevPos, id1);
				Vector3 vec0 = p[id0] - x[id0];
				Vector3 vec1 = p[id1] - x[id1];

				// average velocity
				//vecSetSum(this.vecs, 2, this.vecs, 0, this.vecs, 1, 0.5);
				Vector3 vec2 = (vec0 + vec1) * 0.5f;

				// velocity corrections
				//vecSetDiff(this.vecs, 0, this.vecs, 2, this.vecs, 0);
				//vecSetDiff(this.vecs, 1, this.vecs, 2, this.vecs, 1);
				vec0 = vec2 - vec0;
				vec1 = vec2 - vec1;

				// add corrections
				float friction = 0.0f;
				//vecAdd(this.pos, id0, this.vecs, 0, friction);
				//vecAdd(this.pos, id1, this.vecs, 1, friction);
				p[id0] += vec0 * friction;
				p[id1] += vec1 * friction;
			}
		}*/
	}

	public void Init() {
		if (!gameObject.activeInHierarchy) return;

		pinnedVertLocalPos?.Clear();
		pinnedVertInvMass?.Clear();
		cutPins.Clear();
		cutPinStartIndex = -1;

		if (x.IsCreated) x.Dispose();
		if (normals.IsCreated) normals.Dispose();
		if (v.IsCreated) v.Dispose();
		if (uv.IsCreated) uv.Dispose();
		if (textureUV.IsCreated) textureUV.Dispose();

		switch (clothShape) {
			case ClothShape.Trapezoidal:
				InitTrapezoidal(LODs[activeLODIndex]);
				break;

			default:
				InitQuad(LODs[activeLODIndex]);
				break;
		}

		float maxWidth = Mathf.Max(topWidth, bottomWidth);
		float maxHeight = Mathf.Max(leftHeight, rightHeight);

		distBetweenHandles = (maxWidth > maxHeight ? maxHeight : maxWidth) / (LODs[activeLODIndex].numColumns + LODs[activeLODIndex].numRows);
		hashSpacing = (maxWidth > maxHeight ? maxWidth : maxHeight) * 2f / (LODs[activeLODIndex].numColumns + LODs[activeLODIndex].numRows);

		if (Application.isPlaying) {

			for (int i = 0; i < LODs[activeLODIndex].pinIndices.Count; i++) {
				x[LODs[activeLODIndex].pinIndices[i]] = pinnedVertices[LODs[activeLODIndex].pinListIndices[i]].vector;
			}

		} else {

			Vector3[] xWithPins = x.ToArray();
			for (int i = 0; i < LODs[activeLODIndex].pinIndices.Count; i++) {

				xWithPins[LODs[activeLODIndex].pinIndices[i]] = pinnedVertices[LODs[activeLODIndex].pinListIndices[i]].vector;

				if (isDoubleSided) {
					xWithPins[LODs[activeLODIndex].pinIndices[i] + numOneSidedVerts] = pinnedVertices[LODs[activeLODIndex].pinListIndices[i]].vector;
				}
			}

			Mesh mesh = new Mesh();
			mesh.vertices = xWithPins;
			mesh.uv = uv.ToArray();
			mesh.uv2 = textureUV.ToArray();

			if (isDoubleSided) {
				int[] meshTriangles = new int[triangles.Length];
				for (int i = 0; i < meshTriangles.Length; i++) {
					meshTriangles[i] = i >= triangles.Length / 2 ? triangles[i] + numOneSidedVerts : triangles[i];
				}

				mesh.triangles = meshTriangles;
			} else {
				mesh.triangles = triangles;
			}

			mesh.RecalculateBounds();
			mesh.RecalculateNormals();

			Vector3[] normals = mesh.normals;
			for (int i = 0; i < normals.Length; i++) {
				normals[i] *= -1;
			}

			mesh.normals = normals;
			meshFilter.sharedMesh = mesh;
		}

		isInitialized = true;
	}

	private void InitQuad(ClothLOD lod) {
		int numRows = lod.numRows;
		int numColumns = lod.numColumns;

		// Calculate the number of vertices
		numOneSidedVerts = (numRows + 1) * (numColumns + 1);

		if (isDoubleSided) {
			if (Application.isPlaying) {
				x = new NativeArray<Vector3>(numOneSidedVerts, Allocator.Persistent);
				v = new NativeArray<Vector3>(numOneSidedVerts, Allocator.Persistent);
				normals = new NativeArray<Vector3>(numOneSidedVerts, Allocator.Persistent);
				uv = new NativeArray<Vector2>(numOneSidedVerts, Allocator.Persistent);
				textureUV = new NativeArray<Vector2>(numOneSidedVerts, Allocator.Persistent);
			} else {
				x = new NativeArray<Vector3>(numOneSidedVerts * 2, Allocator.Persistent);
				normals = new NativeArray<Vector3>(numOneSidedVerts * 2, Allocator.Persistent);
				uv = new NativeArray<Vector2>(numOneSidedVerts * 2, Allocator.Persistent);
				textureUV = new NativeArray<Vector2>(numOneSidedVerts * 2, Allocator.Persistent);
			}
			triangles = new int[numRows * numColumns * 12];
			numOneSidedTriangles = triangles.Length / 6;
		} else {
			x = new NativeArray<Vector3>(numOneSidedVerts, Allocator.Persistent);
			normals = new NativeArray<Vector3>(numOneSidedVerts, Allocator.Persistent);
			uv = new NativeArray<Vector2>(numOneSidedVerts, Allocator.Persistent);
			textureUV = new NativeArray<Vector2>(numOneSidedVerts, Allocator.Persistent);
			triangles = new int[numRows * numColumns * 6];
			numOneSidedTriangles = triangles.Length / 3;

			if (Application.isPlaying) {
				v = new NativeArray<Vector3>(numOneSidedVerts, Allocator.Persistent);
			}
		}

		w = new float[numOneSidedVerts];
		dragFactor = new float[numOneSidedVerts];

		// Calculate vertex positions and UV coordinates
		for (int i = 0; i < numRows + 1; i++) {
			float rowWidth = (bottomWidth - topWidth) / numRows * i + topWidth;
			float rowOffset = Mathf.Lerp(topCenter - topWidth / 2f, bottomCenter - bottomWidth / 2f, (float)i / numRows);

			float semiAxisA = rowWidth / (numColumns * 2f);

			for (int j = 0; j < numColumns + 1; j++) {
				float colHeight = (rightHeight - leftHeight) / numColumns * j + leftHeight;
				float colOffset = Mathf.Lerp(leftCenter - leftHeight / 2f, rightCenter - rightHeight / 2f, (float)j / numColumns);

				int index = j + i * (numColumns + 1);
				float xPos = (j * rowWidth) / numColumns + rowOffset;
				float yPos = colHeight - (i * colHeight) / numRows + colOffset;

				x[index] = new Vector3(xPos, yPos, 0);
				uv[index] = new Vector2(xPos, yPos);

				if (!Application.isPlaying && isDoubleSided) {
					x[numOneSidedVerts + index] = new Vector3(xPos, yPos, 0);
					uv[numOneSidedVerts + index] = new Vector2(xPos, yPos);
				}

				if (randomness > 0 && j > 0 && j < numColumns && i > 0 && i < numRows && !lod.pinIndices.Contains(index)) {
					float semiAxisB = colHeight / (numRows * 2f);

					int perVertexSeed = seed + index * (int)Mathf.Pow(2, maxSubdivisions) / (int)Mathf.Pow(2, lod.subdivisions);
					System.Random rand = new System.Random(perVertexSeed);

					float randAngle = (float)rand.NextDouble() * 2f * Mathf.PI;
					float randDist = (float)rand.NextDouble() * 0.6f * randomness;
					Vector3 adjustment = new Vector3(semiAxisA * Mathf.Cos(randAngle), semiAxisB * Mathf.Sin(randAngle), 0) * randDist;

					x[index] += adjustment;
					uv[index] += new Vector2(adjustment.x, adjustment.y);

					if (!Application.isPlaying && isDoubleSided) {
						x[numOneSidedVerts + index] += adjustment;
						uv[numOneSidedVerts + index] += new Vector2(adjustment.x, adjustment.y);
					}
				}

				Vector2 texUV = new Vector2((uv[index].x - textureOffset.x) / textureSize.x, (uv[index].y - textureOffset.y) / textureSize.y);
				Vector2 rotatedTexUV = new Vector2(texUV.x * Mathf.Cos(Mathf.Deg2Rad * textureRotation) - texUV.y * Mathf.Sin(Mathf.Deg2Rad * textureRotation), texUV.x * Mathf.Sin(Mathf.Deg2Rad * textureRotation) + texUV.y * Mathf.Cos(Mathf.Deg2Rad * textureRotation)) + Vector2.one / 2;

				textureUV[index] = rotatedTexUV;

				if (!Application.isPlaying && isDoubleSided) {
					textureUV[numOneSidedVerts + index] = rotatedTexUV;
				}
			}
		}

		// Create arrays to store triangle indices
		vertexToTriangles = new List<int>[numOneSidedVerts];
		for (int i = 0; i < vertexToTriangles.Length; i++) {
			vertexToTriangles[i] = new List<int>(4);
		}
		int triangleIndex = 0;

		// Generate triangle indices
		for (int i = 0; i < numRows; i++) {
			for (int j = 0; j < numColumns; j++) {
				int realTriIndex = triangleIndex / 3;
				int id0 = i * (numColumns + 1) + j;
				int id1 = id0 + 1;
				int id2 = (i + 1) * (numColumns + 1) + j;
				int id3 = id2 + 1;

				vertexToTriangles[id0].Add(realTriIndex);
				vertexToTriangles[id1].Add(realTriIndex);
				vertexToTriangles[id2].Add(realTriIndex);
				triangles[triangleIndex++] = id0;
				triangles[triangleIndex++] = id1;
				triangles[triangleIndex++] = id2;

				vertexToTriangles[id1].Add(realTriIndex);
				vertexToTriangles[id3].Add(realTriIndex);
				vertexToTriangles[id2].Add(realTriIndex);
				triangles[triangleIndex++] = id1;
				triangles[triangleIndex++] = id3;
				triangles[triangleIndex++] = id2;

				if (isDoubleSided) {
					triangles[triangles.Length / 2 + triangleIndex - 6] = id0;
					triangles[triangles.Length / 2 + triangleIndex - 5] = id2;
					triangles[triangles.Length / 2 + triangleIndex - 4] = id1;

					triangles[triangles.Length / 2 + triangleIndex - 3] = id1;
					triangles[triangles.Length / 2 + triangleIndex - 2] = id2;
					triangles[triangles.Length / 2 + triangleIndex - 1] = id3;
				}
			}
		}

		if (Application.isPlaying) {

			float totalArea = 0f;
			for (int i = 0; i < numRows + 1; i++) {

				for (int j = 0; j < numColumns + 1; j++) {
					int index = j + i * (numColumns + 1);
					float cellArea = 0;

					if (i == 0 && j == 0) {
						// Top Left
						cellArea = GetPolygonArea(x[numColumns + 1], x[numColumns + 2], x[1], x[0]) / 4f;

					} else if (i == numRows && j == 0) {
						// Bottom Left
						cellArea = GetPolygonArea(x[numRows * (numColumns + 1)], x[numRows * (numColumns + 1) + 1], x[(numRows - 1) * (numColumns + 1) + 1], x[(numRows - 1) * (numColumns + 1)]) / 4f;

					} else if (i == numRows && j == numColumns) {
						// Bottom Right
						cellArea = GetPolygonArea(x[numOneSidedVerts - 2], x[numOneSidedVerts - 1], x[numRows * (numColumns + 1) - 1], x[numRows * (numColumns + 1) - 2]) / 4f;

					} else if (i == 0 && j == numColumns) {
						// Top Right
						cellArea = GetPolygonArea(x[numColumns * 2], x[numColumns + 1 + numColumns], x[numColumns], x[numColumns - 1]) / 4f;

					} else if (i == 0) {
						// Top Edge
						cellArea = GetPolygonArea(x[index + numColumns], x[index + numColumns + 2], x[index + 1], x[index - 1]) / 4f;

					} else if (j == 0) {
						// Left Edge
						cellArea = GetPolygonArea(x[index + numColumns + 1], x[index + numColumns + 2], x[index - numColumns], x[index - numColumns - 1]) / 4f;

					} else if (i == numRows) {
						// Bottom Edge
						cellArea = GetPolygonArea(x[index - 1], x[index + 1], x[index - numColumns], x[index - numColumns - 2]) / 4f;

					} else if (j == numColumns) {
						// Right Edge
						cellArea = GetPolygonArea(x[index + numColumns], x[index + numColumns + 1], x[index - numColumns - 1], x[index - numColumns - 2]) / 4f;

					} else {
						// Inner Point
						cellArea = GetPolygonArea(x[index + numColumns], x[index + numColumns + 2], x[index - numColumns], x[index - numColumns - 2]) / 4f;

					}

					cellArea *= (1f + compression.x) * (1f + compression.y);
					w[index] = wScale / (dispatcher.materials[(int)clothMaterial].surfaceDensity * cellArea);
					dragFactor[index] = 0.5f * cellArea * airDensity;
					totalArea += cellArea;
				}
			}

			mass = totalArea * dispatcher.materials[(int)clothMaterial].surfaceDensity;

			for (int i = 0; i < lod.pinIndices.Count; i++) {
				pinnedVertInvMass.Add(w[lod.pinIndices[i]]);
				w[lod.pinIndices[i]] = 0;
				pinnedVertLocalPos.Add(pinnedVertices[lod.pinListIndices[i]].vector);
			}

			InitConstraints(x);
		}
	}

	private void InitTrapezoidal(ClothLOD lod) {
		int numRows = lod.numRows;
		int numColumns = lod.numColumns;

		// Calculate the number of vertices
		int minSubdivision = (int)Mathf.Min(numRows, numColumns);
		numOneSidedVerts = (numRows + 1) * (numColumns + 1) - minSubdivision * (minSubdivision + 1) / 2;

		if (isDoubleSided) {
			if (Application.isPlaying) {
				x = new NativeArray<Vector3>(numOneSidedVerts, Allocator.Persistent);
				v = new NativeArray<Vector3>(numOneSidedVerts, Allocator.Persistent);
				normals = new NativeArray<Vector3>(numOneSidedVerts, Allocator.Persistent);
				uv = new NativeArray<Vector2>(numOneSidedVerts, Allocator.Persistent);
				textureUV = new NativeArray<Vector2>(numOneSidedVerts, Allocator.Persistent);
			} else {
				x = new NativeArray<Vector3>(numOneSidedVerts * 2, Allocator.Persistent);
				normals = new NativeArray<Vector3>(numOneSidedVerts * 2, Allocator.Persistent);
				uv = new NativeArray<Vector2>(numOneSidedVerts * 2, Allocator.Persistent);
				textureUV = new NativeArray<Vector2>(numOneSidedVerts * 2, Allocator.Persistent);
			}
			triangles = new int[numRows * numColumns * 12 - minSubdivision * minSubdivision * 6];
			numOneSidedTriangles = triangles.Length / 6;
		} else {
			x = new NativeArray<Vector3>(numOneSidedVerts, Allocator.Persistent);
			normals = new NativeArray<Vector3>(numOneSidedVerts, Allocator.Persistent);
			uv = new NativeArray<Vector2>(numOneSidedVerts, Allocator.Persistent);
			textureUV = new NativeArray<Vector2>(numOneSidedVerts, Allocator.Persistent);
			triangles = new int[numRows * numColumns * 6 - minSubdivision * minSubdivision * 3];
			numOneSidedTriangles = triangles.Length / 3;

			if (Application.isPlaying) {
				v = new NativeArray<Vector3>(numOneSidedVerts, Allocator.Persistent);
			}
		}

		w = new float[numOneSidedVerts];
		dragFactor = new float[numOneSidedVerts];
		NativeArray<int> rectIndToTriIndNative = new NativeArray<int>((numRows + 1) * (numColumns + 1), Allocator.Temp);
		NativeArray<Vector2> xSquareNative = new NativeArray<Vector2>((numRows + 1) * (numColumns + 1), Allocator.Temp);

		// Calculate vertex positions and UV coordinates
		int ind = 0;
		for (int i = 0; i < numRows + 1; i++) {
			float rowWidth = (bottomWidth - topWidth) / numRows * i + topWidth;
			float rowOffset = Mathf.Lerp(topCenter - topWidth / 2f, bottomCenter - bottomWidth / 2f, (float)i / numRows);

			float semiAxisA = rowWidth / (numColumns * 2f);

			for (int j = 0; j < numColumns + 1; j++) {
				float colHeight = (rightHeight - leftHeight) / numColumns * j + leftHeight;
				float colOffset = Mathf.Lerp(leftCenter - leftHeight / 2f, rightCenter - rightHeight / 2f, (float)j / numColumns);

				float xPos = (j * rowWidth) / numColumns + rowOffset;
				float yPos = colHeight - (i * colHeight) / numRows + colOffset;

				xSquareNative[i * (numColumns + 1) + j] = new Vector2(xPos, yPos);

				if (j >= minSubdivision - i) {

					x[ind] = new Vector3(xPos, yPos, 0);
					uv[ind] = new Vector2(xPos, yPos);

					if (!Application.isPlaying && isDoubleSided) {
						x[numOneSidedVerts + ind] = new Vector3(xPos, yPos, 0);
						uv[numOneSidedVerts + ind] = new Vector2(xPos, yPos);
					}

					if (randomness > 0 && j > minSubdivision - i && j < numColumns && i > 0 && i < numRows && !lod.pinIndices.Contains(ind)) {
						float semiAxisB = colHeight / (numRows * 2f);

						int perVertexSeed = seed + ind * (int)Mathf.Pow(2, maxSubdivisions) / (int)Mathf.Pow(2, lod.subdivisions);
						System.Random rand = new System.Random(perVertexSeed);

						float randAngle = (float)rand.NextDouble() * 2f * Mathf.PI;
						float randDist = (float)rand.NextDouble() * 0.6f * randomness;
						Vector3 adjustment = new Vector3(semiAxisA * Mathf.Cos(randAngle), semiAxisB * Mathf.Sin(randAngle), 0) * randDist;

						x[ind] += adjustment;
						uv[ind] += new Vector2(adjustment.x, adjustment.y);

						if (!Application.isPlaying && isDoubleSided) {
							x[numOneSidedVerts + ind] += adjustment;
							uv[numOneSidedVerts + ind] += new Vector2(adjustment.x, adjustment.y);
						}
					}

					Vector2 texUV = new Vector2((uv[ind].x - textureOffset.x) / textureSize.x, (uv[ind].y - textureOffset.y) / textureSize.y);
					Vector2 rotatedTexUV = new Vector2(texUV.x * Mathf.Cos(Mathf.Deg2Rad * textureRotation) - texUV.y * Mathf.Sin(Mathf.Deg2Rad * textureRotation), texUV.x * Mathf.Sin(Mathf.Deg2Rad * textureRotation) + texUV.y * Mathf.Cos(Mathf.Deg2Rad * textureRotation)) + Vector2.one / 2;

					textureUV[ind] = rotatedTexUV;

					if (!Application.isPlaying && isDoubleSided) {
						textureUV[numOneSidedVerts + ind] = rotatedTexUV;
					}

					rectIndToTriIndNative[i * (numColumns + 1) + j] = ind;
					ind++;
				}
			}
		}


		// Create arrays to store triangle indices
		vertexToTriangles = new List<int>[numOneSidedVerts];
		for (int i = 0; i < vertexToTriangles.Length; i++) {
			vertexToTriangles[i] = new List<int>(4);
		}

		int triangleIndex = 0;
		// Generate triangle indices
		for (int i = 0; i < numRows; i++) {
			for (int j = 0; j < numColumns; j++) {
				if (j < minSubdivision - i - 1) continue;

				int realTriIndex = triangleIndex / 3;
				int id0 = i * (numColumns + 1) + j;
				int id1 = id0 + 1;
				int id2 = (i + 1) * (numColumns + 1) + j;
				int id3 = id2 + 1;

				if (j > minSubdivision - i - 1) {
					vertexToTriangles[rectIndToTriIndNative[id0]].Add(realTriIndex);
					vertexToTriangles[rectIndToTriIndNative[id1]].Add(realTriIndex);
					vertexToTriangles[rectIndToTriIndNative[id2]].Add(realTriIndex);
					triangles[triangleIndex++] = rectIndToTriIndNative[id0];
					triangles[triangleIndex++] = rectIndToTriIndNative[id1];
					triangles[triangleIndex++] = rectIndToTriIndNative[id2];
				}

				vertexToTriangles[rectIndToTriIndNative[id1]].Add(realTriIndex);
				vertexToTriangles[rectIndToTriIndNative[id3]].Add(realTriIndex);
				vertexToTriangles[rectIndToTriIndNative[id2]].Add(realTriIndex);
				triangles[triangleIndex++] = rectIndToTriIndNative[id1];
				triangles[triangleIndex++] = rectIndToTriIndNative[id3];
				triangles[triangleIndex++] = rectIndToTriIndNative[id2];

				if (isDoubleSided) {
					if (j == minSubdivision - i - 1) {

						triangles[triangles.Length / 2 + triangleIndex - 3] = rectIndToTriIndNative[id1];
						triangles[triangles.Length / 2 + triangleIndex - 2] = rectIndToTriIndNative[id2];
						triangles[triangles.Length / 2 + triangleIndex - 1] = rectIndToTriIndNative[id3];
					} else {

						triangles[triangles.Length / 2 + triangleIndex - 6] = rectIndToTriIndNative[id0];
						triangles[triangles.Length / 2 + triangleIndex - 5] = rectIndToTriIndNative[id2];
						triangles[triangles.Length / 2 + triangleIndex - 4] = rectIndToTriIndNative[id1];

						triangles[triangles.Length / 2 + triangleIndex - 3] = rectIndToTriIndNative[id1];
						triangles[triangles.Length / 2 + triangleIndex - 2] = rectIndToTriIndNative[id2];
						triangles[triangles.Length / 2 + triangleIndex - 1] = rectIndToTriIndNative[id3];
					}
				}
			}
		}

		if (Application.isPlaying) {

			float totalArea = 0f;
			ind = 0;
			for (int i = 0; i < numRows + 1; i++) {

				for (int j = 0; j < numColumns + 1; j++) {
					int index = j + i * (numColumns + 1);
					float cellArea = 0;

					if (j < minSubdivision - i) continue;

					if (j == minSubdivision - i) {
						// Top Left
						if (i == 0 && j == numColumns) {
							// Top Right
							cellArea = GetPolygonArea(xSquareNative[numColumns * 2], xSquareNative[numColumns * 2 + 1], xSquareNative[numColumns], xSquareNative[numColumns - 1]) / 8f;

						} else if (i == numRows && j == 0) {
							// Bottom Left
							cellArea = GetPolygonArea(xSquareNative[numRows * (numColumns + 1)], xSquareNative[numRows * (numColumns + 1) + 1], xSquareNative[(numRows - 1) * (numColumns + 1) + 1], xSquareNative[(numRows - 1) * (numColumns + 1)]) / 8f;

						} else if (i == 0) {
							// Top Edge
							cellArea = GetPolygonArea(xSquareNative[index + numColumns], xSquareNative[index + numColumns + 2], xSquareNative[index + 1], xSquareNative[index - 1]) * 3f / 16f;

						} else if (j == 0) {
							// Left Edge
							cellArea = GetPolygonArea(xSquareNative[index + numColumns + 1], xSquareNative[index + numColumns + 2], xSquareNative[index - numColumns], xSquareNative[index - numColumns - 1]) * 3f / 16f;

						} else {
							// Inner Point
							cellArea = GetPolygonArea(xSquareNative[index + numColumns], xSquareNative[index + numColumns + 2], xSquareNative[index - numColumns], xSquareNative[index - numColumns - 2]) / 8f;

						}

					} else if (i == numRows && j == 0) {
						// Bottom Left
						cellArea = GetPolygonArea(xSquareNative[numRows * (numColumns + 1)], xSquareNative[numRows * (numColumns + 1) + 1], xSquareNative[(numRows - 1) * (numColumns + 1) + 1], xSquareNative[(numRows - 1) * (numColumns + 1)]) / 4f;

					} else if (i == numRows && j == numColumns) {
						// Bottom Right
						cellArea = GetPolygonArea(xSquareNative[xSquareNative.Length - 2], xSquareNative[xSquareNative.Length - 1], xSquareNative[numRows * (numColumns + 1) - 1], xSquareNative[numRows * (numColumns + 1) - 2]) / 4f;
					} else if (i == 0 && j == numColumns) {
						// Top Right
						cellArea = GetPolygonArea(xSquareNative[numColumns * 2], xSquareNative[numColumns + 1 + numColumns], xSquareNative[numColumns], xSquareNative[numColumns - 1]) / 4f;

					} else if (i == 0) {
						// Top Edge
						cellArea = GetPolygonArea(xSquareNative[index + numColumns], xSquareNative[index + numColumns + 2], xSquareNative[index + 1], xSquareNative[index - 1]) / 4f;

					} else if (j == 0) {
						// Left Edge
						cellArea = GetPolygonArea(xSquareNative[index + numColumns + 1], xSquareNative[index + numColumns + 2], xSquareNative[index - numColumns], xSquareNative[index - numColumns - 1]) / 4f;

					} else if (i == numRows) {
						// Bottom Edge
						cellArea = GetPolygonArea(xSquareNative[index - 1], xSquareNative[index + 1], xSquareNative[index - numColumns], xSquareNative[index - numColumns - 2]) / 4f;

					} else if (j == numColumns) {
						// Right Edge
						cellArea = GetPolygonArea(xSquareNative[index + numColumns], xSquareNative[index + numColumns + 1], xSquareNative[index - numColumns - 1], xSquareNative[index - numColumns - 2]) / 4f;

					} else {
						// Inner Point
						cellArea = GetPolygonArea(xSquareNative[index + numColumns], xSquareNative[index + numColumns + 2], xSquareNative[index - numColumns], xSquareNative[index - numColumns - 2]) / 4f;

					}

					cellArea *= (1f + compression.x) * (1f + compression.y);
					w[ind] = wScale / (dispatcher.materials[(int)clothMaterial].surfaceDensity * cellArea);
					dragFactor[ind++] = 0.5f * cellArea * airDensity;
					totalArea += cellArea;

				}
			}

			mass = totalArea * dispatcher.materials[(int)clothMaterial].surfaceDensity;

			for (int i = 0; i < lod.pinIndices.Count; i++) {
				pinnedVertInvMass.Add(w[lod.pinIndices[i]]);
				w[lod.pinIndices[i]] = 0;
				pinnedVertLocalPos.Add(pinnedVertices[lod.pinListIndices[i]].vector);
			}

			InitConstraints(x);
		}

		rectIndToTriIndNative.Dispose();
		xSquareNative.Dispose();
	}

	private void InitConstraints(NativeArray<Vector3> pos) {
		neighbors = FindTriNeighbors(triangles);

		List<int>[] triPairs = new List<int>[] {
			new List<int>(), new List<int>(),  new List<int>(), new List<int>(),  new List<int>(), new List<int>()
		};
		List<int>[] edges = new List<int>[] {
			new List<int>(), new List<int>(),  new List<int>(), new List<int>(), new List<int>(), new List<int>()
		};

		int count = 0;
		int numRows = LODs[activeLODIndex].numRows;
		int numColumns = LODs[activeLODIndex].numColumns;

		if (clothShape == ClothShape.Quadrilateral) {

			for (int i = 0; i < numOneSidedTriangles; i++) {
				bool evenTri = i % 2 == 1; // Is the triangle a top-left triangle instead of bottom-right
				bool evenCol = ((i / 2) % numColumns) % 2 == 1; // Is the column the triangle is in an even one
				bool evenRow = (i / (numColumns * 2)) % 2 == 1; // Is the row the triangle is in an even one
				bool topRow = i < numColumns * 2;
				bool bottomRow = i > (numOneSidedTriangles - numColumns * 2);

				for (int j = 0; j < 3; j++) {
					int id0 = triangles[i * 3 + j];
					int id1 = triangles[i * 3 + (j + 1) % 3];

					// each edge only once
					int n = neighbors[i * 3 + j];
					if (n < 0 || id0 < id1) {
						//edges.Add(id0); // Creating edge constraints
						//edges.Add(id1);

						if (!evenTri) {
							if (j == 0) { // Upper Horizontal edge
								if (evenCol) {
									edges[0].Add(id0);
									edges[0].Add(id1);
								} else {
									edges[1].Add(id0);
									edges[1].Add(id1);
								}
							} else if (i % (numColumns * 2) == 0 && j == 2) { // First Column & Left Vertical Edge
								if (evenRow) {
									edges[2].Add(id0);
									edges[2].Add(id1);
								} else {
									edges[3].Add(id0);
									edges[3].Add(id1);
								}
							} else { // Diagonal Edge
								if (evenRow) {
									edges[4].Add(id0);
									edges[4].Add(id1);
								} else {
									edges[5].Add(id0);
									edges[5].Add(id1);
								}
							}

						} else {
							// Even (Lower-Right) Triangle
							// Always make Bottom and Right edges
							if (j == 0) {
								// Right Edge
								if (evenRow) {
									edges[2].Add(id0);
									edges[2].Add(id1);
								} else {
									edges[3].Add(id0);
									edges[3].Add(id1);
								}
							} else if (j == 1 && bottomRow) {
								// Bottom Edge & Bottom Row
								if (evenCol) {
									edges[0].Add(id0);
									edges[0].Add(id1);
								} else {
									edges[1].Add(id0);
									edges[1].Add(id1);
								}
							}
						}
					}


					if (i % (numColumns * 2) == numColumns * 2 - 1) count = 0; // Rightmost column

					// tri pair
					if (n >= 0 && id0 < id1) { // Creating bending constraints
						int ni = n / 3;
						int nj = n % 3;
						int id2 = triangles[i * 3 + (j + 2) % 3];
						int id3 = triangles[ni * 3 + (nj + 2) % 3];

						if (j == 1) {
							if (evenCol) {
								triPairs[0].Add(id2);
								triPairs[0].Add(id3);
							} else {
								triPairs[2].Add(id2);
								triPairs[2].Add(id3);
							}
						} else {
							if (topRow) {
								if (evenCol) {
									triPairs[1].Add(id2);
									triPairs[1].Add(id3);
								} else {
									triPairs[3].Add(id2);
									triPairs[3].Add(id3);
								}
							} else {
								if (count == 0) {
									if (evenCol) {
										triPairs[4].Add(id2);
										triPairs[4].Add(id3);
									} else {
										triPairs[5].Add(id2);
										triPairs[5].Add(id3);
									}
								} else {
									if (evenCol) {
										if (evenRow) {
											triPairs[3].Add(id2);
											triPairs[3].Add(id3);
										} else {
											triPairs[1].Add(id2);
											triPairs[1].Add(id3);
										}
									} else {
										if (evenRow) {
											triPairs[1].Add(id2);
											triPairs[1].Add(id3);
										} else {
											triPairs[3].Add(id2);
											triPairs[3].Add(id3);
										}
									}
								}
								count++;
								if (count > 1) count = 0;
							}
						}

					}
				}
			}
		} else if (clothShape == ClothShape.Trapezoidal) {

			int minSubdivision = (int)Mathf.Min(numRows, numColumns);
			int ind = 0;
			for (int i = 0; i < numRows * numColumns * 2; i++) {
				bool evenTri = i % 2 == 1; // Is the triangle a top-left triangle instead of bottom-right
				bool evenCol = ((i / 2) % numColumns) % 2 == 1; // Is the column the triangle is in an even one
				bool evenRow = (i / (numColumns * 2)) % 2 == 1; // Is the row the triangle is in an even one
				bool topRow = i < numColumns * 2;
				bool bottomRow = i > (numRows * numColumns * 2 - numColumns * 2);
				bool diagonalTri = i % (numColumns * 2) == minSubdivision - i / (numColumns * 2) * 2 + 1;

				if (i % (numColumns * 2) < (minSubdivision - i / (numColumns * 2)) * 2 - 1) {
					continue;
				}

				for (int j = 0; j < 3; j++) {
					int id0 = triangles[ind * 3 + j];
					int id1 = triangles[ind * 3 + (j + 1) % 3];

					// each edge only once
					int n = neighbors[ind * 3 + j];
					if (n < 0 || id0 < id1) {
						//edges.Add(id0); // Creating edge constraints
						//edges.Add(id1);

						if (!evenTri) {
							if (j == 0) { // Upper Horizontal edge
								if (evenCol) {
									edges[0].Add(id0);
									edges[0].Add(id1);
								} else {
									edges[1].Add(id0);
									edges[1].Add(id1);
								}
							} else if (i % (numColumns * 2) == 0 && j == 2) { // First Column & Left Vertical Edge
								if (evenRow) {
									edges[2].Add(id0);
									edges[2].Add(id1);
								} else {
									edges[3].Add(id0);
									edges[3].Add(id1);
								}
							} else { // Diagonal Edge
								if (evenRow) {
									edges[4].Add(id0);
									edges[4].Add(id1);
								} else {
									edges[5].Add(id0);
									edges[5].Add(id1);
								}
							}

						} else {
							// Even (Lower-Right) Triangle
							// Always make Bottom and Right edges
							if (j == 0) {
								// Right Edge
								if (evenRow) {
									edges[2].Add(id0);
									edges[2].Add(id1);
								} else {
									edges[3].Add(id0);
									edges[3].Add(id1);
								}
							} else if (j == 1 && bottomRow) {
								// Bottom Edge & Bottom Row
								if (evenCol) {
									edges[0].Add(id0);
									edges[0].Add(id1);
								} else {
									edges[1].Add(id0);
									edges[1].Add(id1);
								}
							} else if (diagonalTri) {
								if (evenRow) {
									edges[4].Add(id0);
									edges[4].Add(id1);
								} else {
									edges[5].Add(id0);
									edges[5].Add(id1);
								}
							}
						}
					}


					if (i % (numColumns * 2) == numColumns * 2 - 1) count = 0; // Rightmost column

					// tri pair
					if (n >= 0 && id0 < id1) { // Creating bending constraints
						int ni = n / 3;
						int nj = n % 3;
						int id2 = triangles[ind * 3 + (j + 2) % 3];
						int id3 = triangles[ni * 3 + (nj + 2) % 3];

						if (j == 1) {
							if (evenCol) {
								triPairs[0].Add(id2);
								triPairs[0].Add(id3);
							} else {
								triPairs[1].Add(id2);
								triPairs[1].Add(id3);
							}
						} else {
							if (topRow) {
								triPairs[2].Add(id2);
								triPairs[2].Add(id3);
							} else {
								if (count == 0) {
									if (i / (numColumns * 2) < minSubdivision) {
										if (evenRow) {
											triPairs[3].Add(id2);
											triPairs[3].Add(id3);
										} else {
											triPairs[2].Add(id2);
											triPairs[2].Add(id3);
										}
									} else if (evenCol) {
										if (evenRow) {
											triPairs[4].Add(id2);
											triPairs[4].Add(id3);
										} else {
											triPairs[5].Add(id2);
											triPairs[5].Add(id3);
										}
									} else {
										if (evenRow) {
											triPairs[5].Add(id2);
											triPairs[5].Add(id3);
										} else {
											triPairs[4].Add(id2);
											triPairs[4].Add(id3);
										}
									}
								} else {
									if (i / (numColumns * 2) >= minSubdivision) {
										if (evenRow) {
											triPairs[3].Add(id2);
											triPairs[3].Add(id3);
										} else {
											triPairs[2].Add(id2);
											triPairs[2].Add(id3);
										}
									} else if (evenCol) {
										if (evenRow) {
											triPairs[4].Add(id2);
											triPairs[4].Add(id3);
										} else {
											triPairs[5].Add(id2);
											triPairs[5].Add(id3);
										}
									} else {
										if (evenRow) {
											triPairs[5].Add(id2);
											triPairs[5].Add(id3);
										} else {
											triPairs[4].Add(id2);
											triPairs[4].Add(id3);
										}
									}
								}
								count++;
								if (count > 1) count = 0;
							}
						}

					}
				}

				ind++;
			}
		}

		stretchingIDs = new int[edges.Length][]; // Initialize the outer array

		for (int i = 0; i < edges.Length; i++) {
			if (edges[i] != null) {
				stretchingIDs[i] = edges[i].ToArray();
			} else {
				stretchingIDs[i] = new int[0];
			}
		}

		bendingIDs = new int[triPairs.Length][];

		for (int i = 0; i < triPairs.Length; i++) {
			if (triPairs[i] != null) {
				bendingIDs[i] = triPairs[i].ToArray();
			} else {
				bendingIDs[i] = new int[0];
			}
		}

		d0 = new float[stretchingIDs.Length][];
		for (int i = 0; i < d0.Length; i++) {
			d0[i] = new float[stretchingIDs[i].Length / 2];
			for (int j = 0; j < d0[i].Length; j++) {
				int id0 = stretchingIDs[i][2 * j];
				int id1 = stretchingIDs[i][2 * j + 1];

				float constraintLength = (new Vector3((pos[id0] - pos[id1]).x * (1 + compression.x), (pos[id0] - pos[id1]).y * (1 + compression.y), 0)).magnitude;

				d0[i][j] = constraintLength;
			}
		}

		int index = 0;
		dihedral0 = new float[bendingIDs.Length][];
		for (int i = 0; i < dihedral0.Length; i++) {
			dihedral0[i] = new float[bendingIDs[i].Length / 2];
			for (int j = 0; j < dihedral0[i].Length; j++) {
				int id0 = bendingIDs[i][2 * j];
				int id1 = bendingIDs[i][2 * j + 1];

				float constraintLength = (new Vector3((pos[id0] - pos[id1]).x * (1 + compression.x), (pos[id0] - pos[id1]).y * (1 + compression.y), 0)).magnitude;

				System.Random rand = new System.Random(seed + index);
				index++;

				float wrinklinessFactor = Mathf.Lerp(1f, (float)rand.NextDouble() * 0.05f + 0.95f, wrinkliness);

				dihedral0[i][j] = constraintLength * wrinklinessFactor;
			}
		}
	}

	private void InitConstraints(NativeArray<Vector2> pos) {
		neighbors = FindTriNeighbors(triangles);

		List<int>[] triPairs = new List<int>[] {
			new List<int>(), new List<int>(),  new List<int>(), new List<int>(),  new List<int>(), new List<int>()
		};
		List<int>[] edges = new List<int>[] {
			new List<int>(), new List<int>(),  new List<int>(), new List<int>(), new List<int>(), new List<int>()
		};

		int count = 0;
		int numRows = LODs[activeLODIndex].numRows;
		int numColumns = LODs[activeLODIndex].numColumns;

		if (clothShape == ClothShape.Quadrilateral) {

			for (int i = 0; i < numOneSidedTriangles; i++) {
				bool evenTri = i % 2 == 1; // Is the triangle a top-left triangle instead of bottom-right
				bool evenCol = ((i / 2) % numColumns) % 2 == 1; // Is the column the triangle is in an even one
				bool evenRow = (i / (numColumns * 2)) % 2 == 1; // Is the row the triangle is in an even one
				bool topRow = i < numColumns * 2;
				bool bottomRow = i > (numOneSidedTriangles - numColumns * 2);

				for (int j = 0; j < 3; j++) {
					int id0 = triangles[i * 3 + j];
					int id1 = triangles[i * 3 + (j + 1) % 3];

					// each edge only once
					int n = neighbors[i * 3 + j];
					if (n < 0 || id0 < id1) {
						//edges.Add(id0); // Creating edge constraints
						//edges.Add(id1);

						if (!evenTri) {
							if (j == 0) { // Upper Horizontal edge
								if (evenCol) {
									edges[0].Add(id0);
									edges[0].Add(id1);
								} else {
									edges[1].Add(id0);
									edges[1].Add(id1);
								}
							} else if (i % (numColumns * 2) == 0 && j == 2) { // First Column & Left Vertical Edge
								if (evenRow) {
									edges[2].Add(id0);
									edges[2].Add(id1);
								} else {
									edges[3].Add(id0);
									edges[3].Add(id1);
								}
							} else { // Diagonal Edge
								if (evenRow) {
									edges[4].Add(id0);
									edges[4].Add(id1);
								} else {
									edges[5].Add(id0);
									edges[5].Add(id1);
								}
							}

						} else {
							// Even (Lower-Right) Triangle
							// Always make Bottom and Right edges
							if (j == 0) {
								// Right Edge
								if (evenRow) {
									edges[2].Add(id0);
									edges[2].Add(id1);
								} else {
									edges[3].Add(id0);
									edges[3].Add(id1);
								}
							} else if (j == 1 && bottomRow) {
								// Bottom Edge & Bottom Row
								if (evenCol) {
									edges[0].Add(id0);
									edges[0].Add(id1);
								} else {
									edges[1].Add(id0);
									edges[1].Add(id1);
								}
							}
						}
					}


					if (i % (numColumns * 2) == numColumns * 2 - 1) count = 0; // Rightmost column

					// tri pair
					if (n >= 0 && id0 < id1) { // Creating bending constraints
						int ni = n / 3;
						int nj = n % 3;
						int id2 = triangles[i * 3 + (j + 2) % 3];
						int id3 = triangles[ni * 3 + (nj + 2) % 3];

						if (j == 1) {
							if (evenCol) {
								triPairs[0].Add(id2);
								triPairs[0].Add(id3);
							} else {
								triPairs[2].Add(id2);
								triPairs[2].Add(id3);
							}
						} else {
							if (topRow) {
								if (evenCol) {
									triPairs[1].Add(id2);
									triPairs[1].Add(id3);
								} else {
									triPairs[3].Add(id2);
									triPairs[3].Add(id3);
								}
							} else {
								if (count == 0) {
									if (evenCol) {
										triPairs[4].Add(id2);
										triPairs[4].Add(id3);
									} else {
										triPairs[5].Add(id2);
										triPairs[5].Add(id3);
									}
								} else {
									if (evenCol) {
										if (evenRow) {
											triPairs[3].Add(id2);
											triPairs[3].Add(id3);
										} else {
											triPairs[1].Add(id2);
											triPairs[1].Add(id3);
										}
									} else {
										if (evenRow) {
											triPairs[1].Add(id2);
											triPairs[1].Add(id3);
										} else {
											triPairs[3].Add(id2);
											triPairs[3].Add(id3);
										}
									}
								}
								count++;
								if (count > 1) count = 0;
							}
						}

					}
				}
			}
		} else if (clothShape == ClothShape.Trapezoidal) {

			int minSubdivision = (int)Mathf.Min(numRows, numColumns);
			int ind = 0;
			for (int i = 0; i < numRows * numColumns * 2; i++) {
				bool evenTri = i % 2 == 1; // Is the triangle a top-left triangle instead of bottom-right
				bool evenCol = ((i / 2) % numColumns) % 2 == 1; // Is the column the triangle is in an even one
				bool evenRow = (i / (numColumns * 2)) % 2 == 1; // Is the row the triangle is in an even one
				bool topRow = i < numColumns * 2;
				bool bottomRow = i > (numRows * numColumns * 2 - numColumns * 2);
				bool diagonalTri = i % (numColumns * 2) == minSubdivision - i / (numColumns * 2) * 2 + 1;

				if (i % (numColumns * 2) < (minSubdivision - i / (numColumns * 2)) * 2 - 1) {
					continue;
				}

				for (int j = 0; j < 3; j++) {
					int id0 = triangles[ind * 3 + j];
					int id1 = triangles[ind * 3 + (j + 1) % 3];

					// each edge only once
					int n = neighbors[ind * 3 + j];
					if (n < 0 || id0 < id1) {
						//edges.Add(id0); // Creating edge constraints
						//edges.Add(id1);

						if (!evenTri) {
							if (j == 0) { // Upper Horizontal edge
								if (evenCol) {
									edges[0].Add(id0);
									edges[0].Add(id1);
								} else {
									edges[1].Add(id0);
									edges[1].Add(id1);
								}
							} else if (i % (numColumns * 2) == 0 && j == 2) { // First Column & Left Vertical Edge
								if (evenRow) {
									edges[2].Add(id0);
									edges[2].Add(id1);
								} else {
									edges[3].Add(id0);
									edges[3].Add(id1);
								}
							} else { // Diagonal Edge
								if (evenRow) {
									edges[4].Add(id0);
									edges[4].Add(id1);
								} else {
									edges[5].Add(id0);
									edges[5].Add(id1);
								}
							}

						} else {
							// Even (Lower-Right) Triangle
							// Always make Bottom and Right edges
							if (j == 0) {
								// Right Edge
								if (evenRow) {
									edges[2].Add(id0);
									edges[2].Add(id1);
								} else {
									edges[3].Add(id0);
									edges[3].Add(id1);
								}
							} else if (j == 1 && bottomRow) {
								// Bottom Edge & Bottom Row
								if (evenCol) {
									edges[0].Add(id0);
									edges[0].Add(id1);
								} else {
									edges[1].Add(id0);
									edges[1].Add(id1);
								}
							} else if (diagonalTri) {
								if (evenRow) {
									edges[4].Add(id0);
									edges[4].Add(id1);
								} else {
									edges[5].Add(id0);
									edges[5].Add(id1);
								}
							}
						}
					}


					if (i % (numColumns * 2) == numColumns * 2 - 1) count = 0; // Rightmost column

					// tri pair
					if (n >= 0 && id0 < id1) { // Creating bending constraints
						int ni = n / 3;
						int nj = n % 3;
						int id2 = triangles[ind * 3 + (j + 2) % 3];
						int id3 = triangles[ni * 3 + (nj + 2) % 3];

						if (j == 1) {
							if (evenCol) {
								triPairs[0].Add(id2);
								triPairs[0].Add(id3);
							} else {
								triPairs[1].Add(id2);
								triPairs[1].Add(id3);
							}
						} else {
							if (topRow) {
								triPairs[2].Add(id2);
								triPairs[2].Add(id3);
							} else {
								if (count == 0) {
									if (i / (numColumns * 2) < minSubdivision) {
										if (evenRow) {
											triPairs[3].Add(id2);
											triPairs[3].Add(id3);
										} else {
											triPairs[2].Add(id2);
											triPairs[2].Add(id3);
										}
									} else if (evenCol) {
										if (evenRow) {
											triPairs[4].Add(id2);
											triPairs[4].Add(id3);
										} else {
											triPairs[5].Add(id2);
											triPairs[5].Add(id3);
										}
									} else {
										if (evenRow) {
											triPairs[5].Add(id2);
											triPairs[5].Add(id3);
										} else {
											triPairs[4].Add(id2);
											triPairs[4].Add(id3);
										}
									}
								} else {
									if (i / (numColumns * 2) >= minSubdivision) {
										if (evenRow) {
											triPairs[3].Add(id2);
											triPairs[3].Add(id3);
										} else {
											triPairs[2].Add(id2);
											triPairs[2].Add(id3);
										}
									} else if (evenCol) {
										if (evenRow) {
											triPairs[4].Add(id2);
											triPairs[4].Add(id3);
										} else {
											triPairs[5].Add(id2);
											triPairs[5].Add(id3);
										}
									} else {
										if (evenRow) {
											triPairs[5].Add(id2);
											triPairs[5].Add(id3);
										} else {
											triPairs[4].Add(id2);
											triPairs[4].Add(id3);
										}
									}
								}
								count++;
								if (count > 1) count = 0;
							}
						}

					}
				}

				ind++;
			}
		}

		stretchingIDs = new int[edges.Length][]; // Initialize the outer array

		for (int i = 0; i < edges.Length; i++) {
			if (edges[i] != null) {
				stretchingIDs[i] = edges[i].ToArray();
			} else {
				stretchingIDs[i] = new int[0];
			}
		}

		bendingIDs = new int[triPairs.Length][];

		for (int i = 0; i < triPairs.Length; i++) {
			if (triPairs[i] != null) {
				bendingIDs[i] = triPairs[i].ToArray();
			} else {
				bendingIDs[i] = new int[0];
			}
		}

		d0 = new float[stretchingIDs.Length][];
		for (int i = 0; i < d0.Length; i++) {
			d0[i] = new float[stretchingIDs[i].Length / 2];
			for (int j = 0; j < d0[i].Length; j++) {
				int id0 = stretchingIDs[i][2 * j];
				int id1 = stretchingIDs[i][2 * j + 1];

				float constraintLength = (new Vector3((pos[id0] - pos[id1]).x * (1 + compression.x), (pos[id0] - pos[id1]).y * (1 + compression.y), 0)).magnitude;

				d0[i][j] = constraintLength;
			}
		}

		int index = 0;
		dihedral0 = new float[bendingIDs.Length][];
		for (int i = 0; i < dihedral0.Length; i++) {
			dihedral0[i] = new float[bendingIDs[i].Length / 2];
			for (int j = 0; j < dihedral0[i].Length; j++) {
				int id0 = bendingIDs[i][2 * j];
				int id1 = bendingIDs[i][2 * j + 1];

				float constraintLength = (new Vector3((pos[id0] - pos[id1]).x * (1 + compression.x), (pos[id0] - pos[id1]).y * (1 + compression.y), 0)).magnitude;

				System.Random rand = new System.Random(seed + index);
				index++;

				float wrinklinessFactor = Mathf.Lerp(1f, (float)rand.NextDouble() * 0.05f + 0.95f, wrinkliness);

				dihedral0[i][j] = constraintLength * wrinklinessFactor;
			}
		}
	}

	private int[] FindTriNeighbors(int[] triangles) {

		List<int[]> edges = new List<int[]>();

		for (int i = 0; i < numOneSidedTriangles; i++) {
			for (int j = 0; j < 3; j++) {
				int id0 = triangles[i * 3 + j];
				int id1 = triangles[i * 3 + (j + 1) % 3];
				edges.Add(new int[] { Mathf.Min(id0, id1), Mathf.Max(id0, id1), i * 3 + j });
			}
		}

		edges.Sort((a, b) => {
			if (a[0] != b[0])
				return a[0].CompareTo(b[0]); // Compare by the first number
			return a[1].CompareTo(b[1]);     // Compare by the second number if the first numbers are equal
		});

		int[] neighbors = new int[3 * numOneSidedTriangles];
		for (int i = 0; i < neighbors.Length; i++) {
			neighbors[i] = -1;
		}

		int nr = 0;
		while (nr < edges.Count) {
			int[] e0 = edges[nr];
			nr++;
			if (nr < edges.Count) {
				int[] e1 = edges[nr];
				if (e0[0] == e1[0] && e0[1] == e1[1]) {
					neighbors[e0[2]] = e1[2];
					neighbors[e1[2]] = e0[2];
				}
			}
		}

		edges.Clear();

		return neighbors;
	}

	public async void ChangeLOD(int lodIndex) {
		if (lodIndex == activeLODIndex) return;

		pinnedVertInvMass.Clear();
		pinnedVertLocalPos.Clear();

		if (lodIndex > activeLODIndex) {
			await Task.Run(() => IncreaseLOD(lodIndex));
		} else {
			await Task.Run(() => DecreaseLOD(lodIndex));
		}

		isChangingLOD = false;
	}

	private void IncreaseLOD(int lodIndex) {
		if (lodIndex > LODs.Count - 1) lodIndex = LODs.Count - 1;

		UpdatePinnedVerticesOnLOD(lodIndex);

		ClothLOD lod = LODs[lodIndex];

		int numRows = lod.numRows;
		int numColumns = lod.numColumns;
		int minSubdivision = (int)Mathf.Min(lod.numRows, lod.numColumns);
		int minSubdivisionPrev = (int)Mathf.Min(LODs[activeLODIndex].numRows, LODs[activeLODIndex].numColumns);

		// Calculate the number of vertices
		numOneSidedVerts = (numRows + 1) * (numColumns + 1);

		NativeArray<int> rectIndToTriIndNative = new NativeArray<int>(numOneSidedVerts, Allocator.Persistent);
		NativeArray<Vector2> xSquareNative = new NativeArray<Vector2>(numOneSidedVerts, Allocator.Persistent);

		if (clothShape == ClothShape.Trapezoidal) {
			numOneSidedVerts -= minSubdivision * (minSubdivision + 1) / 2;
		}

		NativeArray<Vector2> untransformedX = new NativeArray<Vector2>(numOneSidedVerts, Allocator.Persistent);
		NativeArray<Vector3> newX = new NativeArray<Vector3>(numOneSidedVerts, Allocator.Persistent);
		NativeArray<Vector3> newV = new NativeArray<Vector3>(numOneSidedVerts, Allocator.Persistent);
		NativeArray<Vector3> newNormals = new NativeArray<Vector3>(numOneSidedVerts, Allocator.Persistent);
		NativeArray<Vector2> newUV = new NativeArray<Vector2>(numOneSidedVerts, Allocator.Persistent);
		NativeArray<Vector2> newTextureUV = new NativeArray<Vector2>(numOneSidedVerts, Allocator.Persistent);

		if (isDoubleSided) {

			if (clothShape == ClothShape.Quadrilateral) {
				triangles = new int[numRows * numColumns * 12];
			} else if (clothShape == ClothShape.Trapezoidal) {
				triangles = new int[numRows * numColumns * 12 - minSubdivision * minSubdivision * 6];
			}

			numOneSidedTriangles = triangles.Length / 6;

		} else {

			if (clothShape == ClothShape.Quadrilateral) {
				triangles = new int[numRows * numColumns * 6];
			} else if (clothShape == ClothShape.Trapezoidal) {
				triangles = new int[numRows * numColumns * 6 - minSubdivision * minSubdivision * 3];
			}

			numOneSidedTriangles = triangles.Length / 3;
		}

		w = new float[numOneSidedVerts];
		dragFactor = new float[numOneSidedVerts];

		int every = (int)Mathf.Pow(2, lod.subdivisions - LODs[activeLODIndex].subdivisions);

		int index = 0;
		for (int i = 0; i < numRows + 1; i++) {

			float rowWidth = (bottomWidth - topWidth) / numRows * i + topWidth;
			float rowOffset = Mathf.Lerp(topCenter - topWidth / 2f, bottomCenter - bottomWidth / 2f, (float)i / numRows);

			float semiAxisA = rowWidth / (numColumns * 2f);

			for (int j = 0; j < numColumns + 1; j++) {

				float colHeight = (rightHeight - leftHeight) / numColumns * j + leftHeight;
				float colOffset = Mathf.Lerp(leftCenter - leftHeight / 2f, rightCenter - rightHeight / 2f, (float)j / numColumns);

				float xPos = (j * rowWidth) / numColumns + rowOffset;
				float yPos = colHeight - (i * colHeight) / numRows + colOffset;

				if (randomness > 0 && j > (clothShape == ClothShape.Trapezoidal ? minSubdivision - i : 0) && j < numColumns && i > 0 && i < numRows && !lod.pinIndices.Contains(index)) {
					float semiAxisB = colHeight / (numRows * 2f);

					int perVertexSeed = seed + index * (int)Mathf.Pow(2, maxSubdivisions) / (int)Mathf.Pow(2, lod.subdivisions);
					System.Random rand = new System.Random(perVertexSeed);

					float randAngle = (float)rand.NextDouble() * 2f * Mathf.PI;
					float randDist = (float)rand.NextDouble() * 0.6f * randomness;
					Vector3 adjustment = new Vector3(semiAxisA * Mathf.Cos(randAngle), semiAxisB * Mathf.Sin(randAngle), 0) * randDist;

					xPos += adjustment.x;
					yPos += adjustment.y;
				}

				//int adjustedJ = j / every;
				//int adjustedI = i / every;
				int adjustedIndex = (j / every) + (i / every) * (LODs[activeLODIndex].numColumns + 1);

				if (clothShape == ClothShape.Quadrilateral) {

					untransformedX[index] = new Vector2(xPos, yPos);

					if (j % every == 0 && i % every == 0) {
						// Point is on previous LOD mesh
						newX[index] = x[adjustedIndex];
						newV[index] = v[adjustedIndex];
						newNormals[index] = normals[adjustedIndex];
						newUV[index] = uv[adjustedIndex];
						newTextureUV[index] = textureUV[adjustedIndex];
					} else if (i % every == 0) {
						// On previously existing horizontal

						int id0 = adjustedIndex;
						int id1 = id0 + 1;

						float interpolant = j % every / (float)every;

						newX[index] = Vector3.Lerp(x[id0], x[id1], interpolant);
						newV[index] = Vector3.Lerp(v[id0], v[id1], interpolant);
						newNormals[index] = Vector3.Lerp(normals[id0], normals[id1], interpolant);
						newUV[index] = Vector3.Lerp(uv[id0], uv[id1], interpolant);
						newTextureUV[index] = Vector3.Lerp(textureUV[id0], textureUV[id1], interpolant);
					} else if (j % every == 0) {
						// On previously existing vertical

						int id0 = adjustedIndex;
						int id1 = id0 + LODs[activeLODIndex].numColumns + 1;

						float interpolant = i % every / (float)every;

						newX[index] = Vector3.Lerp(x[id0], x[id1], interpolant);
						newV[index] = Vector3.Lerp(v[id0], v[id1], interpolant);
						newNormals[index] = Vector3.Lerp(normals[id0], normals[id1], interpolant);
						newUV[index] = Vector3.Lerp(uv[id0], uv[id1], interpolant);
						newTextureUV[index] = Vector3.Lerp(textureUV[id0], textureUV[id1], interpolant);
					} else {
						// Point is on plane defined by surrounding 4 previous points

						int id0 = adjustedIndex;
						int id1 = id0 + 1;
						int id2 = id0 + LODs[activeLODIndex].numColumns + 1;
						int id3 = id2 + 1;

						float u = j % every / (float)every;
						float v = i % every / (float)every;

						newX[index] = Vector3.Lerp(Vector3.Lerp(x[id0], x[id1], u), Vector3.Lerp(x[id2], x[id3], u), v);
						newV[index] = Vector3.Lerp(Vector3.Lerp(this.v[id0], this.v[id1], u), Vector3.Lerp(this.v[id2], this.v[id3], u), v);
						newNormals[index] = Vector3.Lerp(Vector3.Lerp(normals[id0], normals[id1], u), Vector3.Lerp(normals[id2], normals[id3], u), v);
						newUV[index] = Vector3.Lerp(Vector3.Lerp(uv[id0], uv[id1], u), Vector3.Lerp(uv[id2], uv[id3], u), v);
						newTextureUV[index] = Vector3.Lerp(Vector3.Lerp(textureUV[id0], textureUV[id1], u), Vector3.Lerp(textureUV[id2], textureUV[id3], u), v);
					}

					index++;
				} else if (clothShape == ClothShape.Trapezoidal) {

					xSquareNative[i * (numColumns + 1) + j] = new Vector2(xPos, yPos);

					if (j >= minSubdivision - i) {

						untransformedX[index] = new Vector2(xPos, yPos);
						rectIndToTriIndNative[i * (numColumns + 1) + j] = index;

						int minSubdivisionMinAdjustedRowIndex = minSubdivisionPrev - i / every;
						adjustedIndex -= minSubdivisionPrev * (minSubdivisionPrev + 1) / 2;

						if (minSubdivisionMinAdjustedRowIndex - 1 > 0) {
							adjustedIndex += (minSubdivisionMinAdjustedRowIndex - 1) * (minSubdivisionMinAdjustedRowIndex) / 2;
						}

						if (j % every == 0 && i % every == 0) {
							// Point is on previous LOD mesh
							newX[index] = x[adjustedIndex];
							newV[index] = v[adjustedIndex];
							newNormals[index] = normals[adjustedIndex];
							newUV[index] = uv[adjustedIndex];
							newTextureUV[index] = textureUV[adjustedIndex];
						} else if (i % every == 0) {
							// On previously existing horizontal

							int id0 = adjustedIndex;
							int id1 = id0 + 1;

							float interpolant = j % every / (float)every;

							newX[index] = Vector3.Lerp(x[id0], x[id1], interpolant);
							newV[index] = Vector3.Lerp(v[id0], v[id1], interpolant);
							newNormals[index] = Vector3.Lerp(normals[id0], normals[id1], interpolant);
							newUV[index] = Vector3.Lerp(uv[id0], uv[id1], interpolant);
							newTextureUV[index] = Vector3.Lerp(textureUV[id0], textureUV[id1], interpolant);
						} else if (j % every == 0) {
							// On previously existing vertical

							int id0 = adjustedIndex;

							int colToRowDiff = LODs[activeLODIndex].numColumns - LODs[activeLODIndex].numRows;
							int numOnNextRow = (colToRowDiff > 0 ? colToRowDiff : 0) + i / every + 2;
							numOnNextRow = numOnNextRow > LODs[activeLODIndex].numColumns + 1 ? LODs[activeLODIndex].numColumns + 1 : numOnNextRow;

							int id1 = id0 + numOnNextRow;

							float interpolant = i % every / (float)every;

							newX[index] = Vector3.Lerp(x[id0], x[id1], interpolant);
							newV[index] = Vector3.Lerp(v[id0], v[id1], interpolant);
							newNormals[index] = Vector3.Lerp(normals[id0], normals[id1], interpolant);
							newUV[index] = Vector3.Lerp(uv[id0], uv[id1], interpolant);
							newTextureUV[index] = Vector3.Lerp(textureUV[id0], textureUV[id1], interpolant);
						} else {
							// Point is on plane defined by surrounding previous points

							float u = j % every / (float)every;
							float v = i % every / (float)every;

							int id1 = adjustedIndex + 1;

							int colToRowDiff = LODs[activeLODIndex].numColumns - LODs[activeLODIndex].numRows;
							int numOnNextRow = (colToRowDiff > 0 ? colToRowDiff : 0) + i / every + 2;
							numOnNextRow = numOnNextRow > LODs[activeLODIndex].numColumns + 1 ? LODs[activeLODIndex].numColumns + 1 : numOnNextRow;

							int id2 = adjustedIndex + numOnNextRow;
							int id3 = id2 + 1;

							if (j <= minSubdivision - i + every) {

								newX[index] = Vector3.Lerp(Vector3.Lerp(x[id2] + (x[id1] - x[id3]), x[id1], u), Vector3.Lerp(x[id2], x[id3], u), v);
								newV[index] = Vector3.Lerp(Vector3.Lerp(this.v[id2] + (this.v[id1] - this.v[id3]), this.v[id1], u), Vector3.Lerp(this.v[id2], this.v[id3], u), v);
								newNormals[index] = Vector3.Lerp(Vector3.Lerp(normals[id2] + (normals[id1] - normals[id3]), normals[id1], u), Vector3.Lerp(normals[id2], normals[id3], u), v);
								newUV[index] = Vector3.Lerp(Vector3.Lerp(uv[id2] + (uv[id1] - uv[id3]), uv[id1], u), Vector3.Lerp(uv[id2], uv[id3], u), v);
								newTextureUV[index] = Vector3.Lerp(Vector3.Lerp(textureUV[id2] + (textureUV[id1] - textureUV[id3]), textureUV[id1], u), Vector3.Lerp(textureUV[id2], textureUV[id3], u), v);

							} else {
								int id0 = adjustedIndex;

								newX[index] = Vector3.Lerp(Vector3.Lerp(x[id0], x[id1], u), Vector3.Lerp(x[id2], x[id3], u), v);
								newV[index] = Vector3.Lerp(Vector3.Lerp(this.v[id0], this.v[id1], u), Vector3.Lerp(this.v[id2], this.v[id3], u), v);
								newNormals[index] = Vector3.Lerp(Vector3.Lerp(normals[id0], normals[id1], u), Vector3.Lerp(normals[id2], normals[id3], u), v);
								newUV[index] = Vector3.Lerp(Vector3.Lerp(uv[id0], uv[id1], u), Vector3.Lerp(uv[id2], uv[id3], u), v);
								newTextureUV[index] = Vector3.Lerp(Vector3.Lerp(textureUV[id0], textureUV[id1], u), Vector3.Lerp(textureUV[id2], textureUV[id3], u), v);
							}
						}

						index++;
					}
				}
			}
		}
		
		activeLODIndex = lodIndex;

		if (x.IsCreated) x.Dispose();
		if (v.IsCreated) v.Dispose();
		if (normals.IsCreated) normals.Dispose();
		if (uv.IsCreated) uv.Dispose();
		if (textureUV.IsCreated) textureUV.Dispose();

		x = new NativeArray<Vector3>(numOneSidedVerts, Allocator.Persistent);
		v = new NativeArray<Vector3>(numOneSidedVerts, Allocator.Persistent);
		normals = new NativeArray<Vector3>(numOneSidedVerts, Allocator.Persistent);
		uv = new NativeArray<Vector2>(numOneSidedVerts, Allocator.Persistent);
		textureUV = new NativeArray<Vector2>(numOneSidedVerts, Allocator.Persistent);

		unsafe {

			int size = Marshal.SizeOf(typeof(Vector3)) * numOneSidedVerts;
			IntPtr sourcePtr = (IntPtr)((Vector3*)newX.GetUnsafePtr());
			IntPtr destPtr = (IntPtr)((Vector3*)x.GetUnsafePtr());
			UnsafeUtility.MemCpy((void*)destPtr, (void*)sourcePtr, size);

			sourcePtr = (IntPtr)((Vector3*)newV.GetUnsafePtr());
			destPtr = (IntPtr)((Vector3*)v.GetUnsafePtr());
			UnsafeUtility.MemCpy((void*)destPtr, (void*)sourcePtr, size);

			sourcePtr = (IntPtr)((Vector3*)newNormals.GetUnsafePtr());
			destPtr = (IntPtr)((Vector3*)normals.GetUnsafePtr());
			UnsafeUtility.MemCpy((void*)destPtr, (void*)sourcePtr, size);

			size = Marshal.SizeOf(typeof(Vector2)) * numOneSidedVerts;
			sourcePtr = (IntPtr)((Vector2*)newUV.GetUnsafePtr());
			destPtr = (IntPtr)((Vector2*)uv.GetUnsafePtr());
			UnsafeUtility.MemCpy((void*)destPtr, (void*)sourcePtr, size);

			sourcePtr = (IntPtr)((Vector2*)newTextureUV.GetUnsafePtr());
			destPtr = (IntPtr)((Vector2*)textureUV.GetUnsafePtr());
			UnsafeUtility.MemCpy((void*)destPtr, (void*)sourcePtr, size);

		}

		newX.Dispose();
		newV.Dispose();
		newNormals.Dispose();
		newUV.Dispose();
		newTextureUV.Dispose();

		// Create arrays to store triangle indices
		vertexToTriangles = new List<int>[numOneSidedVerts];
		for (int i = 0; i < vertexToTriangles.Length; i++) {
			vertexToTriangles[i] = new List<int>(4);
		}
		int triangleIndex = 0;

		// Generate triangle indices
		for (int i = 0; i < numRows; i++) {
			for (int j = 0; j < numColumns; j++) {
				int realTriIndex = triangleIndex / 3;

				if (clothShape == ClothShape.Quadrilateral) {
					int id0 = i * (numColumns + 1) + j;
					int id1 = id0 + 1;
					int id2 = (i + 1) * (numColumns + 1) + j;
					int id3 = id2 + 1;

					vertexToTriangles[id0].Add(realTriIndex);
					vertexToTriangles[id1].Add(realTriIndex);
					vertexToTriangles[id2].Add(realTriIndex);
					triangles[triangleIndex++] = id0;
					triangles[triangleIndex++] = id1;
					triangles[triangleIndex++] = id2;

					vertexToTriangles[id1].Add(realTriIndex);
					vertexToTriangles[id3].Add(realTriIndex);
					vertexToTriangles[id2].Add(realTriIndex);
					triangles[triangleIndex++] = id1;
					triangles[triangleIndex++] = id3;
					triangles[triangleIndex++] = id2;

					if (isDoubleSided) {
						triangles[triangles.Length / 2 + triangleIndex - 6] = id0;
						triangles[triangles.Length / 2 + triangleIndex - 5] = id2;
						triangles[triangles.Length / 2 + triangleIndex - 4] = id1;

						triangles[triangles.Length / 2 + triangleIndex - 3] = id1;
						triangles[triangles.Length / 2 + triangleIndex - 2] = id2;
						triangles[triangles.Length / 2 + triangleIndex - 1] = id3;
					}
				} else if (clothShape == ClothShape.Trapezoidal) {
					if (j < minSubdivision - i - 1) continue;

					int id0 = i * (numColumns + 1) + j;
					int id1 = id0 + 1;
					int id2 = (i + 1) * (numColumns + 1) + j;
					int id3 = id2 + 1;

					if (j > minSubdivision - i - 1) {
						vertexToTriangles[rectIndToTriIndNative[id0]].Add(realTriIndex);
						vertexToTriangles[rectIndToTriIndNative[id1]].Add(realTriIndex);
						vertexToTriangles[rectIndToTriIndNative[id2]].Add(realTriIndex);
						triangles[triangleIndex++] = rectIndToTriIndNative[id0];
						triangles[triangleIndex++] = rectIndToTriIndNative[id1];
						triangles[triangleIndex++] = rectIndToTriIndNative[id2];
					}

					vertexToTriangles[rectIndToTriIndNative[id1]].Add(realTriIndex);
					vertexToTriangles[rectIndToTriIndNative[id3]].Add(realTriIndex);
					vertexToTriangles[rectIndToTriIndNative[id2]].Add(realTriIndex);
					triangles[triangleIndex++] = rectIndToTriIndNative[id1];
					triangles[triangleIndex++] = rectIndToTriIndNative[id3];
					triangles[triangleIndex++] = rectIndToTriIndNative[id2];

					if (isDoubleSided) {
						if (j == minSubdivision - i - 1) {

							triangles[triangles.Length / 2 + triangleIndex - 3] = rectIndToTriIndNative[id1];
							triangles[triangles.Length / 2 + triangleIndex - 2] = rectIndToTriIndNative[id2];
							triangles[triangles.Length / 2 + triangleIndex - 1] = rectIndToTriIndNative[id3];
						} else {

							triangles[triangles.Length / 2 + triangleIndex - 6] = rectIndToTriIndNative[id0];
							triangles[triangles.Length / 2 + triangleIndex - 5] = rectIndToTriIndNative[id2];
							triangles[triangles.Length / 2 + triangleIndex - 4] = rectIndToTriIndNative[id1];

							triangles[triangles.Length / 2 + triangleIndex - 3] = rectIndToTriIndNative[id1];
							triangles[triangles.Length / 2 + triangleIndex - 2] = rectIndToTriIndNative[id2];
							triangles[triangles.Length / 2 + triangleIndex - 1] = rectIndToTriIndNative[id3];
						}
					}
				}
			}
		}

		index = 0;
		if (clothShape == ClothShape.Quadrilateral) {
			for (int i = 0; i < numRows + 1; i++) {

				for (int j = 0; j < numColumns + 1; j++) {
					float cellArea = 0;

					if (i == 0 && j == 0) {
						// Top Left
						cellArea = GetPolygonArea(untransformedX[numColumns + 1], untransformedX[numColumns + 2], untransformedX[1], untransformedX[0]) / 4f;

					} else if (i == numRows && j == 0) {
						// Bottom Left
						cellArea = GetPolygonArea(untransformedX[numRows * (numColumns + 1)], untransformedX[numRows * (numColumns + 1) + 1], untransformedX[(numRows - 1) * (numColumns + 1) + 1], untransformedX[(numRows - 1) * (numColumns + 1)]) / 4f;

					} else if (i == numRows && j == numColumns) {
						// Bottom Right
						cellArea = GetPolygonArea(untransformedX[numOneSidedVerts - 2], untransformedX[numOneSidedVerts - 1], untransformedX[numRows * (numColumns + 1) - 1], untransformedX[numRows * (numColumns + 1) - 2]) / 4f;

					} else if (i == 0 && j == numColumns) {
						// Top Right
						cellArea = GetPolygonArea(untransformedX[numColumns * 2], untransformedX[numColumns + 1 + numColumns], untransformedX[numColumns], untransformedX[numColumns - 1]) / 4f;

					} else if (i == 0) {
						// Top Edge
						cellArea = GetPolygonArea(untransformedX[index + numColumns], untransformedX[index + numColumns + 2], untransformedX[index + 1], untransformedX[index - 1]) / 4f;

					} else if (j == 0) {
						// Left Edge
						cellArea = GetPolygonArea(untransformedX[index + numColumns + 1], untransformedX[index + numColumns + 2], untransformedX[index - numColumns], untransformedX[index - numColumns - 1]) / 4f;

					} else if (i == numRows) {
						// Bottom Edge
						cellArea = GetPolygonArea(untransformedX[index - 1], untransformedX[index + 1], untransformedX[index - numColumns], untransformedX[index - numColumns - 2]) / 4f;

					} else if (j == numColumns) {
						// Right Edge
						cellArea = GetPolygonArea(untransformedX[index + numColumns], untransformedX[index + numColumns + 1], untransformedX[index - numColumns - 1], untransformedX[index - numColumns - 2]) / 4f;

					} else {
						// Inner Point
						cellArea = GetPolygonArea(untransformedX[index + numColumns], untransformedX[index + numColumns + 2], untransformedX[index - numColumns], untransformedX[index - numColumns - 2]) / 4f;

					}

					cellArea *= (1f + compression.x) * (1f + compression.y);
					w[index] = wScale / (dispatcher.materials[(int)clothMaterial].surfaceDensity * cellArea);
					dragFactor[index] = 0.5f * cellArea * airDensity;
					index++;
				}
			}
		} else if (clothShape == ClothShape.Trapezoidal) {
			for (int i = 0; i < numRows + 1; i++) {

				for (int j = 0; j < numColumns + 1; j++) {
					int squareIndex = j + i * (numColumns + 1);
					float cellArea = 0;

					if (j < minSubdivision - i) continue;

					if (j == minSubdivision - i) {
						// Top Left
						if (i == 0 && j == numColumns) {
							// Top Right
							cellArea = GetPolygonArea(xSquareNative[numColumns * 2], xSquareNative[numColumns * 2 + 1], xSquareNative[numColumns], xSquareNative[numColumns - 1]) / 8f;

						} else if (i == numRows && j == 0) {
							// Bottom Left
							cellArea = GetPolygonArea(xSquareNative[numRows * (numColumns + 1)], xSquareNative[numRows * (numColumns + 1) + 1], xSquareNative[(numRows - 1) * (numColumns + 1) + 1], xSquareNative[(numRows - 1) * (numColumns + 1)]) / 8f;

						} else if (i == 0) {
							// Top Edge
							cellArea = GetPolygonArea(xSquareNative[squareIndex + numColumns], xSquareNative[squareIndex + numColumns + 2], xSquareNative[squareIndex + 1], xSquareNative[squareIndex - 1]) * 3f / 16f;

						} else if (j == 0) {
							// Left Edge
							cellArea = GetPolygonArea(xSquareNative[squareIndex + numColumns + 1], xSquareNative[squareIndex + numColumns + 2], xSquareNative[squareIndex - numColumns], xSquareNative[squareIndex - numColumns - 1]) * 3f / 16f;

						} else {
							// Inner Point
							cellArea = GetPolygonArea(xSquareNative[squareIndex + numColumns], xSquareNative[squareIndex + numColumns + 2], xSquareNative[squareIndex - numColumns], xSquareNative[squareIndex - numColumns - 2]) / 8f;

						}

					} else if (i == numRows && j == 0) {
						// Bottom Left
						cellArea = GetPolygonArea(xSquareNative[numRows * (numColumns + 1)], xSquareNative[numRows * (numColumns + 1) + 1], xSquareNative[(numRows - 1) * (numColumns + 1) + 1], xSquareNative[(numRows - 1) * (numColumns + 1)]) / 4f;

					} else if (i == numRows && j == numColumns) {
						// Bottom Right
						cellArea = GetPolygonArea(xSquareNative[xSquareNative.Length - 2], xSquareNative[xSquareNative.Length - 1], xSquareNative[numRows * (numColumns + 1) - 1], xSquareNative[numRows * (numColumns + 1) - 2]) / 4f;
					} else if (i == 0 && j == numColumns) {
						// Top Right
						cellArea = GetPolygonArea(xSquareNative[numColumns * 2], xSquareNative[numColumns + 1 + numColumns], xSquareNative[numColumns], xSquareNative[numColumns - 1]) / 4f;

					} else if (i == 0) {
						// Top Edge
						cellArea = GetPolygonArea(xSquareNative[squareIndex + numColumns], xSquareNative[squareIndex + numColumns + 2], xSquareNative[squareIndex + 1], xSquareNative[squareIndex - 1]) / 4f;

					} else if (j == 0) {
						// Left Edge
						cellArea = GetPolygonArea(xSquareNative[squareIndex + numColumns + 1], xSquareNative[squareIndex + numColumns + 2], xSquareNative[squareIndex - numColumns], xSquareNative[squareIndex - numColumns - 1]) / 4f;

					} else if (i == numRows) {
						// Bottom Edge
						cellArea = GetPolygonArea(xSquareNative[squareIndex - 1], xSquareNative[squareIndex + 1], xSquareNative[squareIndex - numColumns], xSquareNative[squareIndex - numColumns - 2]) / 4f;

					} else if (j == numColumns) {
						// Right Edge
						cellArea = GetPolygonArea(xSquareNative[squareIndex + numColumns], xSquareNative[squareIndex + numColumns + 1], xSquareNative[squareIndex - numColumns - 1], xSquareNative[squareIndex - numColumns - 2]) / 4f;

					} else {
						// Inner Point
						cellArea = GetPolygonArea(xSquareNative[squareIndex + numColumns], xSquareNative[squareIndex + numColumns + 2], xSquareNative[squareIndex - numColumns], xSquareNative[squareIndex - numColumns - 2]) / 4f;

					}

					cellArea *= (1f + compression.x) * (1f + compression.y);
					w[index] = wScale / (dispatcher.materials[(int)clothMaterial].surfaceDensity * cellArea);
					dragFactor[index++] = 0.5f * cellArea * airDensity;

				}
			}
		}

		for (int i = 0; i < lod.pinIndices.Count; i++) {
			pinnedVertInvMass.Add(w[lod.pinIndices[i]]);
			w[lod.pinIndices[i]] = 0;

			if (cutPins.Count == 0 || activeLODIndex == maxQualityLODIndex || i < cutPinStartIndex) {
				pinnedVertLocalPos.Add(pinnedVertices[lod.pinListIndices[i]].vector);
			} else {
				pinnedVertLocalPos.Add(cutPins[lod.pinListIndices[i]].vector);
			}
		}

		InitConstraints(untransformedX);

		untransformedX.Dispose();
		xSquareNative.Dispose();
		rectIndToTriIndNative.Dispose();
	}

	private void DecreaseLOD(int lodIndex) {
		if (lodIndex < 0) lodIndex = 0;

		UpdatePinnedVerticesOnLOD(lodIndex);

		ClothLOD lod = LODs[lodIndex];

		int numRows = lod.numRows;
		int numColumns = lod.numColumns;
		int minSubdivision = (int)Mathf.Min(lod.numRows, lod.numColumns);
		int minSubdivisionPrev = (int)Mathf.Min(LODs[activeLODIndex].numRows, LODs[activeLODIndex].numColumns);

		// Calculate the number of vertices
		numOneSidedVerts = (numRows + 1) * (numColumns + 1);

		NativeArray<int> rectIndToTriIndNative = new NativeArray<int>(numOneSidedVerts, Allocator.Persistent);
		NativeArray<Vector2> xSquareNative = new NativeArray<Vector2>(numOneSidedVerts, Allocator.Persistent);

		if (clothShape == ClothShape.Trapezoidal) {
			numOneSidedVerts -= minSubdivision * (minSubdivision + 1) / 2;
		}

		NativeArray<Vector2> untransformedX = new NativeArray<Vector2>(numOneSidedVerts, Allocator.Persistent);
		NativeArray<Vector3> newX = new NativeArray<Vector3>(numOneSidedVerts, Allocator.Persistent);
		NativeArray<Vector3> newV = new NativeArray<Vector3>(numOneSidedVerts, Allocator.Persistent);
		NativeArray<Vector3> newNormals = new NativeArray<Vector3>(numOneSidedVerts, Allocator.Persistent);
		NativeArray<Vector2> newUV = new NativeArray<Vector2>(numOneSidedVerts, Allocator.Persistent);
		NativeArray<Vector2> newTextureUV = new NativeArray<Vector2>(numOneSidedVerts, Allocator.Persistent);

		if (isDoubleSided) {

			if (clothShape == ClothShape.Quadrilateral) {
				triangles = new int[numRows * numColumns * 12];
			} else if (clothShape == ClothShape.Trapezoidal) {
				triangles = new int[numRows * numColumns * 12 - minSubdivision * minSubdivision * 6];
			}

			numOneSidedTriangles = triangles.Length / 6;

		} else {

			if (clothShape == ClothShape.Quadrilateral) {
				triangles = new int[numRows * numColumns * 6];
			} else if (clothShape == ClothShape.Trapezoidal) {
				triangles = new int[numRows * numColumns * 6 - minSubdivision * minSubdivision * 3];
			}

			numOneSidedTriangles = triangles.Length / 3;
		}

		w = new float[numOneSidedVerts];
		dragFactor = new float[numOneSidedVerts];

		int multiplier = (int)Mathf.Pow(2, LODs[activeLODIndex].subdivisions - lod.subdivisions);

		int index = 0;
		for (int i = 0; i < numRows + 1; i++) {

			float rowWidth = (bottomWidth - topWidth) / numRows * i + topWidth;
			float rowOffset = Mathf.Lerp(topCenter - topWidth / 2f, bottomCenter - bottomWidth / 2f, (float)i / numRows);

			float semiAxisA = rowWidth / (numColumns * 2f);

			for (int j = 0; j < numColumns + 1; j++) {

				int adjustedIndex = (j + i * (LODs[activeLODIndex].numColumns + 1)) * multiplier;

				float colHeight = (rightHeight - leftHeight) / numColumns * j + leftHeight;
				float colOffset = Mathf.Lerp(leftCenter - leftHeight / 2f, rightCenter - rightHeight / 2f, (float)j / numColumns);

				float xPos = (j * rowWidth) / numColumns + rowOffset;
				float yPos = colHeight - (i * colHeight) / numRows + colOffset;

				if (randomness > 0 && j > (clothShape == ClothShape.Trapezoidal ? minSubdivision - i : 0) && j < numColumns && i > 0 && i < numRows && !lod.pinIndices.Contains(index)) {
					float semiAxisB = colHeight / (numRows * 2f);

					int perVertexSeed = seed + index * (int)Mathf.Pow(2, maxSubdivisions) / (int)Mathf.Pow(2, lod.subdivisions);
					System.Random rand = new System.Random(perVertexSeed);

					float randAngle = (float)rand.NextDouble() * 2f * Mathf.PI;
					float randDist = (float)rand.NextDouble() * 0.6f * randomness;
					Vector3 adjustment = new Vector3(semiAxisA * Mathf.Cos(randAngle), semiAxisB * Mathf.Sin(randAngle), 0) * randDist;

					xPos += adjustment.x;
					yPos += adjustment.y;
				}

				if (clothShape == ClothShape.Quadrilateral) {
					newX[index] = x[adjustedIndex];
					newV[index] = v[adjustedIndex];
					newNormals[index] = normals[adjustedIndex];
					newUV[index] = uv[adjustedIndex];
					newTextureUV[index] = textureUV[adjustedIndex];
					untransformedX[index] = new Vector2(xPos, yPos);

					index++;
				} else if (clothShape == ClothShape.Trapezoidal) {

					xSquareNative[i * (numColumns + 1) + j] = new Vector2(xPos, yPos);

					if (j >= minSubdivision - i) {

						int minSubdivisionMinAdjustedRowIndex = minSubdivisionPrev - i * multiplier;
						adjustedIndex -= minSubdivisionPrev * (minSubdivisionPrev + 1) / 2;

						if (minSubdivisionMinAdjustedRowIndex - 1 > 0) {
							adjustedIndex += (minSubdivisionMinAdjustedRowIndex - 1) * (minSubdivisionMinAdjustedRowIndex) / 2;
						}

						newX[index] = x[adjustedIndex];
						newV[index] = v[adjustedIndex];
						newNormals[index] = normals[adjustedIndex];
						newUV[index] = uv[adjustedIndex];
						newTextureUV[index] = textureUV[adjustedIndex];
						untransformedX[index] = new Vector2(xPos, yPos);

						rectIndToTriIndNative[i * (numColumns + 1) + j] = index;

						index++;
					}
				}
			}
		}

		activeLODIndex = lodIndex;

		if (x.IsCreated) x.Dispose();
		if (v.IsCreated) v.Dispose();
		if (normals.IsCreated) normals.Dispose();
		if (uv.IsCreated) uv.Dispose();
		if (textureUV.IsCreated) textureUV.Dispose();

		x = new NativeArray<Vector3>(numOneSidedVerts, Allocator.Persistent);
		v = new NativeArray<Vector3>(numOneSidedVerts, Allocator.Persistent);
		normals = new NativeArray<Vector3>(numOneSidedVerts, Allocator.Persistent);
		uv = new NativeArray<Vector2>(numOneSidedVerts, Allocator.Persistent);
		textureUV = new NativeArray<Vector2>(numOneSidedVerts, Allocator.Persistent);

		unsafe {

			int size = Marshal.SizeOf(typeof(Vector3)) * numOneSidedVerts;
			IntPtr sourcePtr = (IntPtr)((Vector3*)newX.GetUnsafePtr());
			IntPtr destPtr = (IntPtr)((Vector3*)x.GetUnsafePtr());
			UnsafeUtility.MemCpy((void*)destPtr, (void*)sourcePtr, size);

			sourcePtr = (IntPtr)((Vector3*)newV.GetUnsafePtr());
			destPtr = (IntPtr)((Vector3*)v.GetUnsafePtr());
			UnsafeUtility.MemCpy((void*)destPtr, (void*)sourcePtr, size);

			sourcePtr = (IntPtr)((Vector3*)newNormals.GetUnsafePtr());
			destPtr = (IntPtr)((Vector3*)normals.GetUnsafePtr());
			UnsafeUtility.MemCpy((void*)destPtr, (void*)sourcePtr, size);

			size = Marshal.SizeOf(typeof(Vector2)) * numOneSidedVerts;
			sourcePtr = (IntPtr)((Vector2*)newUV.GetUnsafePtr());
			destPtr = (IntPtr)((Vector2*)uv.GetUnsafePtr());
			UnsafeUtility.MemCpy((void*)destPtr, (void*)sourcePtr, size);

			sourcePtr = (IntPtr)((Vector2*)newTextureUV.GetUnsafePtr());
			destPtr = (IntPtr)((Vector2*)textureUV.GetUnsafePtr());
			UnsafeUtility.MemCpy((void*)destPtr, (void*)sourcePtr, size);

		}

		newX.Dispose();
		newV.Dispose();
		newNormals.Dispose();
		newUV.Dispose();
		newTextureUV.Dispose();

		// Create arrays to store triangle indices
		vertexToTriangles = new List<int>[numOneSidedVerts];
		for (int i = 0; i < vertexToTriangles.Length; i++) {
			vertexToTriangles[i] = new List<int>(4);
		}
		int triangleIndex = 0;

		// Generate triangle indices
		for (int i = 0; i < numRows; i++) {
			for (int j = 0; j < numColumns; j++) {
				int realTriIndex = triangleIndex / 3;

				if (clothShape == ClothShape.Quadrilateral) {
					int id0 = i * (numColumns + 1) + j;
					int id1 = id0 + 1;
					int id2 = (i + 1) * (numColumns + 1) + j;
					int id3 = id2 + 1;

					vertexToTriangles[id0].Add(realTriIndex);
					vertexToTriangles[id1].Add(realTriIndex);
					vertexToTriangles[id2].Add(realTriIndex);
					triangles[triangleIndex++] = id0;
					triangles[triangleIndex++] = id1;
					triangles[triangleIndex++] = id2;

					vertexToTriangles[id1].Add(realTriIndex);
					vertexToTriangles[id3].Add(realTriIndex);
					vertexToTriangles[id2].Add(realTriIndex);
					triangles[triangleIndex++] = id1;
					triangles[triangleIndex++] = id3;
					triangles[triangleIndex++] = id2;

					if (isDoubleSided) {
						triangles[triangles.Length / 2 + triangleIndex - 6] = id0;
						triangles[triangles.Length / 2 + triangleIndex - 5] = id2;
						triangles[triangles.Length / 2 + triangleIndex - 4] = id1;

						triangles[triangles.Length / 2 + triangleIndex - 3] = id1;
						triangles[triangles.Length / 2 + triangleIndex - 2] = id2;
						triangles[triangles.Length / 2 + triangleIndex - 1] = id3;
					}
				} else if (clothShape == ClothShape.Trapezoidal) {
					if (j < minSubdivision - i - 1) continue;

					int id0 = i * (numColumns + 1) + j;
					int id1 = id0 + 1;
					int id2 = (i + 1) * (numColumns + 1) + j;
					int id3 = id2 + 1;

					if (j > minSubdivision - i - 1) {
						vertexToTriangles[rectIndToTriIndNative[id0]].Add(realTriIndex);
						vertexToTriangles[rectIndToTriIndNative[id1]].Add(realTriIndex);
						vertexToTriangles[rectIndToTriIndNative[id2]].Add(realTriIndex);
						triangles[triangleIndex++] = rectIndToTriIndNative[id0];
						triangles[triangleIndex++] = rectIndToTriIndNative[id1];
						triangles[triangleIndex++] = rectIndToTriIndNative[id2];
					}

					vertexToTriangles[rectIndToTriIndNative[id1]].Add(realTriIndex);
					vertexToTriangles[rectIndToTriIndNative[id3]].Add(realTriIndex);
					vertexToTriangles[rectIndToTriIndNative[id2]].Add(realTriIndex);
					triangles[triangleIndex++] = rectIndToTriIndNative[id1];
					triangles[triangleIndex++] = rectIndToTriIndNative[id3];
					triangles[triangleIndex++] = rectIndToTriIndNative[id2];

					if (isDoubleSided) {
						if (j == minSubdivision - i - 1) {

							triangles[triangles.Length / 2 + triangleIndex - 3] = rectIndToTriIndNative[id1];
							triangles[triangles.Length / 2 + triangleIndex - 2] = rectIndToTriIndNative[id2];
							triangles[triangles.Length / 2 + triangleIndex - 1] = rectIndToTriIndNative[id3];
						} else {

							triangles[triangles.Length / 2 + triangleIndex - 6] = rectIndToTriIndNative[id0];
							triangles[triangles.Length / 2 + triangleIndex - 5] = rectIndToTriIndNative[id2];
							triangles[triangles.Length / 2 + triangleIndex - 4] = rectIndToTriIndNative[id1];

							triangles[triangles.Length / 2 + triangleIndex - 3] = rectIndToTriIndNative[id1];
							triangles[triangles.Length / 2 + triangleIndex - 2] = rectIndToTriIndNative[id2];
							triangles[triangles.Length / 2 + triangleIndex - 1] = rectIndToTriIndNative[id3];
						}
					}
				}
			}
		}

		index = 0;
		if (clothShape == ClothShape.Quadrilateral) {
			for (int i = 0; i < numRows + 1; i++) {

				for (int j = 0; j < numColumns + 1; j++) {
					float cellArea = 0;

					if (i == 0 && j == 0) {
						// Top Left
						cellArea = GetPolygonArea(untransformedX[numColumns + 1], untransformedX[numColumns + 2], untransformedX[1], untransformedX[0]) / 4f;

					} else if (i == numRows && j == 0) {
						// Bottom Left
						cellArea = GetPolygonArea(untransformedX[numRows * (numColumns + 1)], untransformedX[numRows * (numColumns + 1) + 1], untransformedX[(numRows - 1) * (numColumns + 1) + 1], untransformedX[(numRows - 1) * (numColumns + 1)]) / 4f;

					} else if (i == numRows && j == numColumns) {
						// Bottom Right
						cellArea = GetPolygonArea(untransformedX[numOneSidedVerts - 2], untransformedX[numOneSidedVerts - 1], untransformedX[numRows * (numColumns + 1) - 1], untransformedX[numRows * (numColumns + 1) - 2]) / 4f;

					} else if (i == 0 && j == numColumns) {
						// Top Right
						cellArea = GetPolygonArea(untransformedX[numColumns * 2], untransformedX[numColumns + 1 + numColumns], untransformedX[numColumns], untransformedX[numColumns - 1]) / 4f;

					} else if (i == 0) {
						// Top Edge
						cellArea = GetPolygonArea(untransformedX[index + numColumns], untransformedX[index + numColumns + 2], untransformedX[index + 1], untransformedX[index - 1]) / 4f;

					} else if (j == 0) {
						// Left Edge
						cellArea = GetPolygonArea(untransformedX[index + numColumns + 1], untransformedX[index + numColumns + 2], untransformedX[index - numColumns], untransformedX[index - numColumns - 1]) / 4f;

					} else if (i == numRows) {
						// Bottom Edge
						cellArea = GetPolygonArea(untransformedX[index - 1], untransformedX[index + 1], untransformedX[index - numColumns], untransformedX[index - numColumns - 2]) / 4f;

					} else if (j == numColumns) {
						// Right Edge
						cellArea = GetPolygonArea(untransformedX[index + numColumns], untransformedX[index + numColumns + 1], untransformedX[index - numColumns - 1], untransformedX[index - numColumns - 2]) / 4f;

					} else {
						// Inner Point
						cellArea = GetPolygonArea(untransformedX[index + numColumns], untransformedX[index + numColumns + 2], untransformedX[index - numColumns], untransformedX[index - numColumns - 2]) / 4f;

					}

					cellArea *= (1f + compression.x) * (1f + compression.y);
					w[index] = wScale / (dispatcher.materials[(int)clothMaterial].surfaceDensity * cellArea);
					dragFactor[index] = 0.5f * cellArea * airDensity;
					index++;
				}
			}
		} else if (clothShape == ClothShape.Trapezoidal) {
			for (int i = 0; i < numRows + 1; i++) {

				for (int j = 0; j < numColumns + 1; j++) {
					int squareIndex = j + i * (numColumns + 1);
					float cellArea = 0;

					if (j < minSubdivision - i) continue;

					if (j == minSubdivision - i) {
						// Top Left
						if (i == 0 && j == numColumns) {
							// Top Right
							cellArea = GetPolygonArea(xSquareNative[numColumns * 2], xSquareNative[numColumns * 2 + 1], xSquareNative[numColumns], xSquareNative[numColumns - 1]) / 8f;

						} else if (i == numRows && j == 0) {
							// Bottom Left
							cellArea = GetPolygonArea(xSquareNative[numRows * (numColumns + 1)], xSquareNative[numRows * (numColumns + 1) + 1], xSquareNative[(numRows - 1) * (numColumns + 1) + 1], xSquareNative[(numRows - 1) * (numColumns + 1)]) / 8f;

						} else if (i == 0) {
							// Top Edge
							cellArea = GetPolygonArea(xSquareNative[squareIndex + numColumns], xSquareNative[squareIndex + numColumns + 2], xSquareNative[squareIndex + 1], xSquareNative[squareIndex - 1]) * 3f / 16f;

						} else if (j == 0) {
							// Left Edge
							cellArea = GetPolygonArea(xSquareNative[squareIndex + numColumns + 1], xSquareNative[squareIndex + numColumns + 2], xSquareNative[squareIndex - numColumns], xSquareNative[squareIndex - numColumns - 1]) * 3f / 16f;

						} else {
							// Inner Point
							cellArea = GetPolygonArea(xSquareNative[squareIndex + numColumns], xSquareNative[squareIndex + numColumns + 2], xSquareNative[squareIndex - numColumns], xSquareNative[squareIndex - numColumns - 2]) / 8f;

						}

					} else if (i == numRows && j == 0) {
						// Bottom Left
						cellArea = GetPolygonArea(xSquareNative[numRows * (numColumns + 1)], xSquareNative[numRows * (numColumns + 1) + 1], xSquareNative[(numRows - 1) * (numColumns + 1) + 1], xSquareNative[(numRows - 1) * (numColumns + 1)]) / 4f;

					} else if (i == numRows && j == numColumns) {
						// Bottom Right
						cellArea = GetPolygonArea(xSquareNative[xSquareNative.Length - 2], xSquareNative[xSquareNative.Length - 1], xSquareNative[numRows * (numColumns + 1) - 1], xSquareNative[numRows * (numColumns + 1) - 2]) / 4f;
					} else if (i == 0 && j == numColumns) {
						// Top Right
						cellArea = GetPolygonArea(xSquareNative[numColumns * 2], xSquareNative[numColumns + 1 + numColumns], xSquareNative[numColumns], xSquareNative[numColumns - 1]) / 4f;

					} else if (i == 0) {
						// Top Edge
						cellArea = GetPolygonArea(xSquareNative[squareIndex + numColumns], xSquareNative[squareIndex + numColumns + 2], xSquareNative[squareIndex + 1], xSquareNative[squareIndex - 1]) / 4f;

					} else if (j == 0) {
						// Left Edge
						cellArea = GetPolygonArea(xSquareNative[squareIndex + numColumns + 1], xSquareNative[squareIndex + numColumns + 2], xSquareNative[squareIndex - numColumns], xSquareNative[squareIndex - numColumns - 1]) / 4f;

					} else if (i == numRows) {
						// Bottom Edge
						cellArea = GetPolygonArea(xSquareNative[squareIndex - 1], xSquareNative[squareIndex + 1], xSquareNative[squareIndex - numColumns], xSquareNative[squareIndex - numColumns - 2]) / 4f;

					} else if (j == numColumns) {
						// Right Edge
						cellArea = GetPolygonArea(xSquareNative[squareIndex + numColumns], xSquareNative[squareIndex + numColumns + 1], xSquareNative[squareIndex - numColumns - 1], xSquareNative[squareIndex - numColumns - 2]) / 4f;

					} else {
						// Inner Point
						cellArea = GetPolygonArea(xSquareNative[squareIndex + numColumns], xSquareNative[squareIndex + numColumns + 2], xSquareNative[squareIndex - numColumns], xSquareNative[squareIndex - numColumns - 2]) / 4f;

					}

					cellArea *= (1f + compression.x) * (1f + compression.y);
					w[index] = wScale / (dispatcher.materials[(int)clothMaterial].surfaceDensity * cellArea);
					dragFactor[index++] = 0.5f * cellArea * airDensity;

				}
			}
		}

		for (int i = 0; i < lod.pinIndices.Count; i++) {
			pinnedVertInvMass.Add(w[lod.pinIndices[i]]);
			w[lod.pinIndices[i]] = 0;

			if (cutPins.Count == 0 || activeLODIndex == maxQualityLODIndex || i < cutPinStartIndex) {
				pinnedVertLocalPos.Add(pinnedVertices[lod.pinListIndices[i]].vector);
			} else {
				pinnedVertLocalPos.Add(cutPins[lod.pinListIndices[i]].vector);
			}
		}

		InitConstraints(untransformedX);

		untransformedX.Dispose();
		xSquareNative.Dispose();
		rectIndToTriIndNative.Dispose();
	}

	public void InitLODs() {
		if (LODs.Count == 0) {
			ClothLOD defaultLOD = new ClothLOD();
			defaultLOD.subdivisions = 0;
			defaultLOD.maxDistance = 1000;
			defaultLOD.substeps = 1;

			LODs.Add(defaultLOD);
		}

		maxSubdivisions = -1;
		for (int i = 0; i < LODs.Count; i++) {
			ClothLOD lod = LODs[i];

			int multiplier = (int)Mathf.Pow(2, lod.subdivisions);

			lod.numRows = baseNumRows * multiplier;
			lod.numColumns = baseNumColumns * multiplier;
			lod.detailLevel = lod.numColumns * lod.numRows;
			lod.maxDistanceSqr = lod.maxDistance * lod.maxDistance;

			lod.numVertices = (lod.numColumns + 1) * (lod.numRows + 1);
			if (clothShape == ClothShape.Trapezoidal) {
				int minSubdivision = (int)Mathf.Min(lod.numRows, lod.numColumns);
				lod.numVertices -= minSubdivision * (minSubdivision + 1) / 2;
			}

			if (lod.subdivisions > maxSubdivisions) {
				maxSubdivisions = lod.subdivisions;
				maxNumRows = lod.numRows;
				maxNumColumns = lod.numColumns;
				maxQualityLODIndex = i;
			}

			lod.pinIndices = new List<int>(pinnedVertices.Count);
			lod.pinListIndices = new List<int>(pinnedVertices.Count);

			LODs[i] = lod;
		}

		if (!Application.isPlaying) activeLODIndex = maxQualityLODIndex;

		UpdatePinnedVerticesOnLOD(activeLODIndex);
	}

	private void ProcessColliders() {
		if (!isInitialized || !cuttablePins || pinnedVertices.Count == 0) return;

		int testStart = 0;
		foreach (Collider col in _currentTriggers) {
			collisionTestPoints.Add(transform.InverseTransformPoint(col.transform.position));

			float collisionDist = col.bounds.extents.x;
			float collisionDist2 = collisionDist * collisionDist;

			if (col.attachedRigidbody != null) {
				Vector3 moveVec = col.attachedRigidbody.velocity * Time.fixedDeltaTime;
				float moveVecMag2 = moveVec.sqrMagnitude;

				if (moveVecMag2 > collisionDist2) {
					if (moveVecMag2 < 2 * collisionDist2) {

						Vector3 destLocalPos = transform.InverseTransformPoint(col.transform.position + moveVec);
						collisionTestPoints.Add(destLocalPos);

					} else {

						float moveMag = moveVec.magnitude;
						int numPoints = (int)(moveMag / collisionDist);

						for (int i = 0; i < numPoints; i++) {
							collisionTestPoints.Add(transform.InverseTransformPoint(col.transform.position + moveVec * (i + 1) * collisionDist / moveMag));
						}
					}
				}
			}

			for (int i = testStart; i < collisionTestPoints.Count; i++) {
				pinnedVerticesHash.Query(collisionTestPoints[i], collisionDist);

				for (int j = 0; j < pinnedVerticesHash.querySize; j++) {
					int id0 = pinnedVerticesHash.queryIds[j];

					Vector3 vec = pinnedVertices[id0].vector - collisionTestPoints[i];

					float dist2 = vec.sqrMagnitude;

					if (dist2 > collisionDist2)
						continue;

					if (!collisionCutIndices.Contains(id0)) collisionCutIndices.Add(id0);
				}
			}

			testStart = collisionTestPoints.Count;
		}

		collisionCutIndices.Sort();
		int numCutPrior = cutPins.Count;

		if (collisionCutIndices.Count > 0) {

			int minSubdivision = (int)Mathf.Min(LODs[activeLODIndex].numRows, LODs[activeLODIndex].numColumns);

			int removed = 0;
			for (int i = 0; i < collisionCutIndices.Count; i++) {
				cutPins.Add(pinnedVertices[collisionCutIndices[i] - removed]);

				int cutGridIndex = pinnedVertices[collisionCutIndices[i] - removed].index;
				pinnedVertices.RemoveAt(collisionCutIndices[i] - removed);

				int rowIndex = 0;
				int colIndex = 0;

				if (clothShape == ClothShape.Trapezoidal) {
					int passed = 0;
					int colToRowDiff = maxNumColumns - maxNumRows;
					for (int j = 0; j <= maxNumRows; j++) {
						int numOnRow = (colToRowDiff > 0 ? colToRowDiff : 0) + j + 1;
						numOnRow = numOnRow > maxNumColumns + 1 ? maxNumColumns + 1 : numOnRow;

						if (cutGridIndex > numOnRow - 1 + passed) {
							passed += numOnRow;
						} else {
							rowIndex = j;
							colIndex = maxNumColumns - numOnRow + 1 + (cutGridIndex - passed) % numOnRow;
							break;
						}
					}
				} else {
					rowIndex = cutGridIndex / (maxNumColumns + 1);
					colIndex = cutGridIndex % (maxNumColumns + 1);
				}

				int every = (int)Mathf.Pow(2, maxSubdivisions - LODs[activeLODIndex].subdivisions);

				if (colIndex % every == 0 && rowIndex % every == 0) {
					// Index is on this quality level
					int adjustedGridIndex = rowIndex / every * (LODs[activeLODIndex].numColumns + 1) + colIndex / every;

					if (clothShape == ClothShape.Trapezoidal) {
						int minSubdivisionMinAdjustedRowIndex = minSubdivision - rowIndex / every;
						adjustedGridIndex -= minSubdivision * (minSubdivision + 1) / 2;

						if (minSubdivisionMinAdjustedRowIndex - 1 > 0) {
							adjustedGridIndex += (minSubdivisionMinAdjustedRowIndex - 1) * (minSubdivisionMinAdjustedRowIndex) / 2;
						}
					}

					int adjPinIndex = LODs[activeLODIndex].pinIndices.IndexOf(adjustedGridIndex);

					w[adjustedGridIndex] = pinnedVertInvMass[adjPinIndex];
					pinnedVertLocalPos.RemoveAt(adjPinIndex);
					pinnedVertInvMass.RemoveAt(adjPinIndex);
					LODs[activeLODIndex].pinIndices.RemoveAt(adjPinIndex);
					LODs[activeLODIndex].pinListIndices.RemoveAt(adjPinIndex);

					cutPinStartIndex--;

				}

				removed++;
			}

			if (activeLODIndex != maxQualityLODIndex && cutPins.Count != 0) {
				int minSubdivisionMax = (int)Mathf.Min(maxNumRows, maxNumColumns);

				for (int i = numCutPrior; i < cutPins.Count; i++) {

					if (AddSubstitutePin(i, activeLODIndex, minSubdivision, minSubdivisionMax)) {
						pinnedVertInvMass.Add(w[LODs[activeLODIndex].pinIndices[LODs[activeLODIndex].pinIndices.Count - 1]]);
						w[LODs[activeLODIndex].pinIndices[LODs[activeLODIndex].pinIndices.Count - 1]] = 0;

						pinnedVertLocalPos.Add(cutPins[LODs[activeLODIndex].pinListIndices[LODs[activeLODIndex].pinIndices.Count - 1]].vector);
					}
				}
			}

			if (removed > 0) {
				dispatcher.refreshPinnedQueued = true;
			}

			if (pinnedVertices.Count > 0) {
				pinnedVerticesHash.Create(pinnedVertices);
			}

			UpdatePinnedBounds();

			collisionTestPoints.Clear();
			collisionCutIndices.Clear();
		}
	}

	private void OnTriggerStay(Collider other) {
		for (int i = 0; i < collisionTags.Count; i++) {
			if (other.CompareTag(collisionTags[i]) && other.attachedRigidbody != null) {
				if (other.attachedRigidbody.mass * other.attachedRigidbody.velocity.magnitude >= minCuttingMomentum) {
					_currentTriggers.Add(other);
				}
			}
		}
	}

	private void UpdatePinnedBounds() {
		pinnedCenter = Vector3.zero;

		Vector3 lowestLeftOut = Vector3.positiveInfinity;
		Vector3 highestRightIn = Vector3.negativeInfinity;

		for (int i = 0; i < pinnedVertices.Count; i++) {
			Vector3 vert = pinnedVertices[i].vector;
			pinnedCenter += vert;

			if (vert.x < lowestLeftOut.x) {
				lowestLeftOut.x = vert.x;
			}

			if (vert.y < lowestLeftOut.y) {
				lowestLeftOut.y = vert.y;
			}

			if (vert.z < lowestLeftOut.z) {
				lowestLeftOut.z = vert.z;
			}

			if (vert.x > highestRightIn.x) {
				highestRightIn.x = vert.x;
			}

			if (vert.y > highestRightIn.y) {
				highestRightIn.y = vert.y;
			}

			if (vert.z > highestRightIn.z) {
				highestRightIn.z = vert.z;
			}
		}

		Vector3 geoCenter = Vector3.zero;

		if (pinnedVertices.Count > 0) {
			pinnedCenter /= pinnedVertices.Count;
			geoCenter = (lowestLeftOut + highestRightIn) / 2;
		}

		Vector3 extent = Vector3.zero;
		for (int i = 0; i < pinnedVertices.Count; i++) {
			Vector3 vert = pinnedVertices[i].vector - geoCenter;

			if (Mathf.Abs(vert.x) > extent.x) {
				extent.x = Mathf.Abs(vert.x);
			}

			if (Mathf.Abs(vert.y) > extent.y) {
				extent.y = Mathf.Abs(vert.y);
			}

			if (Mathf.Abs(vert.z) > extent.z) {
				extent.z = Mathf.Abs(vert.z);
			}
		}

		collider.center = geoCenter;
		collider.size = 2 * extent;
	}

	private void UpdatePinnedVerticesOnLOD(int lodIndex) {
		LODs[lodIndex].pinIndices.Clear();
		LODs[lodIndex].pinListIndices.Clear();

		int minSubdivision = (int)Mathf.Min(LODs[lodIndex].numRows, LODs[lodIndex].numColumns);

		for (int i = 0; i < pinnedVertices.Count; i++) {

			pinnedVertices[i] = new IndexAndVector3(pinnedVertices[i].index, pinnedVertices[i].vector, -1);

			int rowIndex = 0;
			int colIndex = 0;

			if (clothShape == ClothShape.Trapezoidal) {
				int passed = 0;
				int colToRowDiff = maxNumColumns - maxNumRows;
				for (int j = 0; j <= maxNumRows; j++) {
					int numOnRow = (colToRowDiff > 0 ? colToRowDiff : 0) + j + 1;
					numOnRow = numOnRow > maxNumColumns + 1 ? maxNumColumns + 1 : numOnRow;

					if (pinnedVertices[i].index > numOnRow - 1 + passed) {
						passed += numOnRow;
					} else {
						rowIndex = j;
						colIndex = maxNumColumns - numOnRow + 1 + (pinnedVertices[i].index - passed) % numOnRow;
						break;
					}
				}
			} else {
				rowIndex = pinnedVertices[i].index / (maxNumColumns + 1);
				colIndex = pinnedVertices[i].index % (maxNumColumns + 1);
			}

			int every = (int)Mathf.Pow(2, maxSubdivisions - LODs[lodIndex].subdivisions);

			if (colIndex % every == 0 && rowIndex % every == 0) {
				// Index is on this quality level
				int adjustedGridIndex = rowIndex / every * (LODs[lodIndex].numColumns + 1) + colIndex / every;

				if (clothShape == ClothShape.Trapezoidal) {
					int minSubdivisionMinAdjustedRowIndex = minSubdivision - rowIndex / every;
					adjustedGridIndex -= minSubdivision * (minSubdivision + 1) / 2;

					if (minSubdivisionMinAdjustedRowIndex - 1 > 0) {
						adjustedGridIndex += (minSubdivisionMinAdjustedRowIndex - 1) * (minSubdivisionMinAdjustedRowIndex) / 2;
					}
				}

				LODs[lodIndex].pinIndices.Add(adjustedGridIndex);
				LODs[lodIndex].pinListIndices.Add(i);
			}
		}

		if (lodIndex != maxQualityLODIndex && cutPins.Count != 0 && pinnedVertices.Count != 0) {
			int minSubdivisionMax = (int)Mathf.Min(maxNumRows, maxNumColumns);
			cutPinStartIndex = LODs[lodIndex].pinListIndices.Count;

			for (int i = 0; i < cutPins.Count; i++) {
				cutPins[i] = new IndexAndVector3(cutPins[i].index, cutPins[i].vector, -1);
				AddSubstitutePin(i, lodIndex, minSubdivision, minSubdivisionMax);
			}
		}
	}

	private bool AddSubstitutePin(int i, int lodIndex, int minSubdivision, int minSubdivisionMax) {
		int rowIndex = 0;
		int colIndex = 0;

		if (clothShape == ClothShape.Trapezoidal) {
			int passed = 0;
			int colToRowDiff = maxNumColumns - maxNumRows;
			for (int j = 0; j <= maxNumRows; j++) {
				int numOnRow = (colToRowDiff > 0 ? colToRowDiff : 0) + j + 1;
				numOnRow = numOnRow > maxNumColumns + 1 ? maxNumColumns + 1 : numOnRow;

				if (cutPins[i].index > numOnRow - 1 + passed) {
					passed += numOnRow;
				} else {
					rowIndex = j;
					colIndex = maxNumColumns - numOnRow + 1 + (cutPins[i].index - passed) % numOnRow;
					break;
				}
			}
		} else {
			rowIndex = cutPins[i].index / (maxNumColumns + 1);
			colIndex = cutPins[i].index % (maxNumColumns + 1);
		}

		int every = (int)Mathf.Pow(2, maxSubdivisions - LODs[lodIndex].subdivisions);

		if (colIndex % every == 0 && rowIndex % every == 0) {
			// Index is on this quality level

			int radius = every / 2;
			for (int r = rowIndex - radius; r <= rowIndex + radius; r++) {
				// Iterate through columns within the radius

				int minSubdivisionMinRowIndex = minSubdivisionMax - r;

				for (int c = colIndex - radius; c <= colIndex + radius; c++) {
					// Check boundary conditions: Ensure (r, c) is within the grid
					if (r == rowIndex && c == colIndex) continue;

					if (r >= 0 && r <= maxNumRows && c >= (minSubdivisionMinRowIndex > 0 && clothShape == ClothShape.Trapezoidal ? minSubdivisionMinRowIndex : 0) && c <= maxNumColumns) {
						int index = r * (maxNumColumns + 1) + c;

						if (clothShape == ClothShape.Trapezoidal) {
							index -= minSubdivisionMax * (minSubdivisionMax + 1) / 2;

							if (minSubdivisionMinRowIndex - 1 > 0) {
								index += (minSubdivisionMinRowIndex - 1) * (minSubdivisionMinRowIndex) / 2;
							}
						}

						for (int k = 0; k < pinnedVertices.Count; k++) {
							if (pinnedVertices[k].index == index && pinnedVertices[k].substituteIndex == -1) {
								int adjustedGridIndex = rowIndex / every * (LODs[lodIndex].numColumns + 1) + colIndex / every;

								if (clothShape == ClothShape.Trapezoidal) {
									int minSubdivisionMinAdjustedRowIndex = minSubdivision - rowIndex / every;
									adjustedGridIndex -= minSubdivision * (minSubdivision + 1) / 2;

									if (minSubdivisionMinAdjustedRowIndex - 1 > 0) {
										adjustedGridIndex += (minSubdivisionMinAdjustedRowIndex - 1) * (minSubdivisionMinAdjustedRowIndex) / 2;
									}
								}

								pinnedVertices[k] = new IndexAndVector3(pinnedVertices[k].index, pinnedVertices[k].vector, adjustedGridIndex);

								LODs[lodIndex].pinIndices.Add(adjustedGridIndex);
								LODs[lodIndex].pinListIndices.Add(i);

								return true;
							}
						}
					}
				}
			}
		} else if (cutPins[i].substituteIndex != -1) {
			// Substitue pin was cut, remove
			int adjPinIndex = LODs[lodIndex].pinIndices.IndexOf(cutPins[i].substituteIndex);

			w[cutPins[i].substituteIndex] = pinnedVertInvMass[adjPinIndex];
			pinnedVertLocalPos.RemoveAt(adjPinIndex);
			pinnedVertInvMass.RemoveAt(adjPinIndex);
			LODs[lodIndex].pinIndices.RemoveAt(adjPinIndex);
			int listIndex = LODs[lodIndex].pinListIndices[adjPinIndex];
			LODs[lodIndex].pinListIndices.RemoveAt(adjPinIndex);

			return AddSubstitutePin(listIndex, lodIndex, minSubdivision, minSubdivisionMax);
		}

		return false;
	}

	void OnDestroy() {

		if (dispatcher == null) {
			dispatcher = FindAnyObjectByType<ClothDispatcher>();
		}

		if (dispatcher != null) {
			dispatcher.refreshAllQueued = true;
		}

		_currentTriggers.Clear();

		if (x.IsCreated) x.Dispose();
		if (v.IsCreated) v.Dispose();
		if (normals.IsCreated) normals.Dispose();
		if (uv.IsCreated) uv.Dispose();
		if (textureUV.IsCreated) textureUV.Dispose();

#if UNITY_EDITOR
		if (!Application.isPlaying && meshFilter.sharedMesh != null) meshFilter.sharedMesh.Clear();
#endif
	}

	private void OnDisable() {

		if (dispatcher == null) {
			dispatcher = FindAnyObjectByType<ClothDispatcher>();
		}

		if (dispatcher != null) {
			dispatcher.refreshAllQueued = true;
		}

		_currentTriggers.Clear();

		if (x.IsCreated) x.Dispose();
		if (v.IsCreated) v.Dispose();
		if (normals.IsCreated) normals.Dispose();
		if (uv.IsCreated) uv.Dispose();
		if (textureUV.IsCreated) textureUV.Dispose();

#if UNITY_EDITOR
		//if (!Application.isPlaying && meshFilter.sharedMesh != null) meshFilter.sharedMesh.Clear();
#endif
	}

	private void OnEnable() {
		if (isInitialized) {
			Init();

			dispatcher.refreshAllQueued = true;
		}
	}

	private float GetPolygonArea(params Vector2[] vertices) {

		float area = 0;
		for (int i = 0; i < vertices.Length; i++) {
			area += vertices[i].x * vertices[(i + 1) % vertices.Length].y - vertices[(i + 1) % vertices.Length].x * vertices[i].y;
		}

		return Mathf.Abs(area / 2f);
	}

	public void ResetCloth() {
		isInitialized = false;
		pinnedVertices.Clear();
		selectedVertices.Clear();
		InitLODs();
		Init();
		seed = UnityEngine.Random.Range(0, 2000000000);
	}

#if UNITY_EDITOR
	private void OnValidate() {
		if (!Application.isPlaying) {
			if (liveEdit) {
				if (meshFilter != null) {
					if (meshFilter.sharedMesh != null) meshFilter.sharedMesh.Clear();
				}
				isInitialized = false;
				pinnedVertices.Clear();
				selectedVertices.Clear();
				InitLODs();
				Init();

				if (isInitialized) {
					if (meshRenderer == null) meshRenderer = GetComponent<MeshRenderer>();
					if (dispatcher == null) dispatcher = UnityEngine.Object.FindAnyObjectByType<ClothDispatcher>();

					if (!uv.IsCreated || !textureUV.IsCreated) return;

					// Update UV's
					if (clothShape == ClothShape.Quadrilateral) {
						for (int i = 0; i < LODs[maxQualityLODIndex].numRows + 1; i++) {
							for (int j = 0; j < LODs[maxQualityLODIndex].numColumns + 1; j++) {
								int index = j + i * (LODs[maxQualityLODIndex].numColumns + 1);

								Vector2 texUV = new Vector2((uv[index].x - textureOffset.x) / textureSize.x, (uv[index].y - textureOffset.y) / textureSize.y);
								Vector2 rotatedTexUV = new Vector2(texUV.x * Mathf.Cos(Mathf.Deg2Rad * textureRotation) - texUV.y * Mathf.Sin(Mathf.Deg2Rad * textureRotation), texUV.x * Mathf.Sin(Mathf.Deg2Rad * textureRotation) + texUV.y * Mathf.Cos(Mathf.Deg2Rad * textureRotation)) + Vector2.one / 2;

								textureUV[index] = rotatedTexUV;

								if (!Application.isPlaying && isDoubleSided) {
									textureUV[numOneSidedVerts + index] = rotatedTexUV;
								}
							}
						}
					} else if (clothShape == ClothShape.Trapezoidal) {
						int minSubdivision = (int)Mathf.Min(LODs[maxQualityLODIndex].numRows, LODs[maxQualityLODIndex].numColumns);
						int ind = 0;
						for (int i = 0; i < LODs[maxQualityLODIndex].numRows + 1; i++) {
							for (int j = 0; j < LODs[maxQualityLODIndex].numColumns + 1; j++) {
								if (j >= minSubdivision - i) {

									Vector2 texUV = new Vector2((uv[ind].x - textureOffset.x) / textureSize.x, (uv[ind].y - textureOffset.y) / textureSize.y);
									Vector2 rotatedTexUV = new Vector2(texUV.x * Mathf.Cos(Mathf.Deg2Rad * textureRotation) - texUV.y * Mathf.Sin(Mathf.Deg2Rad * textureRotation), texUV.x * Mathf.Sin(Mathf.Deg2Rad * textureRotation) + texUV.y * Mathf.Cos(Mathf.Deg2Rad * textureRotation)) + Vector2.one / 2;

									textureUV[ind] = rotatedTexUV;

									if (!Application.isPlaying && isDoubleSided) {
										textureUV[numOneSidedVerts + ind] = rotatedTexUV;
									}

									ind++;
								}
							}
						}
					}

					meshFilter.sharedMesh.uv = uv.ToArray();
					meshFilter.sharedMesh.uv2 = textureUV.ToArray();

					Material mat = new Material(dispatcher.materials[(int)clothMaterial].material);
					if ((int)clothTexture > 0) {
						mat.SetInt("_UseTexture", 1);
						mat.SetTexture("_Texture", dispatcher.textures[(int)clothTexture].texture);
					} else {
						mat.SetInt("_UseTexture", 0);
						mat.SetTexture("_Texture", null);
					}
					meshRenderer.sharedMaterial = mat;
				}
			}
		}
	}

	public void ShowCloth() {
		//if (!isInitialized || meshFilter.sharedMesh == null) Init();
		if (meshFilter == null) meshFilter = GetComponent<MeshFilter>();
		if (meshRenderer == null) meshRenderer = GetComponent<MeshRenderer>();
		if (dispatcher == null) UnityEngine.Object.FindAnyObjectByType<ClothDispatcher>();

		//meshRenderer.sharedMaterial = dispatcher.materials[(int)clothMaterial].material;
		Init();

		meshRenderer.enabled = true;
	}

	public void HideCloth() {
		if (meshFilter == null) meshFilter = GetComponent<MeshFilter>();
		if (meshRenderer == null) meshRenderer = GetComponent<MeshRenderer>();

		meshRenderer.enabled = false;
		meshFilter.sharedMesh.Clear();
		isInitialized = false;
		showShapingGizmos = false;
		selectedVertices.Clear();

		if (x.IsCreated) x.Dispose();
		if (normals.IsCreated) normals.Dispose();
		if (uv.IsCreated) uv.Dispose();
		if (textureUV.IsCreated) textureUV.Dispose();
		triangles = null;
		w = null;
	}

	#region Editor Selection and Pinning
	public void SelectRow(int row) {
		int minSubdivision = (int)Mathf.Min(maxNumRows, maxNumColumns);
		int index = 0;

		row *= maxNumRows / baseNumRows;

		for (int i = 0; i < maxNumRows + 1; i++) {
			for (int j = 0; j < maxNumColumns + 1; j++) {

				if (clothShape == ClothShape.Quadrilateral) {

					if (i == row) {
						if (!selectedVertices.Contains(i * (maxNumColumns + 1) + j)) selectedVertices.Add(i * (maxNumColumns + 1) + j);
					}

				} else if (clothShape == ClothShape.Trapezoidal) {

					if (j >= minSubdivision - i) {

						if (i == row) {
							if (!selectedVertices.Contains(index)) selectedVertices.Add(index);
						}

						index++;
					}
				}
			}
		}
	}

	public void SelectColumn(int col) {
		int minSubdivision = (int)Mathf.Min(maxNumRows, maxNumColumns);
		int index = 0;

		col *= maxNumRows / baseNumRows;

		for (int i = 0; i < maxNumRows + 1; i++) {
			for (int j = 0; j < maxNumColumns + 1; j++) {

				if (clothShape == ClothShape.Quadrilateral) {

					if (j == col) {
						if (!selectedVertices.Contains(i * (maxNumColumns + 1) + j)) selectedVertices.Add(i * (maxNumColumns + 1) + j);
					}

				} else if (clothShape == ClothShape.Trapezoidal) {

					if (j >= minSubdivision - i) {

						if (j == col) {
							if (!selectedVertices.Contains(index)) selectedVertices.Add(index);
						}

						index++;
					}
				}
			}
		}
	}

	public void PinCorners() {
		int minSubdivision = (int)Mathf.Min(maxNumRows, maxNumColumns);
		int index = 0;

		for (int i = 0; i < maxNumRows + 1; i++) {
			for (int j = 0; j < maxNumColumns + 1; j++) {

				if (clothShape == ClothShape.Quadrilateral) {

					if ((j < 1 || j > maxNumColumns - 1) && (i < 1 || i > maxNumRows - 1)) {
						index = i * (maxNumColumns + 1) + j;

						bool isPinned = false;
						for (int k = 0; k < pinnedVertices.Count; k++) {
							if (pinnedVertices[k].index == index) {
								isPinned = true;
								break;
							}
						}

						if (!isPinned) {

							bool isConnected = false;
							for (int k = 0; k < connectedVertices.Count; k++) {
								if (connectedVertices[k].vertIndex == index) {
									isConnected = true;
									break;
								}
							}

							if (!isConnected) pinnedVertices.Add(new IndexAndVector3(index, x[index]));
						}
					}

				} else if (clothShape == ClothShape.Trapezoidal) {

					if (j >= minSubdivision - i) {
						if ((j < 1 || j > maxNumColumns - 1) && (i < 1 || i > maxNumRows - 1)) {
							bool isPinned = false;
							for (int k = 0; k < pinnedVertices.Count; k++) {
								if (pinnedVertices[k].index == index) {
									isPinned = true;
									break;
								}
							}

							if (!isPinned) {

								bool isConnected = false;
								for (int k = 0; k < connectedVertices.Count; k++) {
									if (connectedVertices[k].vertIndex == index) {
										isConnected = true;
										break;
									}
								}

								if (!isConnected) pinnedVertices.Add(new IndexAndVector3(index, x[index]));
							}
						}

						index++;
					}

				}

			}
		}
	}

	public void PinLeftEdge() {
		int minSubdivision = (int)Mathf.Min(maxNumRows, maxNumColumns);
		int index = 0;

		for (int i = 0; i < maxNumRows + 1; i++) {
			for (int j = 0; j < maxNumColumns + 1; j++) {

				if (clothShape == ClothShape.Quadrilateral) {

					if (j == 0) {
						index = i * (maxNumColumns + 1) + j;

						bool isPinned = false;
						for (int k = 0; k < pinnedVertices.Count; k++) {
							if (pinnedVertices[k].index == index) {
								isPinned = true;
								break;
							}
						}

						if (!isPinned) {

							bool isConnected = false;
							for (int k = 0; k < connectedVertices.Count; k++) {
								if (connectedVertices[k].vertIndex == index) {
									isConnected = true;
									break;
								}
							}

							if (!isConnected) pinnedVertices.Add(new IndexAndVector3(index, x[index]));
						}
					}

				} else if (clothShape == ClothShape.Trapezoidal) {

					if (j >= minSubdivision - i) {
						if (j == 0 || j == minSubdivision - i) {
							bool isPinned = false;
							for (int k = 0; k < pinnedVertices.Count; k++) {
								if (pinnedVertices[k].index == index) {
									isPinned = true;
									break;
								}
							}

							if (!isPinned) {

								bool isConnected = false;
								for (int k = 0; k < connectedVertices.Count; k++) {
									if (connectedVertices[k].vertIndex == index) {
										isConnected = true;
										break;
									}
								}

								if (!isConnected) pinnedVertices.Add(new IndexAndVector3(index, x[index]));
							}
						}

						index++;
					}

				}

			}
		}
	}

	public void PinRightEdge() {
		int minSubdivision = (int)Mathf.Min(maxNumRows, maxNumColumns);
		int index = 0;

		for (int i = 0; i < maxNumRows + 1; i++) {
			for (int j = 0; j < maxNumColumns + 1; j++) {

				if (clothShape == ClothShape.Quadrilateral) {

					if (j == maxNumColumns) {
						index = i * (maxNumColumns + 1) + j;

						bool isPinned = false;
						for (int k = 0; k < pinnedVertices.Count; k++) {
							if (pinnedVertices[k].index == index) {
								isPinned = true;
								break;
							}
						}

						if (!isPinned) {

							bool isConnected = false;
							for (int k = 0; k < connectedVertices.Count; k++) {
								if (connectedVertices[k].vertIndex == index) {
									isConnected = true;
									break;
								}
							}

							if (!isConnected) pinnedVertices.Add(new IndexAndVector3(index, x[index]));
						}
					}

				} else if (clothShape == ClothShape.Trapezoidal) {

					if (j >= minSubdivision - i) {
						if (j == maxNumColumns) {
							bool isPinned = false;
							for (int k = 0; k < pinnedVertices.Count; k++) {
								if (pinnedVertices[k].index == index) {
									isPinned = true;
									break;
								}
							}

							if (!isPinned) {

								bool isConnected = false;
								for (int k = 0; k < connectedVertices.Count; k++) {
									if (connectedVertices[k].vertIndex == index) {
										isConnected = true;
										break;
									}
								}

								if (!isConnected) pinnedVertices.Add(new IndexAndVector3(index, x[index]));
							}
						}

						index++;
					}

				}

			}
		}
	}

	public void PinTopEdge() {
		int minSubdivision = (int)Mathf.Min(maxNumRows, maxNumColumns);
		int index = 0;

		for (int i = 0; i < maxNumRows + 1; i++) {
			for (int j = 0; j < maxNumColumns + 1; j++) {

				if (clothShape == ClothShape.Quadrilateral) {

					if (i == 0) {
						index = i * (maxNumColumns + 1) + j;

						bool isPinned = false;
						for (int k = 0; k < pinnedVertices.Count; k++) {
							if (pinnedVertices[k].index == index) {
								isPinned = true;
								break;
							}
						}

						if (!isPinned) {

							bool isConnected = false;
							for (int k = 0; k < connectedVertices.Count; k++) {
								if (connectedVertices[k].vertIndex == index) {
									isConnected = true;
									break;
								}
							}

							if (!isConnected) pinnedVertices.Add(new IndexAndVector3(index, x[index]));
						}
					}

				} else if (clothShape == ClothShape.Trapezoidal) {

					if (j >= minSubdivision - i) {
						if (i == 0) {
							bool isPinned = false;
							for (int k = 0; k < pinnedVertices.Count; k++) {
								if (pinnedVertices[k].index == index) {
									isPinned = true;
									break;
								}
							}

							if (!isPinned) {

								bool isConnected = false;
								for (int k = 0; k < connectedVertices.Count; k++) {
									if (connectedVertices[k].vertIndex == index) {
										isConnected = true;
										break;
									}
								}

								if (!isConnected) pinnedVertices.Add(new IndexAndVector3(index, x[index]));
							}
						}

						index++;
					}

				}

			}
		}
	}

	public void PinBottomEdge() {
		int minSubdivision = (int)Mathf.Min(maxNumRows, maxNumColumns);
		int index = 0;

		for (int i = 0; i < maxNumRows + 1; i++) {
			for (int j = 0; j < maxNumColumns + 1; j++) {

				if (clothShape == ClothShape.Quadrilateral) {

					if (i == maxNumRows) {
						index = i * (maxNumColumns + 1) + j;

						bool isPinned = false;
						for (int k = 0; k < pinnedVertices.Count; k++) {
							if (pinnedVertices[k].index == index) {
								isPinned = true;
								break;
							}
						}

						if (!isPinned) {

							bool isConnected = false;
							for (int k = 0; k < connectedVertices.Count; k++) {
								if (connectedVertices[k].vertIndex == index) {
									isConnected = true;
									break;
								}
							}

							if (!isConnected) pinnedVertices.Add(new IndexAndVector3(index, x[index]));
						}
					}

				} else if (clothShape == ClothShape.Trapezoidal) {

					if (j >= minSubdivision - i) {
						if (i == maxNumRows) {
							bool isPinned = false;
							for (int k = 0; k < pinnedVertices.Count; k++) {
								if (pinnedVertices[k].index == index) {
									isPinned = true;
									break;
								}
							}

							if (!isPinned) {

								bool isConnected = false;
								for (int k = 0; k < connectedVertices.Count; k++) {
									if (connectedVertices[k].vertIndex == index) {
										isConnected = true;
										break;
									}
								}

								if (!isConnected) pinnedVertices.Add(new IndexAndVector3(index, x[index]));
							}
						}

						index++;
					}

				}

			}
		}
	}

	public void PinAllEdges() {
		PinLeftEdge();
		PinRightEdge();
		PinBottomEdge();
		PinTopEdge();
	}
	#endregion
#endif
}

class Hash {
	public float spacing;
	public int tableSize;
	public int[] cellStart;
	public int[] cellEntries;
	public int[] queryIds;
	public int querySize;
	public int maxNumObjects;
	public int[] firstAdjId;
	public int[] adjIds;

	public Hash(float spacing, int maxNumObjects) {
		this.spacing = spacing;
		tableSize = 5 * maxNumObjects;
		cellStart = new int[tableSize + 1];
		cellEntries = new int[maxNumObjects];
		queryIds = new int[maxNumObjects];
		querySize = 0;

		this.maxNumObjects = maxNumObjects;
		firstAdjId = new int[maxNumObjects + 1];
		adjIds = new int[10 * maxNumObjects];
	}

	private int HashCoords(int xi, int yi, int zi) {
		int h = (xi * 92837111) ^ (yi * 689287499) ^ (zi * 283923481);
		return Mathf.Abs(h) % tableSize;
	}

	private int IntCoord(float coord) {
		return Mathf.FloorToInt(coord / spacing);
	}

	private int HashPos(Vector3 pos) {
		return HashCoords(IntCoord(pos.x), IntCoord(pos.y), IntCoord(pos.z));
	}

	public void Create(Vector3[] pos) {
		int numObjects = Mathf.Min(pos.Length, cellEntries.Length);

		// determine cell sizes

		for (int i = 0; i < cellStart.Length; i++) {
			cellStart[i] = 0;
		}
		for (int i = 0; i < cellEntries.Length; i++) {
			cellEntries[i] = 0;
		}

		// determine cell starts

		int start = 0;
		for (int i = 0; i < tableSize; i++) {
			start += cellStart[i];
			cellStart[i] = start;
		}
		cellStart[tableSize] = start; // guard

		// Count

		for (int i = 0; i < numObjects; i++) {
			int h = HashPos(pos[i]);
			cellStart[h]++;
		}

		// Partial Sums

		for (int i = 1; i < cellStart.Length; i++) {
			cellStart[i] += cellStart[i - 1];
		}

		// fill in object ids

		for (int i = 0; i < numObjects; i++) {
			int h = HashPos(pos[i]);
			cellStart[h]--;
			cellEntries[cellStart[h]] = i;
		}
	}

	public void Create(List<Vector3> pos) {
		int numObjects = Mathf.Min(pos.Count, cellEntries.Length);

		// determine cell sizes

		for (int i = 0; i < cellStart.Length; i++) {
			cellStart[i] = 0;
		}
		for (int i = 0; i < cellEntries.Length; i++) {
			cellEntries[i] = 0;
		}

		// determine cell starts

		int start = 0;
		for (int i = 0; i < tableSize; i++) {
			start += cellStart[i];
			cellStart[i] = start;
		}
		cellStart[tableSize] = start; // guard

		// Count

		for (int i = 0; i < numObjects; i++) {
			int h = HashPos(pos[i]);
			cellStart[h]++;
		}

		// Partial Sums

		for (int i = 1; i < cellStart.Length; i++) {
			cellStart[i] += cellStart[i - 1];
		}

		// fill in object ids

		for (int i = 0; i < numObjects; i++) {
			int h = HashPos(pos[i]);
			cellStart[h]--;
			cellEntries[cellStart[h]] = i;
		}
	}

	public void Create(List<IndexAndVector3> pos) {
		int numObjects = Mathf.Min(pos.Count, cellEntries.Length);

		// determine cell sizes

		for (int i = 0; i < cellStart.Length; i++) {
			cellStart[i] = 0;
		}
		for (int i = 0; i < cellEntries.Length; i++) {
			cellEntries[i] = 0;
		}

		// determine cell starts

		int start = 0;
		for (int i = 0; i < tableSize; i++) {
			start += cellStart[i];
			cellStart[i] = start;
		}
		cellStart[tableSize] = start; // guard

		// Count

		for (int i = 0; i < numObjects; i++) {
			int h = HashPos(pos[i].vector);
			cellStart[h]++;
		}

		// Partial Sums

		for (int i = 1; i < cellStart.Length; i++) {
			cellStart[i] += cellStart[i - 1];
		}

		// fill in object ids

		for (int i = 0; i < numObjects; i++) {
			int h = HashPos(pos[i].vector);
			cellStart[h]--;
			cellEntries[cellStart[h]] = i;
		}
	}

	public void Query(Vector3 pos, float maxDist) {
		int x0 = IntCoord(pos.x - maxDist);
		int y0 = IntCoord(pos.y - maxDist);
		int z0 = IntCoord(pos.z - maxDist);

		int x1 = IntCoord(pos.x + maxDist);
		int y1 = IntCoord(pos.y + maxDist);
		int z1 = IntCoord(pos.z + maxDist);

		querySize = 0;
		HashSet<int> uniqueObjectsInQuery = new HashSet<int>();

		for (int xi = x0; xi <= x1; xi++) {
			for (int yi = y0; yi <= y1; yi++) {
				for (int zi = z0; zi <= z1; zi++) {
					int h = HashCoords(xi, yi, zi);
					int start = cellStart[h];
					int end = cellStart[h + 1];

					for (int i = start; i < end; i++) {
						if (uniqueObjectsInQuery.Add(cellEntries[i])) {
							queryIds[querySize] = cellEntries[i];
							querySize++;
						}
					}
				}
			}
		}
	}

	public void QueryAll(Vector3[] pos, float maxDist) {
		int num = 0;
		float maxDist2 = maxDist * maxDist;

		for (int i = 0; i < maxNumObjects; i++) {
			int id0 = i;
			firstAdjId[id0] = num;
			Query(pos[id0], maxDist);

			for (int j = 0; j < querySize; j++) {
				int id1 = queryIds[j];
				if (id1 >= id0)
					continue;

				float dist2 = (pos[id0] - pos[id1]).sqrMagnitude;
				if (dist2 > maxDist2)
					continue;

				if (num >= adjIds.Length) {
					Debug.Log("Creating Larger Array");
					int[] newIds = new int[2 * num];  // dynamic array
					System.Array.Copy(adjIds, newIds, adjIds.Length);
					//newIds.set(adjIds);
					adjIds = newIds;
				}
				adjIds[num++] = id1;
			}
		}

		firstAdjId[maxNumObjects] = num;
	}
}

[Serializable]
public struct IndexAndVector3 {
	public int index;
	public Vector3 vector;
	public int substituteIndex;

	public IndexAndVector3(int index, Vector3 vector, int substituteIndex = -1) {
		this.index = index;
		this.vector = vector;
		this.substituteIndex = substituteIndex;
	}

	public override string ToString() {
		return $"[{index}, {vector}]";
	}
}