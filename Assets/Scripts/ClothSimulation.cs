using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEditor.Experimental.GraphView;
using UnityEngine;

[RequireComponent(typeof(MeshFilter)), RequireComponent(typeof(MeshRenderer)), RequireComponent(typeof(Rigidbody)), RequireComponent(typeof(Collider)), ExecuteAlways]
public class ClothSimulation : MonoBehaviour
{
	[Range(0, 0.0001f)] public float stretchingCompliance = 0f;
	[Range(0, 10)] public float bendingCompliance = 0f;
	[Min(0.0000001f)] public float density = 100f; // kg / m^2
	[Range(3, 30)] public int substeps = 5;
	[Min(0.0000001f)] public float thickness = 0.001f; // m
	[Min(0.0000001f)] public float wScale = 1f;
	public bool handleCollisions = true;
	public bool simulateGravity = true;
	public bool simulateWind = true;
	[SerializeField] private bool _applyGravity = true;
	public bool applyGravity {
		get { return _applyGravity; }
		set {
			if (_applyGravity != value) {
				_applyGravity = value;

				Debug.Log($"isSimulating changed to: {_applyGravity}");
				if (personalRb != null) personalRb.useGravity = value;

#if UNITY_EDITOR
				UnityEditor.EditorUtility.SetDirty(this);
#endif
			}
		}
	}
	public bool applyWind = true;
	[Tooltip("What rigidbody the force is applied to.")] public Rigidbody forceRigidbody;

	//public float width = 10f;
	[Min(0.0000001f)] public float topWidth = 10f;
	[Min(0.0000001f)] public float bottomWidth = 10f;
	public float topCenter = 5f;
	public float bottomCenter = 5f;
	//public float height = 10f;
	[Min(0.0000001f)] public float leftHeight = 10f;
	[Min(0.0000001f)] public float rightHeight = 10f;
	public float leftCenter = 5f;
	public float rightCenter = 5f;
	[Min(2)] public int numRows = 15;
	[Min(2)] public int numColumns = 15;
	public ComputeShader clothCompute;
	[Range(0, 1)] public float damping = 0.03f;
	public Vector2 textureTiling = Vector2.one;
	public Vector2 textureOffset = Vector2.zero;
	[Range(0, 360)] public float textureRotation = 0;
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
	public Vector2 compression = Vector2.zero;
	public bool isDoubleSided = true;
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
	public bool cuttablePins = false;
	public bool liveEdit = false;

	public Vector3[] x { get; set; }
	public Vector3[] normals { get; set; }
	public Vector2[] uv { get; set; }
	public Vector2[] textureUV { get; set; }
	public float[] w { get; set; }
	public int[] triangles { get; set; }
	public float[][] d0 { get; set; }
	public float[][] dihedral0 { get; set; }
	public int[][] bendingIDs { get; set; }
	private int[] neighbors;
	public int[][] stretchingIDs { get; set; }
	public float[] dragFactor { get; set; }
	public bool isInitialized { get; set; } = false;

	public MeshFilter meshFilter { get; set; }
	public MeshRenderer meshRenderer { get; private set; }
	[HideInInspector] public List<int>[] vertexToTriangles { get; private set; }
	[HideInInspector, SerializeField] public List<int> _pinnedVertices = new List<int>();
	public List<int> pinnedVertices => _pinnedVertices; // Read-only reference variable to _pinnedVertices
	[HideInInspector, SerializeField] private Dictionary<int, Vector3> _modifiedVertices = new Dictionary<int, Vector3>();
	public Dictionary<int, Vector3> modifiedVertices => _modifiedVertices;

	public List<Vector3> pinnedVertLocalPos { get; private set; }
	private List<float> pinnedVertInvMass = new List<float>();
	[HideInInspector] public List<int> selectedVertices = new List<int>();
	private BoxCollider collider;
	private Hash pinnedVerticesHash;
	public ClothDispatcher dispatcher { get; private set; }
	public bool showShapingGizmos { get; set; } = false;

	public bool drawGraphColoring = false;
	public float distBetweenHandles { get; private set; }
	public bool isShowingVerts { get; set; } = false;
	public int cornerPinDimX { get; set; } = 1;
	public int cornerPinDimY { get; set; } = 1;
	public int numOneSidedVerts { get; private set; }
	public int numOneSidedTriangles { get; private set; }
	public bool isActive { get; private set; }
	public Vector3 positionOnReadback { get; set; }
	public Vector3 windForce { get; set; }
	public Rigidbody personalRb { get; private set; }

	private NativeArray<int> rectIndToTriIndNative;
	private NativeArray<Vector2> xSquareNative;
	public Texture2D alphaMap { get; private set; }
	private float hashSpacing = 1f;
	private Vector3 pinnedCenter;
	[SerializeField] public List<ConnectedVertex> connectedVertices = new List<ConnectedVertex>();
	public int clothIndex { get; set; }

	public enum ClothMaterial : int {
		BeachBall,
		Foam,
		Emojis,
		Wave
	};

	public enum ClothShape {
		Quadrilateral,
		Trapezoid
	};

	public enum TextureSide : int {
		Front,
		Back,
		Both
	};

	public enum ClothTexture : int { };

	public ClothMaterial clothMaterial;
	public ClothShape clothShape = ClothShape.Quadrilateral;
	private float airDensity = 1.287f; // kg/m^3

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

	// Start is called before the first frame update
	void Start()
	{
		meshFilter = GetComponent<MeshFilter>();
		meshRenderer = GetComponent<MeshRenderer>();
		personalRb = GetComponent<Rigidbody>();
		collider = GetComponent<BoxCollider>();
		dispatcher = FindAnyObjectByType<ClothDispatcher>();

		if (dispatcher != null ) {
			dispatcher.numClothsChanged = true;
			dispatcher.refreshAllQueued = true;
		}

		isActive = true;

		personalRb.constraints = RigidbodyConstraints.FreezeAll;

		Init();

		if (Application.isPlaying) {
			UpdatePinnedBounds();
			collider.enabled = true;

			if (cuttablePins) pinnedVerticesHash = new Hash(hashSpacing, pinnedVertices.Count);

			meshRenderer.enabled = false;
			//meshRenderer.material.SetTexture("_BaseColorMap", dispatcher.textures[(int)clothTexture]);
		} else {
			collider.enabled = false;
			dispatcher.materials[(int)clothMaterial].material.SetInt("_InEditor", 1);
		}

#if UNITY_EDITOR
		if (drawGraphColoring && numRows <= 40 && numColumns <= 40) {
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
		if (cuttablePins && isInitialized && Application.isPlaying && pinnedVertices.Count > 0) {
			if (pinnedVerticesHash == null) pinnedVerticesHash = new Hash(hashSpacing, pinnedVertices.Count);

			pinnedVerticesHash.Create(pinnedVertLocalPos);
		}
	}

	private void FixedUpdate() {

		/*for (int i = 0; i < normals.Length; i++) {
			Debug.DrawRay(transform.TransformPoint(x[i]), transform.TransformDirection(normals[i]) / 10, i >= normals.Length / 2 ? Color.red : Color.blue);
		}*/

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

		if (applyWind && pinnedVertices != null && pinnedVertices.Count > 0 && forceRigidbody != null) {
			forceRigidbody.AddForceAtPosition(windForce, pinnedCenter);
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

		positionOnReadback = transform.position;

		switch (clothShape) {
			case ClothShape.Quadrilateral:
				InitQuad();
				break;

			case ClothShape.Trapezoid:
				InitTrapezoid();
				break;

			default:

				InitQuad();
				break;
		}

		float maxWidth = Mathf.Max(topWidth, bottomWidth);
		float maxHeight = Mathf.Max(leftHeight, rightHeight);

		distBetweenHandles = (maxWidth > maxHeight ? maxHeight : maxWidth) / (numColumns + numRows);
		hashSpacing = (maxWidth > maxHeight ? maxWidth : maxHeight) * 2f / (numColumns + numRows);

		personalRb.useGravity = applyGravity;

#if UNITY_EDITOR
		if (!Application.isPlaying) {
			Mesh mesh = new Mesh();
			mesh.vertices = x;
			mesh.uv = uv;
			mesh.uv2 = textureUV;

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
			normals = mesh.normals;
			meshFilter.sharedMesh = mesh;
		}
#endif

		isInitialized = true;
		dispatcher.refreshAllQueued = true;
	}

	private void InitQuad() {

		// Calculate the number of vertices
		numOneSidedVerts = (numRows + 1) * (numColumns + 1);

		if (isDoubleSided) {
			if (Application.isPlaying) {
				x = new Vector3[numOneSidedVerts];
				normals = new Vector3[numOneSidedVerts];
				uv = new Vector2[numOneSidedVerts];
				textureUV = new Vector2[numOneSidedVerts];
			} else {
				x = new Vector3[numOneSidedVerts * 2];
				normals = new Vector3[numOneSidedVerts * 2];
				uv = new Vector2[numOneSidedVerts * 2];
				textureUV = new Vector2[numOneSidedVerts * 2];
			}
			triangles = new int[numRows * numColumns * 6 * 2];
			numOneSidedTriangles = triangles.Length / 6;
		} else {
			x = new Vector3[numOneSidedVerts];
			normals = new Vector3[numOneSidedVerts];
			uv = new Vector2[numOneSidedVerts];
			textureUV = new Vector2[numOneSidedVerts];
			triangles = new int[numRows * numColumns * 6];
			numOneSidedTriangles = triangles.Length / 3;
		}

		w = new float[numOneSidedVerts];
		dragFactor = new float[numOneSidedVerts];
		

		// Calculate vertex positions and UV coordinates
		for (int i = 0; i < numRows + 1; i++) {
			float rowWidth = (bottomWidth - topWidth) / numRows * i + topWidth;
			float rowOffset = Mathf.Lerp(topCenter - topWidth / 2f, bottomCenter - bottomWidth / 2f, (float)i / numRows);

			for (int j = 0; j < numColumns + 1; j++) {
				float colHeight = (rightHeight - leftHeight) / numColumns * j + leftHeight;
				float colOffset = Mathf.Lerp(leftCenter - leftHeight / 2f, rightCenter - rightHeight / 2f, (float)j / numColumns);

				int index = j + i * (numColumns + 1);
				float xPos = (j * rowWidth) / numColumns + rowOffset;
				float yPos = colHeight - (i * colHeight) / numRows + colOffset;

				x[index] = new Vector3(xPos, yPos, 0);
				uv[index] = new Vector2(xPos, yPos);

				Vector2 texUV = new Vector2((xPos - textureOffset.x) / textureTiling.x, (yPos - textureOffset.y) / textureTiling.y);
				Vector2 rotatedTexUV = new Vector2(texUV.x * Mathf.Cos(Mathf.Deg2Rad * textureRotation) - texUV.y * Mathf.Sin(Mathf.Deg2Rad * textureRotation), texUV.x * Mathf.Sin(Mathf.Deg2Rad * textureRotation) + texUV.y * Mathf.Cos(Mathf.Deg2Rad * textureRotation)) + Vector2.one / 2;

				textureUV[index] = rotatedTexUV;

				if (!Application.isPlaying && isDoubleSided) {
					x[numOneSidedVerts + index] = new Vector3(xPos, yPos, 0);
					uv[numOneSidedVerts + index] = new Vector2(xPos, yPos);
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
				vertexToTriangles[i * (numColumns + 1) + j].Add(triangleIndex / 3);
				vertexToTriangles[i * (numColumns + 1) + 1 + j].Add(triangleIndex / 3);
				vertexToTriangles[(i + 1) * (numColumns + 1) + j].Add(triangleIndex / 3);
				triangles[triangleIndex++] = i * (numColumns + 1) + j;
				triangles[triangleIndex++] = i * (numColumns + 1) + 1 + j;
				triangles[triangleIndex++] = (i + 1) * (numColumns + 1) + j;

				vertexToTriangles[i * (numColumns + 1) + 1 + j].Add(triangleIndex / 3);
				vertexToTriangles[(i + 1) * (numColumns + 1) + j + 1].Add(triangleIndex / 3);
				vertexToTriangles[(i + 1) * (numColumns + 1) + j].Add(triangleIndex / 3);
				triangles[triangleIndex++] = i * (numColumns + 1) + 1 + j;
				triangles[triangleIndex++] = (i + 1) * (numColumns + 1) + j + 1;
				triangles[triangleIndex++] = (i + 1) * (numColumns + 1) + j;

				if (isDoubleSided) {
					triangles[triangles.Length / 2 + triangleIndex - 6] = i * (numColumns + 1) + j;
					triangles[triangles.Length / 2 + triangleIndex - 5] = (i + 1) * (numColumns + 1) + j;
					triangles[triangles.Length / 2 + triangleIndex - 4] = i * (numColumns + 1) + 1 + j;

					triangles[triangles.Length / 2 + triangleIndex - 3] = i * (numColumns + 1) + 1 + j;
					triangles[triangles.Length / 2 + triangleIndex - 2] = (i + 1) * (numColumns + 1) + j;
					triangles[triangles.Length / 2 + triangleIndex - 1] = (i + 1) * (numColumns + 1) + j + 1;
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
					w[index] = wScale / (density * thickness * cellArea);
					dragFactor[index] = 0.5f * cellArea * airDensity;
					totalArea += cellArea;
				}
			}

			personalRb.mass = totalArea * thickness * density;

			pinnedVertLocalPos = new List<Vector3>(pinnedVertices.Count);
			if (pinnedVertices != null) {
				for (int i = 0; i < pinnedVertices.Count; i++) {
					pinnedVertInvMass.Add(w[pinnedVertices[i]]); 
					w[pinnedVertices[i]] = 0;
					pinnedVertLocalPos.Add(x[pinnedVertices[i]]);
				}
			}

			transform.TransformPoints(x, x);

			// Generate default distances
			neighbors = FindTriNeighbors(triangles);


			List<int>[] triPairs = new List<int>[] {
				new List<int>(), new List<int>(),  new List<int>(), new List<int>(),  new List<int>(), new List<int>()
			};
				List<int>[] edges = new List<int>[] {
				new List<int>(), new List<int>(),  new List<int>(), new List<int>(), new List<int>(), new List<int>()
			};

			int count = 0;
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

			stretchingIDs = new int[edges.Length][]; // Initialize the outer array

			for (int i = 0; i < edges.Length; i++) {
				if (edges[i] != null) {
					stretchingIDs[i] = edges[i].ToArray();
				} else {
					stretchingIDs[i] = new int[0];
				}
			}

			//bendingIDs = triPairs.ToArray();
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

					d0[i][j] = (new Vector3((x[id0] - x[id1]).x * (1 + compression.x), (x[id0] - x[id1]).y * (1 + compression.y), 0)).magnitude;
				}
			}

			dihedral0 = new float[bendingIDs.Length][];
			for (int i = 0; i < dihedral0.Length; i++) {
				dihedral0[i] = new float[bendingIDs[i].Length / 2];
				for (int j = 0; j < dihedral0[i].Length; j++) {
					int id0 = bendingIDs[i][2 * j];
					int id1 = bendingIDs[i][2 * j + 1];

					dihedral0[i][j] = (new Vector3((x[id0] - x[id1]).x * (1 + compression.x), (x[id0] - x[id1]).y * (1 + compression.y), 0)).magnitude;
				}
			}
		}
	}

	private void InitTrapezoid() {
		// Calculate the number of vertices
		numOneSidedVerts = (numRows + 1) * (numColumns + 1);
		int minSubdivision = (int)Mathf.Min(numRows, numColumns);
		numOneSidedVerts = (numRows + 1) * (numColumns + 1) - minSubdivision * (minSubdivision + 1) / 2;

		if (isDoubleSided) {
			if (Application.isPlaying) {
				x = new Vector3[numOneSidedVerts];
				normals = new Vector3[numOneSidedVerts];
				uv = new Vector2[numOneSidedVerts];
				textureUV = new Vector2[numOneSidedVerts];
			} else {
				x = new Vector3[numOneSidedVerts * 2];
				normals = new Vector3[numOneSidedVerts * 2];
				uv = new Vector2[numOneSidedVerts * 2];
				textureUV = new Vector2[numOneSidedVerts * 2];
			}
			triangles = new int[numRows * numColumns * 12 - minSubdivision * minSubdivision * 6];
			numOneSidedTriangles = triangles.Length / 6;
		} else {
			x = new Vector3[numOneSidedVerts];
			normals = new Vector3[numOneSidedVerts];
			uv = new Vector2[numOneSidedVerts];
			textureUV = new Vector2[numOneSidedVerts];
			triangles = new int[numRows * numColumns * 6 - minSubdivision * minSubdivision * 3];
			numOneSidedTriangles = triangles.Length / 3;
		}

		w = new float[numOneSidedVerts];
		dragFactor = new float[numOneSidedVerts];
		rectIndToTriIndNative = new NativeArray<int>((numRows + 1) * (numColumns + 1), Allocator.Temp);
		xSquareNative = new NativeArray<Vector2>((numRows + 1) * (numColumns + 1), Allocator.Temp);

		// Calculate vertex positions and UV coordinates
		int ind = 0;
		for (int i = 0; i < numRows + 1; i++) {
			float rowWidth = (bottomWidth - topWidth) / numRows * i + topWidth;
			float rowOffset = Mathf.Lerp(topCenter - topWidth / 2f, bottomCenter - bottomWidth / 2f, (float)i / numRows);

			for (int j = 0; j < numColumns + 1; j++) {
				float colHeight = (rightHeight - leftHeight) / numColumns * j + leftHeight;
				float colOffset = Mathf.Lerp(leftCenter - leftHeight / 2f, rightCenter - rightHeight / 2f, (float)j / numColumns);

				float xPos = (j * rowWidth) / numColumns + rowOffset;
				float yPos = colHeight - (i * colHeight) / numRows + colOffset;

				xSquareNative[i * (numColumns + 1) + j] = new Vector2(xPos, yPos);

				if (j >= minSubdivision - i) {

					x[ind] = new Vector3(xPos, yPos, 0);
					uv[ind] = new Vector2(xPos, yPos);

					Vector2 texUV = new Vector2((xPos - textureOffset.x) / textureTiling.x, (yPos - textureOffset.y) / textureTiling.y);
					Vector2 rotatedTexUV = new Vector2(texUV.x * Mathf.Cos(Mathf.Deg2Rad * textureRotation) - texUV.y * Mathf.Sin(Mathf.Deg2Rad * textureRotation), texUV.x * Mathf.Sin(Mathf.Deg2Rad * textureRotation) + texUV.y * Mathf.Cos(Mathf.Deg2Rad * textureRotation)) + Vector2.one / 2;

					textureUV[ind] = rotatedTexUV;

					if (!Application.isPlaying && isDoubleSided) {
						x[numOneSidedVerts + ind] = new Vector3(xPos, yPos, 0);
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

				if (j > minSubdivision - i - 1) {
					vertexToTriangles[rectIndToTriIndNative[i * (numColumns + 1) + j]].Add(triangleIndex / 3);
					vertexToTriangles[rectIndToTriIndNative[i * (numColumns + 1) + 1 + j]].Add(triangleIndex / 3);
					vertexToTriangles[rectIndToTriIndNative[(i + 1) * (numColumns + 1) + j]].Add(triangleIndex / 3);
					triangles[triangleIndex++] = rectIndToTriIndNative[i * (numColumns + 1) + j];
					triangles[triangleIndex++] = rectIndToTriIndNative[i * (numColumns + 1) + 1 + j];
					triangles[triangleIndex++] = rectIndToTriIndNative[(i + 1) * (numColumns + 1) + j];
				}

				vertexToTriangles[rectIndToTriIndNative[i * (numColumns + 1) + 1 + j]].Add(triangleIndex / 3);
				vertexToTriangles[rectIndToTriIndNative[(i + 1) * (numColumns + 1) + j + 1]].Add(triangleIndex / 3);
				vertexToTriangles[rectIndToTriIndNative[(i + 1) * (numColumns + 1) + j]].Add(triangleIndex / 3);
				triangles[triangleIndex++] = rectIndToTriIndNative[i * (numColumns + 1) + 1 + j];
				triangles[triangleIndex++] = rectIndToTriIndNative[(i + 1) * (numColumns + 1) + j + 1];
				triangles[triangleIndex++] = rectIndToTriIndNative[(i + 1) * (numColumns + 1) + j];

				if (isDoubleSided) {
					if (j == minSubdivision - i - 1) {

						triangles[triangles.Length / 2 + triangleIndex - 3] = rectIndToTriIndNative[i * (numColumns + 1) + 1 + j];
						triangles[triangles.Length / 2 + triangleIndex - 2] = rectIndToTriIndNative[(i + 1) * (numColumns + 1) + j];
						triangles[triangles.Length / 2 + triangleIndex - 1] = rectIndToTriIndNative[(i + 1) * (numColumns + 1) + j + 1];
					} else {

						triangles[triangles.Length / 2 + triangleIndex - 6] = rectIndToTriIndNative[i * (numColumns + 1) + j];
						triangles[triangles.Length / 2 + triangleIndex - 5] = rectIndToTriIndNative[(i + 1) * (numColumns + 1) + j];
						triangles[triangles.Length / 2 + triangleIndex - 4] = rectIndToTriIndNative[i * (numColumns + 1) + 1 + j];

						triangles[triangles.Length / 2 + triangleIndex - 3] = rectIndToTriIndNative[i * (numColumns + 1) + 1 + j];
						triangles[triangles.Length / 2 + triangleIndex - 2] = rectIndToTriIndNative[(i + 1) * (numColumns + 1) + j];
						triangles[triangles.Length / 2 + triangleIndex - 1] = rectIndToTriIndNative[(i + 1) * (numColumns + 1) + j + 1];
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
					w[ind] = wScale / (density * thickness * cellArea);
					dragFactor[ind++] = 0.5f * cellArea * airDensity;
					totalArea += cellArea;

				}
			}

			personalRb.mass = totalArea * thickness * density;

			pinnedVertLocalPos = new List<Vector3>(pinnedVertices.Count);
			if (pinnedVertices != null) {
				for (int i = 0; i < pinnedVertices.Count; i++) {
					pinnedVertInvMass.Add(w[pinnedVertices[i]]);
					w[pinnedVertices[i]] = 0;
					pinnedVertLocalPos.Add(x[pinnedVertices[i]]);
				}
			}

			transform.TransformPoints(x, x);

			// Generate default distances
			neighbors = FindTriNeighbors(triangles);

			List<int>[] triPairs = new List<int>[] {
				new List<int>(), new List<int>(),  new List<int>(), new List<int>(),  new List<int>(), new List<int>()
			};
				List<int>[] edges = new List<int>[] {
				new List<int>(), new List<int>(),  new List<int>(), new List<int>(), new List<int>(), new List<int>()
			};

			int count = 0;
			ind = 0;
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

			stretchingIDs = new int[edges.Length][]; // Initialize the outer array

			for (int i = 0; i < edges.Length; i++) {
				if (edges[i] != null) {
					stretchingIDs[i] = edges[i].ToArray();
				} else {
					stretchingIDs[i] = new int[0];
				}
			}

			//bendingIDs = triPairs.ToArray();
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

					d0[i][j] = (new Vector3((x[id0] - x[id1]).x * (1 + compression.x), (x[id0] - x[id1]).y * (1 + compression.y), 0)).magnitude;
				}
			}

			dihedral0 = new float[bendingIDs.Length][];
			for (int i = 0; i < dihedral0.Length; i++) {
				dihedral0[i] = new float[bendingIDs[i].Length / 2];
				for (int j = 0; j < dihedral0[i].Length; j++) {
					int id0 = bendingIDs[i][2 * j];
					int id1 = bendingIDs[i][2 * j + 1];

					dihedral0[i][j] = (new Vector3((x[id0] - x[id1]).x * (1 + compression.x), (x[id0] - x[id1]).y * (1 + compression.y), 0)).magnitude;
				}
			}
		}

		rectIndToTriIndNative.Dispose();
		xSquareNative.Dispose();
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

	private void OnTriggerStay(Collider other) {
		if (!isInitialized || !cuttablePins || pinnedVertices.Count == 0) return;

		float collisionDist = other.bounds.extents.x;
		float collisionDist2 = collisionDist * collisionDist;

		Vector3 otherLocalPos = transform.InverseTransformPoint(other.transform.position);

		List<Vector3> testPoints = new List<Vector3>();
		List<int> cutIndices = new List<int>(pinnedVertices.Count);

		testPoints.Add(otherLocalPos);

		if (other.attachedRigidbody != null) {
			Vector3 moveVec = other.attachedRigidbody.velocity * Time.deltaTime;
			float moveVecMag2 = moveVec.sqrMagnitude;

			if (moveVecMag2 > collisionDist2) {
				if (moveVecMag2 < 2 * collisionDist2) {

					Vector3 destLocalPos = transform.InverseTransformPoint(other.transform.position + moveVec);
					testPoints.Add(destLocalPos);

				} else {

					float moveMag = moveVec.magnitude;
					int numPoints = (int)(moveMag / collisionDist);

					for (int i = 0; i < numPoints; i++) {
						testPoints.Add(transform.InverseTransformPoint(other.transform.position + moveVec * (i + 1) * collisionDist / moveMag));
					}
				}
			}
		}

		
		
		for (int i = 0; i < testPoints.Count; i++) {
			pinnedVerticesHash.Query(testPoints[i], collisionDist);

			for (int j = 0; j < pinnedVerticesHash.querySize; j++) {
				int id0 = pinnedVerticesHash.queryIds[j];

				Vector3 vec = pinnedVertLocalPos[id0] - testPoints[i];

				float dist2 = vec.sqrMagnitude;

				if (dist2 > collisionDist2)
					continue;

				cutIndices.Add(id0);
			}
		}
		
		cutIndices.Sort();

		if (cutIndices.Count > 0) {
			for (int i = 0; i < cutIndices.Count; i++) {
				w[pinnedVertices[cutIndices[i]]] = pinnedVertInvMass[cutIndices[i]];
			}

			int removed = 0;
			for (int i = 0; i < cutIndices.Count; i++) {
				pinnedVertices.RemoveAt(cutIndices[i] - removed);
				pinnedVertLocalPos.RemoveAt(cutIndices[i] - removed);
				pinnedVertInvMass.RemoveAt(cutIndices[i] - removed);
				removed++;
			}

			if (pinnedVertices.Count > 0) {
				//pinnedVerticesHash = new Hash(hashSpacing, pinnedVertices.Count);
				pinnedVerticesHash.Create(pinnedVertLocalPos);
			}
		}

		UpdatePinnedBounds();
		dispatcher.refreshPinnedQueued = true;
	}

	private void UpdatePinnedBounds() {
		pinnedCenter = Vector3.zero;

		Vector3 lowestLeftOut = Vector3.positiveInfinity;
		Vector3 highestRightIn = Vector3.negativeInfinity;

		for (int i = 0; i < pinnedVertices.Count; i++) {
			Vector3 vert = pinnedVertLocalPos[i];
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
			Vector3 vert = pinnedVertLocalPos[i] - geoCenter;

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

	void OnDestroy() {

		isActive = false;
		if (dispatcher == null) {
			dispatcher = FindAnyObjectByType<ClothDispatcher>();
		}

		if (dispatcher != null) {
			dispatcher.refreshAllQueued = true;
		}

#if UNITY_EDITOR
		if (!Application.isPlaying && meshFilter.sharedMesh != null) meshFilter.sharedMesh.Clear();
#endif
	}

	private void OnDisable() {

		isActive = false;
		if (dispatcher == null) {
			dispatcher = FindAnyObjectByType<ClothDispatcher>();
		}

		if (dispatcher != null) {
			dispatcher.refreshAllQueued = true;
		}

#if UNITY_EDITOR
		//if (!Application.isPlaying && meshFilter.sharedMesh != null) meshFilter.sharedMesh.Clear();
#endif
	}

	private void OnEnable() {
		if (isInitialized) {
			dispatcher.refreshAllQueued = true;
		}

		isActive = true;
	}

	private float GetPolygonArea(params Vector2[] vertices) {

		float area = 0;
		for (int i = 0; i < vertices.Length; i++) {
			area += vertices[i].x * vertices[(i + 1) % vertices.Length].y - vertices[(i + 1) % vertices.Length].x * vertices[i].y;
		}

		return Mathf.Abs(area / 2f);
	}

#if UNITY_EDITOR
	private void OnValidate() {
		if (!Application.isPlaying) {
			if (liveEdit) {
				if (meshFilter.sharedMesh != null) meshFilter.sharedMesh.Clear();
				pinnedVertices.Clear();
				modifiedVertices.Clear();
				selectedVertices.Clear();
				isInitialized = false;
				Init();
			}

			if (isInitialized) {
				if (meshRenderer == null) meshRenderer = GetComponent<MeshRenderer>();
				if (dispatcher == null) dispatcher = UnityEngine.Object.FindAnyObjectByType<ClothDispatcher>();

				// Update UV's
				if (clothShape == ClothShape.Quadrilateral) {
					for (int i = 0; i < numRows + 1; i++) {
						float rowWidth = (bottomWidth - topWidth) / numRows * i + topWidth;
						float rowOffset = Mathf.Lerp(topCenter - topWidth / 2f, bottomCenter - bottomWidth / 2f, (float)i / numRows);

						for (int j = 0; j < numColumns + 1; j++) {
							float colHeight = (rightHeight - leftHeight) / numColumns * j + leftHeight;
							float colOffset = Mathf.Lerp(leftCenter - leftHeight / 2f, rightCenter - rightHeight / 2f, (float)j / numColumns);

							int index = j + i * (numColumns + 1);
							float xPos = (j * rowWidth) / numColumns + rowOffset;
							float yPos = colHeight - (i * colHeight) / numRows + colOffset;

							uv[index] = new Vector2(xPos, yPos);

							Vector2 texUV = new Vector2((xPos - textureOffset.x) / textureTiling.x, (yPos - textureOffset.y) / textureTiling.y);
							Vector2 rotatedTexUV = new Vector2(texUV.x * Mathf.Cos(Mathf.Deg2Rad * textureRotation) - texUV.y * Mathf.Sin(Mathf.Deg2Rad * textureRotation), texUV.x * Mathf.Sin(Mathf.Deg2Rad * textureRotation) + texUV.y * Mathf.Cos(Mathf.Deg2Rad * textureRotation)) + Vector2.one / 2;

							textureUV[index] = rotatedTexUV;

							if (!Application.isPlaying && isDoubleSided) {
								uv[numOneSidedVerts + index] = new Vector2(xPos, yPos);
								textureUV[numOneSidedVerts + index] = rotatedTexUV;
							}
						}
					}
				} else if (clothShape == ClothShape.Trapezoid) {
					int minSubdivision = (int)Mathf.Min(numRows, numColumns);
					int ind = 0;
					for (int i = 0; i < numRows + 1; i++) {
						float rowWidth = (bottomWidth - topWidth) / numRows * i + topWidth;
						float rowOffset = Mathf.Lerp(topCenter - topWidth / 2f, bottomCenter - bottomWidth / 2f, (float)i / numRows);

						for (int j = 0; j < numColumns + 1; j++) {
							float colHeight = (rightHeight - leftHeight) / numColumns * j + leftHeight;
							float colOffset = Mathf.Lerp(leftCenter - leftHeight / 2f, rightCenter - rightHeight / 2f, (float)j / numColumns);

							float xPos = (j * rowWidth) / numColumns + rowOffset;
							float yPos = colHeight - (i * colHeight) / numRows + colOffset;

							if (j >= minSubdivision - i) {

								uv[ind] = new Vector2(xPos, yPos);

								Vector2 texUV = new Vector2((xPos - textureOffset.x) / textureTiling.x, (yPos - textureOffset.y) / textureTiling.y);
								Vector2 rotatedTexUV = new Vector2(texUV.x * Mathf.Cos(Mathf.Deg2Rad * textureRotation) - texUV.y * Mathf.Sin(Mathf.Deg2Rad * textureRotation), texUV.x * Mathf.Sin(Mathf.Deg2Rad * textureRotation) + texUV.y * Mathf.Cos(Mathf.Deg2Rad * textureRotation)) + Vector2.one / 2;

								textureUV[ind] = rotatedTexUV;

								if (!Application.isPlaying && isDoubleSided) {
									x[numOneSidedVerts + ind] = new Vector3(xPos, yPos, 0);
									textureUV[numOneSidedVerts + ind] = rotatedTexUV;
								}

								ind++;
							}
						}
					}
				}

				meshFilter.sharedMesh.uv = uv;
				meshFilter.sharedMesh.uv2 = textureUV;


				//Material mat = new Material(editorMaterial);
				//mat.SetTexture("_BaseMap", dispatcher.materials[(int)clothMaterial].colorMap);
				//mat.SetTexture("_NormalMap", dispatcher.materials[(int)clothMaterial].normalMap);
				//mat.SetTexture("_MetalnessMap", dispatcher.materials[(int)clothMaterial].metalnessMap);
				//mat.SetTexture("_RoughnessMap", dispatcher.materials[(int)clothMaterial].smoothnessMap);
				//mat.SetTexture("_AmbientOcclusionMap", dispatcher.materials[(int)clothMaterial].ambientOcclusionMap);
				//if (dispatcher.materials[(int)clothMaterial].alphaMap != null) mat.SetTexture("_AlphaMap", dispatcher.materials[(int)clothMaterial].alphaMap);
				//meshRenderer.sharedMaterial = mat;
				//Material mat = new Material(meshRenderer.sharedMaterial);
				dispatcher.materials[(int)clothMaterial].material.SetInt("_InEditor", 1);

				meshRenderer.sharedMaterial = new Material(dispatcher.materials[(int)clothMaterial].material);
				meshRenderer.sharedMaterial.SetTexture("_Texture", dispatcher.textures[(int)clothTexture]);
				if ((int)clothTexture > 0) {
					meshRenderer.sharedMaterial.SetTexture("_Texture", dispatcher.textures[(int)clothTexture]);
					meshRenderer.sharedMaterial.SetInt("_UseTexture", 1);
				}
			}
		}
	}

	public void ShowCloth() {
		//if (!isInitialized || meshFilter.sharedMesh == null) Init();
		if (meshFilter == null) meshFilter = GetComponent<MeshFilter>();
		if (meshRenderer == null) meshRenderer = GetComponent<MeshRenderer>();
		if (dispatcher == null) UnityEngine.Object.FindAnyObjectByType<ClothDispatcher>();


		//Material mat = new Material(meshRenderer.sharedMaterial);
		//mat.SetTexture("_BaseColorMap", dispatcher.materials[(int)clothMaterial].material);
		meshRenderer.sharedMaterial = dispatcher.materials[(int)clothMaterial].material;
		Init();

		meshRenderer.enabled = true;
	}

	public void HideCloth() {
		if (meshFilter == null) meshFilter = GetComponent<MeshFilter>();
		if (meshRenderer == null) meshRenderer = GetComponent<MeshRenderer>();

		meshRenderer.enabled = false;
		meshFilter.sharedMesh.Clear();
		isInitialized = false;
		x = null;
		normals = null;
		uv = null;
		triangles = null;
		w = null;
	}

	public void PinCorners() {
		int minSubdivision = (int)Mathf.Min(numRows, numColumns);
		int index = 0;

		for (int i = 0; i < numRows + 1; i++) {
			for (int j = 0; j < numColumns + 1; j++) {

				if (clothShape == ClothShape.Quadrilateral) {

					if ((j < cornerPinDimX || j > numColumns - cornerPinDimX) && (i < cornerPinDimY || i > numRows - cornerPinDimY)) {
						if (!pinnedVertices.Contains(i * (numColumns + 1) + j)) {
							pinnedVertices.Add(i * (numColumns + 1) + j);
						}
					}

				} else if (clothShape == ClothShape.Trapezoid) {

					if (j >= minSubdivision - i) {
						if ((j < cornerPinDimX || j > numColumns - cornerPinDimX) && (i < cornerPinDimY || i > numRows - cornerPinDimY)) {
							if (!pinnedVertices.Contains(index)) {
								pinnedVertices.Add(index);
							}
						}

						index++;
					}

				}

			}
		}
	}

	public void PinLeftEdge(int pinWidth) {
		int minSubdivision = (int)Mathf.Min(numRows, numColumns);
		int index = 0;

		for (int i = 0; i < numRows + 1; i++) {
			for (int j = 0; j < numColumns + 1; j++) {

				if (clothShape == ClothShape.Quadrilateral) {

					if (j < pinWidth) {
						if (!pinnedVertices.Contains(i * (numColumns + 1) + j)) {
							pinnedVertices.Add(i * (numColumns + 1) + j);
						}
					}

				} else if (clothShape == ClothShape.Trapezoid) {

					if (j >= minSubdivision - i) {
						if (j < pinWidth || j < minSubdivision - i + pinWidth) {
							if (!pinnedVertices.Contains(index)) {
								pinnedVertices.Add(index);
							}
						}

						index++;
					}

				}

			}
		}
	}

	public void PinRightEdge(int pinWidth) {
		int minSubdivision = (int)Mathf.Min(numRows, numColumns);
		int index = 0;

		for (int i = 0; i < numRows + 1; i++) {
			for (int j = 0; j < numColumns + 1; j++) {

				if (clothShape == ClothShape.Quadrilateral) {

					if (j > numColumns - pinWidth) {
						if (!pinnedVertices.Contains(i * (numColumns + 1) + j)) {
							pinnedVertices.Add(i * (numColumns + 1) + j);
						}
					}

				} else if (clothShape == ClothShape.Trapezoid) {

					if (j >= minSubdivision - i) {
						if (j > numColumns - pinWidth) {
							if (!pinnedVertices.Contains(index)) {
								pinnedVertices.Add(index);
							}
						}

						index++;
					}

				}

			}
		}
	}

	public void PinTopEdge(int pinWidth) {
		int minSubdivision = (int)Mathf.Min(numRows, numColumns);
		int index = 0;

		for (int i = 0; i < numRows + 1; i++) {
			for (int j = 0; j < numColumns + 1; j++) {

				if (clothShape == ClothShape.Quadrilateral) {

					if (i < pinWidth) {
						if (!pinnedVertices.Contains(i * (numColumns + 1) + j)) {
							pinnedVertices.Add(i * (numColumns + 1) + j);
						}
					}

				} else if (clothShape == ClothShape.Trapezoid) {

					if (j >= minSubdivision - i) {
						if (i < pinWidth) {
							if (!pinnedVertices.Contains(index)) {
								pinnedVertices.Add(index);
							}
						}

						index++;
					}

				}

			}
		}
	}

	public void PinBottomEdge(int pinWidth) {
		int minSubdivision = (int)Mathf.Min(numRows, numColumns);
		int index = 0;

		for (int i = 0; i < numRows + 1; i++) {
			for (int j = 0; j < numColumns + 1; j++) {

				if (clothShape == ClothShape.Quadrilateral) {

					if (i > numRows - pinWidth) {
						if (!pinnedVertices.Contains(i * (numColumns + 1) + j)) {
							pinnedVertices.Add(i * (numColumns + 1) + j);
						}
					}

				} else if (clothShape == ClothShape.Trapezoid) {

					if (j >= minSubdivision - i) {
						if (i > numRows - pinWidth) {
							if (!pinnedVertices.Contains(index)) {
								pinnedVertices.Add(index);
							}
						}

						index++;
					}

				}

			}
		}
	}

	public void PinAllEdges(int pinWidth) {
		PinLeftEdge(pinWidth);
		PinRightEdge(pinWidth);
		PinBottomEdge(pinWidth);
		PinTopEdge(pinWidth);
	}

	private void OnDrawGizmosSelected() {
		if (!showShapingGizmos || x == null || x.Length == 0) {
			return;
		}

		Vector3[] worldX = new Vector3[x.Length];
		transform.TransformPoints(x, worldX);
		Vector3 size = Vector3.one * distBetweenHandles * 1.5f;

		for (int i = 0; i < x.Length / 2; i++) {
			if (_pinnedVertices.Contains(i)) {
				Gizmos.color = Color.red;
				Gizmos.DrawWireCube(worldX[i], size);
			}
		}
	}
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