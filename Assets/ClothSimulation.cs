using NUnit.Framework.Constraints;
using NUnit.Framework.Internal;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;
using static UnityEngine.InputManagerEntry;

public class ClothSimulation : MonoBehaviour
{
	[Range(0, 0.0001f)]
	public float stretchingCompliance = 0f;
	[Range(0, 10)]
	public float bendingCompliance = 0.5f;
	public float density = 0.1f; // kg / m^2
	[Range(3, 20)]
	public int substeps = 5;
	public float thickness = 0.004f; // m
	public bool handleCollisions = true;
	public bool applyGravity = true;
	public bool applyWind = true;

	public float width = 10f;
	public float height = 10f;
	public int subdivisions = 15;
	public ComputeShader clothCompute;
	public float damping = 0.03f;

	public Vector3[] x { get; set; }
	public Vector3[] normals { get; set; }
	private Vector3[] v;
	private Vector3[] restPos;
	public float[] w { get; set; }
	public float[][] d0 { get; set; }
	public float[][] dihedral0 { get; set; }
	public int[][] bendingIDs { get; set; }
	private int[] neighbors;
	public int[][] stretchingIDs { get; set; }
	public bool isInitialized { get; set; } = false;
	public bool isSimulating { get; set; } = false;

	private MeshFilter meshFilter;
	public MeshRenderer meshRenderer { get; private set; }
	public List<int>[] vertexToTriangles { get; private set; }
	private MeshCollider meshCollider;
	public Mesh mesh { get; set; }
	private Hash hash;

	public bool drawGraphColoring = false;


	// Start is called before the first frame update
	void Start()
	{
		//meshFilter = GetComponent<MeshFilter>();
		//meshRenderer = GetComponent<MeshRenderer>();
		//meshCollider = GetComponent<MeshCollider>();

		Init();

		if (drawGraphColoring) {
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
					int id0 = bendingIDs[i][j * 4 + 2];
					int id1 = bendingIDs[i][j * 4 + 3];

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
	}
	void Update()
	{
		
	}

	private void FixedUpdate() {
		//mesh.vertices = x;
		//mesh.RecalculateBounds();

		//mesh.normals = normals;
		//mesh.RecalculateNormals();
		//meshCollider.sharedMesh = mesh;
	}

	private void SolveCollisions(Vector3[] p) {
		float thickness2 = thickness * thickness;

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
		}
	}

	public void Init() {

		// Create an empty Mesh
		mesh = new Mesh();

		// Calculate the number of vertices
		int vertexCount = (subdivisions + 1) * (subdivisions + 1);


		x = new Vector3[vertexCount * 2];
		v = new Vector3[vertexCount];
		w = new float[vertexCount];
		normals = new Vector3[vertexCount * 2];
		Vector2[] uv = new Vector2[vertexCount * 2];

		float wScale = 0.00001f;
		float invCellMass = (subdivisions * subdivisions) / (density * thickness * width * height) * wScale;
		//float maxW = 0f; // For normalization

		// Calculate vertex positions and UV coordinates
		for (int i = 0; i < subdivisions + 1; i++) {
			for (int j = 0; j < subdivisions + 1; j++) {
				x[j + i * (subdivisions + 1)] = new Vector3(j / (float)subdivisions * width, 0, height - i / (float)subdivisions * height);
				x[vertexCount + j + i * (subdivisions + 1)] = new Vector3(j / (float)subdivisions * width, 0, height - i / (float)subdivisions * height);
				uv[j + i * (subdivisions + 1)] = new Vector3(j / (float)subdivisions * width, height - i / (float)subdivisions * height);
				uv[vertexCount + j + i * (subdivisions + 1)] = new Vector3(j / (float)subdivisions * width, height - i / (float)subdivisions * height);

				if ((i == 0 && j == 0) || (i == subdivisions && j == 0) || (i == subdivisions && j == subdivisions) || (i == 0 && j == subdivisions)) {
					// Corner Point
					w[j + i * (subdivisions + 1)] = 4f * invCellMass;
				} else if (i == 0 || i == subdivisions || j == 0 || j == subdivisions) {
					// Edge Point
					w[j + i * (subdivisions + 1)] = 2f * invCellMass;
				} else {
					// Inner Point
					w[j + i * (subdivisions + 1)] = invCellMass;
				}
				//w[j + i * (subdivisions + 1)] = 1;
				//maxW = Mathf.Max(maxW, w[j + i * (subdivisions + 1)]); // For normalization
			}
		}

		// Normalization (Optional)
		/*for (int i = 0; i < subdivisions + 1; i++) {
			for (int j = 0; j < subdivisions + 1; j++) {
				w[j + i * (subdivisions + 1)] /= maxW; // Normalize to 0-1 range
			}
		}*/

		//w[0] = 0;
		int attachmentLength = subdivisions / 20;
		attachmentLength = attachmentLength == 0 ? 1 : attachmentLength;
		for (int i = 0; i < attachmentLength; i++) {
			w[i] = 0;
		}
		for (int i = subdivisions; i > subdivisions - attachmentLength; i--) {
			w[i] = 0;
		}
		for (int i = w.Length - 1; i > w.Length - 1 - attachmentLength; i--) {
			w[i] = 0;
		}
		for (int i = w.Length - 1 - subdivisions; i < w.Length - 1 - subdivisions + attachmentLength; i++) {
			w[i] = 0;
		}
		//w[subdivisions] = 0;
		//w[w.Length - 1] = 0;
		//w[w.Length - 1 - subdivisions] = 0;
		
		//w[w.Length / 2 + subdivisions / 2] = 0;

		// Create arrays to store triangle indices
		int[] tris = new int[subdivisions * subdivisions * 6 * 2];
		vertexToTriangles = new List<int>[vertexCount];
		for (int i = 0; i < vertexToTriangles.Length; i++)
		{
			vertexToTriangles[i] = new List<int>(4);
		}
		int triangleIndex = 0;

		// Generate triangle indices
		for (int i = 0; i < subdivisions; i++) {
			for (int j = 0; j < subdivisions; j++) {
				vertexToTriangles[i * (subdivisions + 1) + j].Add(triangleIndex / 3);
				vertexToTriangles[i * (subdivisions + 1) + 1 + j].Add(triangleIndex / 3);
				vertexToTriangles[(i + 1) * (subdivisions + 1) + j].Add(triangleIndex / 3);
				tris[triangleIndex++] = i * (subdivisions + 1) + j;
				tris[triangleIndex++] = i * (subdivisions + 1) + 1 + j;
				tris[triangleIndex++] = (i + 1) * (subdivisions + 1) + j;

				tris[tris.Length - triangleIndex] = vertexCount + i * (subdivisions + 1) + j;
				tris[tris.Length - triangleIndex + 1] = vertexCount + (i + 1) * (subdivisions + 1) + j;
				tris[tris.Length - triangleIndex + 2] = vertexCount + i * (subdivisions + 1) + 1 + j;


				vertexToTriangles[i * (subdivisions + 1) + 1 + j].Add(triangleIndex / 3);
				vertexToTriangles[(i + 1) * (subdivisions + 1) + j + 1].Add(triangleIndex / 3);
				vertexToTriangles[(i + 1) * (subdivisions + 1) + j].Add(triangleIndex / 3);
				tris[triangleIndex++] = i * (subdivisions + 1) + 1 + j;
				tris[triangleIndex++] = (i + 1) * (subdivisions + 1) + j + 1;
				tris[triangleIndex++] = (i + 1) * (subdivisions + 1) + j;

				tris[tris.Length - triangleIndex] = vertexCount + i * (subdivisions + 1) + 1 + j;
				tris[tris.Length - triangleIndex + 1] = vertexCount + (i + 1) * (subdivisions + 1) + j;
				tris[tris.Length - triangleIndex + 2] = vertexCount + (i + 1) * (subdivisions + 1) + j + 1;
			}
		}

		// Assign the calculated data to the mesh
		mesh.vertices = x;
		mesh.triangles = tris;
		mesh.uv = uv;
		mesh.RecalculateNormals();
		mesh.RecalculateTangents();
		mesh.RecalculateBounds();

		// Assign the mesh to the GameObject
		//meshFilter.mesh = mesh;
		//meshCollider.sharedMesh = mesh;


		// Generate default distances
		int numTris = tris.Length / 6;
		neighbors = FindTriNeighbors(tris);


		List<int>[] triPairs = new List<int>[] {
			new List<int>(), new List<int>(),  new List<int>(), new List<int>(),  new List<int>(), new List<int>()
		};
		List<int>[] edges = new List<int>[] {
			new List<int>(), new List<int>(),  new List<int>(), new List<int>(), new List<int>(), new List<int>()
		};

		int count = 0;
		for (int i = 0; i < numTris; i++) {
			bool evenTri = i % 2 == 1; // Is the triangle a top-left triangle instead of bottom-right
			bool evenCol = (i / 2) % 2 == 1; // Is the column the triangle is in an even one
			bool evenRow = (i / (subdivisions * 2)) % 2 == 1; // Is the row the triangle is in an even one
			bool topRow = i < subdivisions * 2;
			bool bottomRow = i > (numTris - subdivisions * 2);

			for (int j = 0; j < 3; j++) {
				int id0 = tris[i * 3 + j];
				int id1 = tris[i * 3 + (j + 1) % 3];

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
						} else if (i % (subdivisions * 2) == 0 && j == 2) { // First Column & Left Vertical Edge
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


				if (i % (subdivisions * 2) == subdivisions * 2 - 1) count = 0; // Rightmost column

				// tri pair
				if (n >= 0 && id0 < id1) { // Creating bending constraints
					int ni = n / 3;
					int nj = n % 3;
					int id2 = tris[i * 3 + (j + 2) % 3];
					int id3 = tris[ni * 3 + (nj + 2) % 3];

					if (j == 1) {
						if (evenCol) {
							triPairs[0].Add(id0);
							triPairs[0].Add(id1);
							triPairs[0].Add(id2);
							triPairs[0].Add(id3);
						} else {
							triPairs[2].Add(id0);
							triPairs[2].Add(id1);
							triPairs[2].Add(id2);
							triPairs[2].Add(id3);
						}
					} else {
						if (topRow) {
							if (evenCol) {
								triPairs[1].Add(id0);
								triPairs[1].Add(id1);
								triPairs[1].Add(id2);
								triPairs[1].Add(id3);
							} else {
								triPairs[3].Add(id0);
								triPairs[3].Add(id1);
								triPairs[3].Add(id2);
								triPairs[3].Add(id3);
							}
						} else {
							if (count == 0) {
								if (evenCol) {
									triPairs[4].Add(id0);
									triPairs[4].Add(id1);
									triPairs[4].Add(id2);
									triPairs[4].Add(id3);
								} else {
									triPairs[5].Add(id0);
									triPairs[5].Add(id1);
									triPairs[5].Add(id2);
									triPairs[5].Add(id3);
								}
							} else {
								if (evenCol) {
									if (evenRow) {
										triPairs[3].Add(id0);
										triPairs[3].Add(id1);
										triPairs[3].Add(id2);
										triPairs[3].Add(id3);
									} else {
										triPairs[1].Add(id0);
										triPairs[1].Add(id1);
										triPairs[1].Add(id2);
										triPairs[1].Add(id3);
									}
								} else {
									if (evenRow) {
										triPairs[1].Add(id0);
										triPairs[1].Add(id1);
										triPairs[1].Add(id2);
										triPairs[1].Add(id3);
									} else {
										triPairs[3].Add(id0);
										triPairs[3].Add(id1);
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
				d0[i][j] = (x[id0] - x[id1]).magnitude;
			}
		}

		dihedral0 = new float[bendingIDs.Length][];
		for (int i = 0; i < dihedral0.Length; i++) {
			dihedral0[i] = new float[bendingIDs[i].Length / 4];
			for (int j = 0; j < dihedral0[i].Length; j++) {
				int id0 = bendingIDs[i][4 * j + 2];
				int id1 = bendingIDs[i][4 * j + 3];
				dihedral0[i][j] = (x[id0] - x[id1]).magnitude;
			}
		}

		restPos = x;

		isInitialized = true;
	}

	private int[] FindTriNeighbors(int[] tris) {

		int numTris = tris.Length / 6;
		List<int[]> edges = new List<int[]>();

		for (int i = 0; i < numTris; i++) {
			for (int j = 0; j < 3; j++) {
				int id0 = tris[i * 3 + j];
				int id1 = tris[i * 3 + (j + 1) % 3];
				edges.Add(new int[] { Mathf.Min(id0, id1), Mathf.Max(id0, id1), i * 3 + j });
			}
		}

		edges.Sort((a, b) => {
			if (a[0] != b[0])
				return a[0].CompareTo(b[0]); // Compare by the first number
			return a[1].CompareTo(b[1]);     // Compare by the second number if the first numbers are equal
		});

		int[] neighbors = new int[3 * numTris];
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

		return neighbors;
	}

	void OnDestroy() {
		
	}

	private void OnDisable() {
		
	}

	private void OnEnable() {
		/*if (isInitialized) {
			CreateBuffers();
		}*/
	}
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

		//int start = (int)Mathf.Log(pos.Length) + 4;
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

		for (int xi = x0; xi <= x1; xi++) {
			for (int yi = y0; yi <= y1; yi++) {
				for (int zi = z0; zi <= z1; zi++) {
					int h = HashCoords(xi, yi, zi);
					int start = cellStart[h];
					int end = cellStart[h + 1];

					for (int i = start; i < end; i++) {
						queryIds[querySize] = cellEntries[i];
						querySize++;
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