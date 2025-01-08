using NUnit.Framework.Constraints;
using NUnit.Framework.Internal;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using static UnityEngine.InputManagerEntry;

public class ClothSimulation : MonoBehaviour
{
    [Range(0, 0.0001f)]
    public float stretchingCompliance = 0f;
	[Range(0, 10)]
	public float bendingCompliance = 0.5f;
    public float density = 0.1f; // kg / m^2
	public int substeps = 5;

    private Vector3[] x;
    private Vector3[] v;
    private float[] w;
	private float[] d0;
	private float[] dihedral0;
	private int[] bendingIDs;
	private int[] neighbors;
	private int[] stretchingIDs;
	private Vector3 lastPosition;

    private MeshFilter meshFilter;
    private MeshRenderer meshRenderer;
	Mesh mesh;

	// Start is called before the first frame update
	void Start()
    {
		meshFilter = GetComponent<MeshFilter>();
		meshRenderer = GetComponent<MeshRenderer>();
		lastPosition = transform.position;

		Init();
    }

    // Update is called once per frame
    void Update()
    {
        
    }

	private void FixedUpdate() {
        float dt = Time.fixedDeltaTime;
		float dt_step = dt / substeps;

		Vector3 stepVelocity = transform.InverseTransformVector((transform.position - lastPosition) / substeps);
		lastPosition = transform.position;

		for (int step = 0; step < substeps; step++) {

			Vector3[] p = new Vector3[x.Length];
			for (int i = 0; i < x.Length; i++) {
				v[i] += dt_step * w[i] * transform.InverseTransformVector(Physics.gravity); // Applies gravity
				p[i] = x[i] + dt_step * v[i] - w[i] * stepVelocity;
			}

			SolveStretching(stretchingCompliance, dt_step, p);

			SolveBending(bendingCompliance, dt_step, p);

			// loop solverIterations

			for (int i = 0; i < x.Length; i++) {
				v[i] = (p[i] - x[i] + w[i] * stepVelocity) / dt_step;
				v[i] *= (1 - 0.03f / substeps); // Damp Velocities
				x[i] = p[i];
			}
		}

		
		mesh.vertices = x;
	}

	private void SolveStretching(float compliance, float dt, Vector3[] p) {

		float alpha = compliance / dt / dt;

		for (int i = 0; i < d0.Length; i++) {
			int id0 = stretchingIDs[i * 2];
			int id1 = stretchingIDs[i * 2 + 1];
			float w0 = w[id0];
			float w1 = w[id1];
			float wT = w0 + w1;
			if (wT == 0) continue;

			Vector3 p0 = p[id0];
			Vector3 p1 = p[id1];
			float len = (p0 - p1).magnitude;
			if (len == 0) continue;

			float C = len - d0[i];
			float s = -C / (wT + alpha);

			p[id0] += (p0 - p1).normalized * s * w0;
			p[id1] += (p0 - p1).normalized * -s * w1;
		}
	}

	private void SolveBending(float compliance, float dt, Vector3[] p) {

		float alpha = compliance / dt / dt;

		for (int i = 0; i < dihedral0.Length; i++) {
			int id0 = bendingIDs[i * 4];
			int id1 = bendingIDs[i * 4 + 1];
			int id2 = bendingIDs[i * 4 + 2];
			int id3 = bendingIDs[i * 4 + 3];

			/*Vector3 p1 = p[id0];
			Vector3 p2 = p[id1] - p1;
			Vector3 p3 = p[id2] - p1;
			Vector3 p4 = p[id3] - p1;

			float w1 = w[id0];
			float w2 = w[id1];
			float w3 = w[id2];
			float w4 = w[id3];

			Vector3 n1 = Vector3.Cross(p2, p3).normalized;
			Vector3 n2 = Vector3.Cross(p2, p4).normalized;

			float d = Vector3.Dot(n1, n2);

			Vector3 q3 = (Vector3.Cross(p2, n2) + Vector3.Cross(n1, p2) * d) / Vector3.Cross(p2, p3).magnitude;
			Vector3 q4 = (Vector3.Cross(p2, n1) + Vector3.Cross(n2, p2) * d) / Vector3.Cross(p2, p4).magnitude;
			Vector3 q2 = -(Vector3.Cross(p3, n2) + Vector3.Cross(n1, p3) * d) / Vector3.Cross(p2, p3).magnitude - (Vector3.Cross(p4, n1) + Vector3.Cross(n2, p4) * d) / Vector3.Cross(p2, p4).magnitude;
			Vector3 q1 = -q2 - q3 - q4;

			float bottom = (w1 * q1.magnitude * q1.magnitude + w2 * q2.magnitude * q2.magnitude + w3 * q3.magnitude * q3.magnitude + w4 * q4.magnitude * q4.magnitude);

			float temp = 0;
			if (bottom != 0) {
				temp = Mathf.Sqrt(1 - d * d) * (Mathf.Acos(d) - dihedral0[i]) / bottom;
			}

			Vector3 dp1 = -w1 * temp * q1;
			Vector3 dp2 = -w2 * temp * q2;
			Vector3 dp3 = -w3 * temp * q3;
			Vector3 dp4 = -w4 * temp * q4;

			p[id0] += dp1 * bendingCompliance;
			p[id1] += dp2 * bendingCompliance;
			p[id2] += dp3 * bendingCompliance;
			p[id3] += dp4 * bendingCompliance;*/

			float w0 = w[id2];
			float w1 = w[id3];
			float wT = w0 + w1;
			if (wT == 0) continue;

			Vector3 p0 = p[id2];
			Vector3 p1 = p[id3];
			float len = (p0 - p1).magnitude;
			if (len == 0) continue;

			float C = len - dihedral0[i];
			float s = -C / (wT + alpha);

			p[id2] += (p0 - p1).normalized * s * w0;
			p[id3] += (p0 - p1).normalized * -s * w1;
		}

		
	}

    public void Init() {

		float width = 10f;
		float height = 10f;
		int subdivisions = 25;


		// Create an empty Mesh
		mesh = new Mesh();

		// Calculate the number of vertices
		int vertexCount = (subdivisions + 1) * (subdivisions + 1);


		x = new Vector3[vertexCount];
		v = new Vector3[vertexCount];
		w = new float[vertexCount];

		// Calculate vertex positions and UV coordinates
		for (int i = 0; i < subdivisions + 1; i++) {
			for (int j = 0; j < subdivisions + 1; j++) {
				x[j + i * (subdivisions + 1)] = new Vector3(j / (float)subdivisions * width, 0, height - i / (float)subdivisions * height);

				w[j + i * (subdivisions + 1)] = 1f;
			}
		}

		w[0] = 0;
		w[subdivisions] = 0;
		w[w.Length - 1] = 0;
		w[w.Length - 1 - subdivisions] = 0;

		// Create arrays to store triangle indices
		int[] tris = new int[subdivisions * subdivisions * 6];
		int triangleIndex = 0;

		// Generate triangle indices
		for (int i = 0; i < subdivisions; i++) {
			for (int j = 0; j < subdivisions; j++) {
				tris[triangleIndex++] = i * (subdivisions + 1) + j;
				tris[triangleIndex++] = i * (subdivisions + 1) + 1 + j;
				tris[triangleIndex++] = (i + 1) * (subdivisions + 1) + j;

				tris[triangleIndex++] = i * (subdivisions + 1) + 1 + j;
				tris[triangleIndex++] = (i + 1) * (subdivisions + 1) + j + 1;
				tris[triangleIndex++] = (i + 1) * (subdivisions + 1) + j;
			}
		}

		// Assign the calculated data to the mesh
		mesh.vertices = x;
		mesh.triangles = tris;
		mesh.RecalculateNormals();
		mesh.RecalculateTangents();

		// Assign the mesh to the GameObject
		meshFilter.mesh = mesh;


		// Generate default distances
		int numTris = tris.Length / 3;
		/*d0 = new float[tris.Length];
		for (int i = 0; i < numTris; i++) {
			Vector3 A = x[tris[i * 3]];
			Vector3 B = x[tris[i * 3 + 1]];
			Vector3 C = x[tris[i * 3 + 2]];

			d0[i * 3] = (A - B).magnitude;
			d0[i * 3 + 1] = (B - C).magnitude;
			d0[i * 3 + 2] = (C - A).magnitude;
		}*/

		// Generate Triangle Pairs
		/*for (int i = 0; i < numTris; i++) {
			int A1 = tris[i * 3];
			int A2 = tris[i * 3 + 1];
			int A3 = tris[i * 3 + 2];

			int numMatches = 0;
			for (int j = i + 1; j < numTris; j++) {
				int B1 = tris[j * 3];
				int B2 = tris[j * 3 + 1];
				int B3 = tris[j * 3 + 2];

				if (A1 == B1) {
					if (A2 == B2) {
						numMatches++;
						triPairs.Add(new int[] { A1, A2, A3, B3 });
						dihedral.Add(Mathf.PI);
					} else if (A2 == B3) {
						numMatches++;
						triPairs.Add(new int[] { A1, A2, A3, B2 });
						dihedral.Add(Mathf.PI);
					} else if (A3 == B2) {
						numMatches++;
						triPairs.Add(new int[] { A1, A3, A2, B3 });
						dihedral.Add(Mathf.PI);
					} else if (A3 == B3) {
						numMatches++;
						triPairs.Add(new int[] { A1, A3, A2, B2 });
						dihedral.Add(Mathf.PI);
					}
				} else if (A1 == B2) {
					if (A2 == B1) {
						numMatches++;
						triPairs.Add(new int[] { A1, A2, A3, B3 });
						dihedral.Add(Mathf.PI);
					} else if (A2 == B3) {
						numMatches++;
						triPairs.Add(new int[] { A1, A2, A3, B1 });
						dihedral.Add(Mathf.PI);
					} else if (A3 == B1) {
						numMatches++;
						triPairs.Add(new int[] { A1, A3, A2, B3 });
						dihedral.Add(Mathf.PI);
					} else if (A3 == B3) {
						numMatches++;
						triPairs.Add(new int[] { A1, A3, A2, B1 });
						dihedral.Add(Mathf.PI);
					}
				} else if (A1 == B3) {
					if (A2 == B1) {
						numMatches++;
						triPairs.Add(new int[] { A1, A2, A3, B2 });
						dihedral.Add(Mathf.PI);
					} else if (A2 == B2) {
						numMatches++;
						triPairs.Add(new int[] { A1, A2, A3, B1 });
						dihedral.Add(Mathf.PI);
					} else if (A3 == B1) {
						numMatches++;
						triPairs.Add(new int[] { A1, A3, A2, B2 });
						dihedral.Add(Mathf.PI);
					} else if (A3 == B2) {
						numMatches++;
						triPairs.Add(new int[] { A1, A3, A2, B1 });
						dihedral.Add(Mathf.PI);
					}
				} else if (A2 == B1) {
					if (A1 == B2) {
						numMatches++;
						triPairs.Add(new int[] { A2, A1, A3, B3 });
						dihedral.Add(Mathf.PI);
					} else if (A1 == B3) {
						numMatches++;
						triPairs.Add(new int[] { A2, A1, A3, B2 });
						dihedral.Add(Mathf.PI);
					} else if (A3 == B2) {
						numMatches++;
						triPairs.Add(new int[] { A2, A3, A1, B3 });
						dihedral.Add(Mathf.PI);
					} else if (A3 == B3) {
						numMatches++;
						triPairs.Add(new int[] { A2, A3, A1, B2 });
						dihedral.Add(Mathf.PI);
					}
				} else if (A2 == B2) {
					if (A1 == B1) {
						numMatches++;
						triPairs.Add(new int[] { A2, A1, A3, B3 });
						dihedral.Add(Mathf.PI);
					} else if (A1 == B3) {
						numMatches++;
						triPairs.Add(new int[] { A2, A1, A3, B1 });
						dihedral.Add(Mathf.PI);
					} else if (A3 == B1) {
						numMatches++;
						triPairs.Add(new int[] { A2, A3, A1, B3 });
						dihedral.Add(Mathf.PI);
					} else if (A3 == B3) {
						numMatches++;
						triPairs.Add(new int[] { A2, A3, A1, B1 });
						dihedral.Add(Mathf.PI);
					}
				} else if (A2 == B3) {
					if (A1 == B1) {
						numMatches++;
						triPairs.Add(new int[] { A2, A1, A3, B2 });
						dihedral.Add(Mathf.PI);
					} else if (A1 == B2) {
						numMatches++;
						triPairs.Add(new int[] { A2, A1, A3, B1 });
						dihedral.Add(Mathf.PI);
					} else if (A3 == B1) {
						numMatches++;
						triPairs.Add(new int[] { A2, A3, A1, B2 });
						dihedral.Add(Mathf.PI);
					} else if (A3 == B2) {
						numMatches++;
						triPairs.Add(new int[] { A2, A3, A1, B1 });
						dihedral.Add(Mathf.PI);
					}
				}
				// No point checking third point, need 2 matching points

				if (numMatches > 2) { // Triangle can only have 3 adjacent triangles at most
					break;
				}
			}
		}
		*/

		neighbors = FindTriNeighbors(tris);


		List<int> triPairs = new List<int>();
		List<int> edges = new List<int>();
		for (int i = 0; i < numTris; i++) {
			for (int j = 0; j < 3; j++) {
				int id0 = tris[i * 3 + j];
				int id1 = tris[i * 3 + (j + 1) % 3];

				// each edge only once
				int n = neighbors[i * 3 + j];
				if (n < 0 || id0 < id1) {
					edges.Add(id0); // Creating edge constraints
					edges.Add(id1);
				}

				// tri pair
				if (n >= 0 && id0 < id1) { // Creating bending constraints
					int ni = n / 3;
					int nj = n % 3;
					int id2 = tris[i * 3 + (j + 2) % 3];
					int id3 = tris[ni * 3 + (nj + 2) % 3];
					triPairs.Add(id0);
					triPairs.Add(id1);
					triPairs.Add(id2);
					triPairs.Add(id3);
				}
			}
		}

		stretchingIDs = edges.ToArray();
		bendingIDs = triPairs.ToArray();

		d0 = new float[stretchingIDs.Length / 2];
		for (int i = 0; i < d0.Length; i++) {
			int id0 = stretchingIDs[2 * i];
			int id1 = stretchingIDs[2 * i + 1];
			d0[i] = (x[id0] - x[id1]).magnitude;
		}

		dihedral0 = new float[bendingIDs.Length / 4];
		for (int i = 0; i < dihedral0.Length; i++) {
			//dihedral0[i] = (Mathf.PI);
			dihedral0[i] = (x[bendingIDs[i * 4 + 2]] - x[bendingIDs[i * 4 + 3]]).magnitude;
		}
	}

	private int[] FindTriNeighbors(int[] tris) {

		int numTris = tris.Length / 3;
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
}
