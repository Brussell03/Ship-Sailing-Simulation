using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class ClothDispatcher : MonoBehaviour
{
    public ClothSimulation[] cloths;
    public ComputeShader clothCompute;

	[Range(0, 0.0001f)]
	public float stretchingCompliance = 0f;
	[Range(0, 10)]
	public float bendingCompliance = 0.5f;
	public float density = 0.1f; // kg / m^2
	[Range(3, 20)]
	public int substeps = 5;
	public float thickness = 0.004f; // m
	[Range(0, 50)]
	public float windSpeed = 1f; // m/s
	public bool handleCollisions = true;
	public bool applyGravity = true;
	public bool applyWind = true;
	[Min(1)]
	public int projectionIterations = 3;
	public float damping = 0.03f;
	private Vector3[] clothsLastPosition;

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

	ComputeBuffer xBuffer;
	ComputeBuffer vBuffer;
	ComputeBuffer pBuffer;
	ComputeBuffer wBuffer;
	ComputeBuffer stepVelocityBuffer;
	ComputeBuffer gravityVectorBuffer;
	ComputeBuffer windVectorBuffer;
	ComputeBuffer startIndicesBuffer;
	ComputeBuffer[] d0Buffers;
	ComputeBuffer[] dihedral0Buffers;
	ComputeBuffer[] bendingIDsBuffers;
	ComputeBuffer[] stretchingIDsBuffers;
	
	List<int> lengths = new List<int>();
	List<int> startIndex = new List<int>();
	private Vector3[] x;
	Vector3[] stepVelocities;
	Vector3[] localGravityVectors;
	Vector3[] localWindVectors;
	List<float> w = new List<float>();
	List<List<float>> d0 = new List<List<float>>();
	List<List<float>> dihedral0 = new List<List<float>>();
	List<List<int>> stretchingIDs = new List<List<int>>();
	List<List<int>> bendingIDs = new List<List<int>>();

	private bool isInitialized = false;
	private Vector3[][] clothsX;
	private List<int> simulatedCloths;

	// Start is called before the first frame update
	void Start()
    {
		
	}

    // Update is called once per frame
    void FixedUpdate()
    {
		if (cloths.Length == 0) return;

		if (!isInitialized) {

			InitializeDispatcher();
		}

		bool clothStateChanged = false;
		for (int i = 0; i < cloths.Length; i++) {
			if ((cloths[i].isInitialized && cloths[i].meshRenderer.isVisible) != cloths[i].isSimulating) {
				clothStateChanged = true;
				break;
			}
		}

		if (clothStateChanged) {
			Debug.Log("Cloth state changed");
			InitializeDispatcher();
		}

		if (simulatedCloths.Count == 0) return;

		float dt = Time.fixedDeltaTime;
		float dt_step = dt / substeps;

		float maxVelocity = 1f * substeps;//thickness / dt_step;

		for (int i = 0; i < simulatedCloths.Count; i++) {
			// How far has the cloth object moved in since the last fixed update divided by the number of substeps
			stepVelocities[i] = cloths[simulatedCloths[i]].transform.InverseTransformVector((cloths[simulatedCloths[i]].transform.position - clothsLastPosition[i]) / substeps);
			clothsLastPosition[i] = cloths[simulatedCloths[i]].transform.position;

			// Direction of gravity for the cloth
			localGravityVectors[i] = applyGravity ? cloths[simulatedCloths[i]].transform.InverseTransformVector(Physics.gravity) : Vector3.zero;

			// Direction of wind for the cloth
			float windChaos = UnityEngine.Random.Range(0 , windSpeed / 3);
			localWindVectors[i] = applyWind ? cloths[simulatedCloths[i]].transform.InverseTransformVector(Vector3.back) * (windSpeed + (Mathf.PingPong(Time.time * 5, windChaos) - windChaos)) : Vector3.zero;
		}

		//float maxVelocity = 3; // m/s

		/*if (handleCollisions) {
			hash.Create(x);
			//float maxTravelDist = maxVelocity * dt;
			float maxTravelDist = thickness;
			hash.QueryAll(x, maxTravelDist);
		}*/

		stepVelocityBuffer.SetData(stepVelocities);
		gravityVectorBuffer.SetData(localGravityVectors);
		windVectorBuffer.SetData(localWindVectors);

		clothCompute.SetInt("numSubsteps", substeps);
		clothCompute.SetInt("numVertices", x.Length);
		clothCompute.SetInt("numCloths", simulatedCloths.Count);
		clothCompute.SetFloat("projectionIterations", projectionIterations);
		clothCompute.SetBuffer(updateVelocityKernel, "stepVelocity", stepVelocityBuffer);
		clothCompute.SetBuffer(predictPositionKernel, "stepVelocity", stepVelocityBuffer);
		clothCompute.SetBuffer(predictPositionKernel, "gravityVector", gravityVectorBuffer);
		clothCompute.SetBuffer(predictPositionKernel, "windVector", windVectorBuffer);
		clothCompute.SetFloat("dt_step", dt_step);
		clothCompute.SetFloat("damping", damping);
		clothCompute.SetFloat("maxVelocity", maxVelocity);
		clothCompute.SetFloat("stretchingAlpha", stretchingCompliance / (dt_step * dt_step));
		clothCompute.SetFloat("bendingAlpha", bendingCompliance / (dt_step * dt_step));

		for (int step = 0; step < substeps; step++) {

			// Predict new positions
			ComputeHelper.Dispatch(clothCompute, x.Length, kernelIndex: predictPositionKernel);

			// Solve stretching constraint for each batch of edges
			ComputeHelper.Dispatch(clothCompute, d0[0].Count, kernelIndex: solveStretchingKernel1);
			ComputeHelper.Dispatch(clothCompute, d0[1].Count, kernelIndex: solveStretchingKernel2);
			ComputeHelper.Dispatch(clothCompute, d0[2].Count, kernelIndex: solveStretchingKernel3);
			ComputeHelper.Dispatch(clothCompute, d0[3].Count, kernelIndex: solveStretchingKernel4);
			ComputeHelper.Dispatch(clothCompute, d0[4].Count, kernelIndex: solveStretchingKernel5);
			ComputeHelper.Dispatch(clothCompute, d0[5].Count, kernelIndex: solveStretchingKernel6);

			// Solve bending constraint for each batch of triangle pairs
			ComputeHelper.Dispatch(clothCompute, dihedral0[0].Count, kernelIndex: solveBendingKernel1);
			ComputeHelper.Dispatch(clothCompute, dihedral0[1].Count, kernelIndex: solveBendingKernel2);
			ComputeHelper.Dispatch(clothCompute, dihedral0[2].Count, kernelIndex: solveBendingKernel3);
			ComputeHelper.Dispatch(clothCompute, dihedral0[3].Count, kernelIndex: solveBendingKernel4);
			ComputeHelper.Dispatch(clothCompute, dihedral0[4].Count, kernelIndex: solveBendingKernel5);
			ComputeHelper.Dispatch(clothCompute, dihedral0[5].Count, kernelIndex: solveBendingKernel6);

			// Update velocities
			ComputeHelper.Dispatch(clothCompute, x.Length, kernelIndex: updateVelocityKernel);
		}

		// Read new positions from GPU
		ComputeHelper.ReadDataFromBuffer<Vector3>(xBuffer, x, isAppendBuffer: false);

		// Update positions for each cloth
		for (int i = 0; i < lengths.Count; i++) {
			// Copy specific cloth's positions from global position array
			Array.Copy(x, startIndex[i], clothsX[i], 0, lengths[i]);

			// Set cloth's positions
			cloths[simulatedCloths[i]].x = clothsX[i];
		}
	}

	private void InitializeDispatcher() {
		if (!CreateBuffers()) {
			Debug.Log("No active cloths");
			return;
		}

		clothsLastPosition = new Vector3[simulatedCloths.Count];
		stepVelocities = new Vector3[simulatedCloths.Count];
		localGravityVectors = new Vector3[simulatedCloths.Count];
		localWindVectors = new Vector3[simulatedCloths.Count];

		for (int i = 0; i < simulatedCloths.Count; i++) { // Make this keep its previous last position when the cloth was last simulation, OR maybe need to always track cloth's movements?
			clothsLastPosition[i] = cloths[simulatedCloths[i]].transform.position;
		}

		isInitialized = true;
		Debug.Log("Initialized");
	}

	private bool CreateBuffers() {
		Debug.Log("Creating Buffers");
		Release();
		lengths?.Clear();
		simulatedCloths?.Clear();
		startIndex?.Clear();

		if (cloths.Length == 0) return false;

		int maxStretchBatches = 6;
		int maxBendingBatches = 6;

		simulatedCloths = new List<int>();
		for (int i = 0; i < cloths.Length; i++) {
			if (cloths[i].isInitialized && cloths[i].meshRenderer.isVisible) {
				simulatedCloths.Add(i);
				cloths[i].isSimulating = true;
			} else {
				cloths[i].isSimulating = false;
			}
		}

		if (simulatedCloths.Count == 0) return false; // Return if no cloths are active/initialized/visible

		List<Vector3> x = new List<Vector3>();
		w = new List<float>();
		d0 = new List<List<float>>();
		dihedral0 = new List<List<float>>();
		stretchingIDs = new List<List<int>>();
		bendingIDs = new List<List<int>>();
		clothsX = new Vector3[simulatedCloths.Count][];

		for (int i = 0; i < maxStretchBatches; i++) {
			d0.Add(new List<float>());
		}
		for (int i = 0; i < maxBendingBatches; i++) {
			dihedral0.Add(new List<float>());
		};
		for (int i = 0; i < maxStretchBatches; i++) {
			stretchingIDs.Add(new List<int>());
		}
		for (int i = 0; i < maxBendingBatches; i++) {
			bendingIDs.Add(new List<int>());
		}

		for (int i = 0; i < simulatedCloths.Count; i++) {
			lengths.Add(cloths[simulatedCloths[i]].x.Length);
			
			if (i == 0) {
				startIndex.Add(0);
			} else {
				startIndex.Add(startIndex[i - 1] + lengths[i - 1]);
			}

			x.AddRange(cloths[simulatedCloths[i]].x);
			w.AddRange(cloths[simulatedCloths[i]].w);
			clothsX[i] = new Vector3[lengths[i]];

			for (int j = 0; j < maxStretchBatches; j++) {
				d0[j].AddRange(cloths[simulatedCloths[i]].d0[j]);
			}

			for (int j = 0; j < maxBendingBatches; j++) {
				dihedral0[j].AddRange(cloths[simulatedCloths[i]].dihedral0[j]);
			}

			for (int j = 0; j < maxStretchBatches; j++) {
				int[] clothIDs = new int[cloths[simulatedCloths[i]].stretchingIDs[j].Length];
				Array.Copy(cloths[simulatedCloths[i]].stretchingIDs[j], clothIDs, cloths[simulatedCloths[i]].stretchingIDs[j].Length);
				for (int k = 0; k < clothIDs.Length; k++) {
					clothIDs[k] += startIndex[i];
				}
				stretchingIDs[j].AddRange(clothIDs);
			}

			for (int j = 0; j < maxBendingBatches; j++) {
				int[] clothIDs = new int[cloths[simulatedCloths[i]].bendingIDs[j].Length];
				Array.Copy(cloths[simulatedCloths[i]].bendingIDs[j], clothIDs, cloths[simulatedCloths[i]].bendingIDs[j].Length);
				for (int k = 0; k < clothIDs.Length; k++) {
					clothIDs[k] += startIndex[i];
				}
				bendingIDs[j].AddRange(clothIDs);
			}
		}

		this.x = x.ToArray();

		Vector3[] v = new Vector3[x.Count];

		xBuffer = ComputeHelper.CreateStructuredBuffer<Vector3>(x.Count);
		xBuffer.SetData(this.x);

		vBuffer = ComputeHelper.CreateStructuredBuffer<Vector3>(v.Length);
		vBuffer.SetData(v);

		wBuffer = ComputeHelper.CreateStructuredBuffer<float>(w.Count);
		wBuffer.SetData(w.ToArray());

		pBuffer = ComputeHelper.CreateStructuredBuffer<Vector3>(x.Count);

		clothCompute.SetBuffer(updateVelocityKernel, "x", xBuffer);
		clothCompute.SetBuffer(updateVelocityKernel, "v", vBuffer);
		clothCompute.SetBuffer(updateVelocityKernel, "w", wBuffer);
		clothCompute.SetBuffer(updateVelocityKernel, "p", pBuffer);

		clothCompute.SetBuffer(predictPositionKernel, "x", xBuffer);
		clothCompute.SetBuffer(predictPositionKernel, "v", vBuffer);
		clothCompute.SetBuffer(predictPositionKernel, "w", wBuffer);
		clothCompute.SetBuffer(predictPositionKernel, "p", pBuffer);

		clothCompute.SetBuffer(solveStretchingKernel1, "w", wBuffer);
		clothCompute.SetBuffer(solveStretchingKernel1, "p", pBuffer);
		clothCompute.SetBuffer(solveStretchingKernel2, "w", wBuffer);
		clothCompute.SetBuffer(solveStretchingKernel2, "p", pBuffer);
		clothCompute.SetBuffer(solveStretchingKernel3, "w", wBuffer);
		clothCompute.SetBuffer(solveStretchingKernel3, "p", pBuffer);
		clothCompute.SetBuffer(solveStretchingKernel4, "w", wBuffer);
		clothCompute.SetBuffer(solveStretchingKernel4, "p", pBuffer);
		clothCompute.SetBuffer(solveStretchingKernel5, "w", wBuffer);
		clothCompute.SetBuffer(solveStretchingKernel5, "p", pBuffer);
		clothCompute.SetBuffer(solveStretchingKernel6, "w", wBuffer);
		clothCompute.SetBuffer(solveStretchingKernel6, "p", pBuffer);

		clothCompute.SetBuffer(solveBendingKernel1, "w", wBuffer);
		clothCompute.SetBuffer(solveBendingKernel1, "p", pBuffer);
		clothCompute.SetBuffer(solveBendingKernel2, "w", wBuffer);
		clothCompute.SetBuffer(solveBendingKernel2, "p", pBuffer);
		clothCompute.SetBuffer(solveBendingKernel3, "w", wBuffer);
		clothCompute.SetBuffer(solveBendingKernel3, "p", pBuffer);
		clothCompute.SetBuffer(solveBendingKernel4, "w", wBuffer);
		clothCompute.SetBuffer(solveBendingKernel4, "p", pBuffer);
		clothCompute.SetBuffer(solveBendingKernel5, "w", wBuffer);
		clothCompute.SetBuffer(solveBendingKernel5, "p", pBuffer);
		clothCompute.SetBuffer(solveBendingKernel6, "w", wBuffer);
		clothCompute.SetBuffer(solveBendingKernel6, "p", pBuffer);

		d0Buffers = new ComputeBuffer[maxStretchBatches];
		stretchingIDsBuffers = new ComputeBuffer[maxStretchBatches];
		for (int i = 0; i < maxStretchBatches; i++) {
			d0Buffers[i] = ComputeHelper.CreateStructuredBuffer<float>(d0[i].Count);
			d0Buffers[i].SetData(d0[i]);

			stretchingIDsBuffers[i] = ComputeHelper.CreateStructuredBuffer<int>(stretchingIDs[i].Count);
			stretchingIDsBuffers[i].SetData(stretchingIDs[i]);
		}

		dihedral0Buffers = new ComputeBuffer[maxBendingBatches];
		bendingIDsBuffers = new ComputeBuffer[maxBendingBatches];
		for (int i = 0; i < maxBendingBatches; i++) {
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

		stepVelocityBuffer = ComputeHelper.CreateStructuredBuffer<Vector3>(simulatedCloths.Count);
		gravityVectorBuffer = ComputeHelper.CreateStructuredBuffer<Vector3>(simulatedCloths.Count);
		windVectorBuffer = ComputeHelper.CreateStructuredBuffer<Vector3>(simulatedCloths.Count);
		startIndicesBuffer = ComputeHelper.CreateStructuredBuffer<int>(simulatedCloths.Count);

		startIndicesBuffer.SetData(startIndex.ToArray());
		clothCompute.SetBuffer(updateVelocityKernel, "startIndex", startIndicesBuffer);
		clothCompute.SetBuffer(predictPositionKernel, "startIndex", startIndicesBuffer);

		return true;
	}

	void OnDestroy() {
		Release();
	}

	private void OnDisable() {
		//Release();
	}

	private void OnEnable() {
		//Release();
		if (isInitialized) {
			//CreateBuffers();
		}
	}

	void Release() {
		ComputeHelper.Release(xBuffer, vBuffer, wBuffer, pBuffer, stepVelocityBuffer, gravityVectorBuffer, windVectorBuffer, startIndicesBuffer);
		ComputeHelper.Release(stretchingIDsBuffers);
		ComputeHelper.Release(d0Buffers);
		ComputeHelper.Release(bendingIDsBuffers);
		ComputeHelper.Release(dihedral0Buffers);
	}
}
