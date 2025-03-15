using System;
using System.Runtime.InteropServices;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.Collections;
using Unity.VisualScripting.FullSerializer;
using UnityEngine;
using Unity.Collections.LowLevel.Unsafe;

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

	private Vector3[] x;
	NativeArray<Vector3> xNative;
	NativeArray<float> wNative;
	NativeArray<Vector3> vNative;
	private NativeArray<int> lengthsNative;
	private NativeArray<int> startIndexNative;
	Vector3[] stepVelocities;
	Vector3[] localGravityVectors;
	Vector3[] localWindVectors;
	List<List<float>> d0 = new List<List<float>>();
	List<List<float>> dihedral0 = new List<List<float>>();
	List<List<int>> stretchingIDs = new List<List<int>>();
	List<List<int>> bendingIDs = new List<List<int>>();

	private bool isInitialized = false;
	private List<int> simulatedCloths = new List<int>();
	int stretchBatches = 6;
	int bendingBatches = 6;
	int activeVerts = 0;
	int activeCloths = 0;

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
			CreateBuffers();
		}

		if (simulatedCloths.Count == 0) return;

		float dt = Time.fixedDeltaTime;
		float dt_step = dt / substeps;

		float maxVelocity = 1f * substeps;//thickness / dt_step;

		for (int i = 0; i < simulatedCloths.Count; i++) {
			// How far has the cloth object moved in since the last fixed update divided by the number of substeps
			stepVelocities[i] = cloths[simulatedCloths[i]].transform.InverseTransformVector((cloths[simulatedCloths[i]].transform.position - clothsLastPosition[simulatedCloths[i]]) / substeps);

			// Direction of gravity for the cloth
			localGravityVectors[i] = applyGravity ? cloths[simulatedCloths[i]].transform.InverseTransformVector(Physics.gravity) : Vector3.zero;

			// Direction of wind for the cloth
			localWindVectors[i] = applyWind ? cloths[simulatedCloths[i]].transform.InverseTransformVector(Vector3.back) * windSpeed * UnityEngine.Random.Range(0.8f, 1.2f) + cloths[simulatedCloths[i]].transform.InverseTransformVector(Vector3.up) * UnityEngine.Random.Range(-1f, 1f) * 4f + cloths[simulatedCloths[i]].transform.InverseTransformVector(Vector3.right) * UnityEngine.Random.Range(-1f, 1f) * 4f : Vector3.zero;
		}

		for (int i = 0; i < cloths.Length; i++) {
			clothsLastPosition[i] = cloths[i].transform.position;
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
		clothCompute.SetInt("numVertices", activeVerts);
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
			ComputeHelper.Dispatch(clothCompute, activeVerts, kernelIndex: predictPositionKernel);

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
			ComputeHelper.Dispatch(clothCompute, activeVerts, kernelIndex: updateVelocityKernel);
		}

		// Read new positions from GPU
		ComputeHelper.ReadDataFromBuffer(xBuffer, x, isAppendBuffer: false);

		// Update positions for each cloth
		for (int i = 0; i < lengthsNative.Length; i++) {
			// Copy specific cloth's positions from global position array
			Array.Copy(x, startIndexNative[i], cloths[simulatedCloths[i]].x, 0, lengthsNative[i]);
		}
	}


	private void InitializeDispatcher() {

		d0?.Clear();
		dihedral0?.Clear();
		stretchingIDs?.Clear();
		bendingIDs?.Clear();

		for (int i = 0; i < stretchBatches; i++) {
			d0.Add(new List<float>());
			stretchingIDs.Add(new List<int>());
		}

		for (int i = 0; i < bendingBatches; i++) {
			dihedral0.Add(new List<float>());
			bendingIDs.Add(new List<int>());
		};

		int totalVerts = 0;
		activeCloths = 0;

		clothsLastPosition = new Vector3[cloths.Length];

		for (int i = 0; i < cloths.Length; i++) {
			clothsLastPosition[i] = cloths[i].transform.position;

			if (cloths[i].gameObject.activeInHierarchy) {
				totalVerts += cloths[i].x.Length;
				activeCloths++;
			}
		}


		if (!CreateBuffers()) {
			Debug.Log("No active cloths");
			return;
		}

		isInitialized = true;
		Debug.Log("Initialized");
	}

	private bool CreateBuffers() {
		Debug.Log("Creating Buffers");
		Release();
		simulatedCloths?.Clear();
		if (xNative.IsCreated) xNative.Dispose();
		if (wNative.IsCreated) wNative.Dispose();
		if (vNative.IsCreated) vNative.Dispose();
		if (lengthsNative.IsCreated) lengthsNative.Dispose();
		if (startIndexNative.IsCreated) startIndexNative.Dispose();

		if (activeCloths == 0) return false;

		activeVerts = 0;
		for (int i = 0; i < cloths.Length; i++) {
			if (cloths[i].isInitialized && cloths[i].meshRenderer.isVisible) {
				simulatedCloths.Add(i);
				activeVerts += cloths[i].x.Length;
				cloths[i].isSimulating = true;
			} else {
				cloths[i].isSimulating = false;
			}
		}

		if (simulatedCloths.Count == 0) return false; // Return if no cloths are active/initialized/visible

		x = new Vector3[activeVerts];

		stepVelocities = new Vector3[simulatedCloths.Count];
		localGravityVectors = new Vector3[simulatedCloths.Count];
		localWindVectors = new Vector3[simulatedCloths.Count];

		xNative = new NativeArray<Vector3>(activeVerts, Allocator.Persistent);
		wNative = new NativeArray<float>(activeVerts, Allocator.Persistent);
		lengthsNative = new NativeArray<int>(simulatedCloths.Count, Allocator.Persistent);
		startIndexNative = new NativeArray<int>(simulatedCloths.Count, Allocator.Persistent);

		for (int i = 0; i < stretchBatches; i++) {
			d0[i].Clear();
			stretchingIDs[i].Clear();
		}

		for (int i = 0; i < bendingBatches; i++) {
			dihedral0[i].Clear();
			bendingIDs[i].Clear();
		};

		for (int i = 0; i < simulatedCloths.Count; i++) {
			lengthsNative[i] = cloths[simulatedCloths[i]].x.Length;
			
			if (i == 0) {
				startIndexNative[i] = 0;
			} else {
				startIndexNative[i] = startIndexNative[i - 1] + lengthsNative[i - 1];
			}

			unsafe {
				int size = Marshal.SizeOf(typeof(Vector3)) * lengthsNative[i];
				IntPtr sourcePtr = Marshal.UnsafeAddrOfPinnedArrayElement(cloths[simulatedCloths[i]].x, 0);
				IntPtr destPtr = (IntPtr)((Vector3*)xNative.GetUnsafePtr() + startIndexNative[i]); // Calculate offset for the subset
				UnsafeUtility.MemCpy((void*)destPtr, (void*)sourcePtr, size);

				size = Marshal.SizeOf(typeof(float)) * lengthsNative[i];
				sourcePtr = Marshal.UnsafeAddrOfPinnedArrayElement(cloths[simulatedCloths[i]].w, 0);
				destPtr = (IntPtr)((float*)wNative.GetUnsafePtr() + startIndexNative[i]); // Calculate offset for the subset
				UnsafeUtility.MemCpy((void*)destPtr, (void*)sourcePtr, size);
			}

			for (int j = 0; j < stretchBatches; j++) {
				d0[j].AddRange(cloths[simulatedCloths[i]].d0[j]);
			}

			for (int j = 0; j < bendingBatches; j++) {
				dihedral0[j].AddRange(cloths[simulatedCloths[i]].dihedral0[j]);
			}

			for (int j = 0; j < stretchBatches; j++) {
				for (int k = 0; k < cloths[simulatedCloths[i]].stretchingIDs[j].Length; k++) {
					stretchingIDs[j].Add((cloths[simulatedCloths[i]].stretchingIDs[j])[k] + startIndexNative[i]);
				}
			}

			for (int j = 0; j < bendingBatches; j++) {
				for (int k = 0; k < cloths[simulatedCloths[i]].bendingIDs[j].Length; k++) {
					bendingIDs[j].Add((cloths[simulatedCloths[i]].bendingIDs[j])[k] + startIndexNative[i]);
				}
			}
		}

		vNative = new NativeArray<Vector3>(activeVerts, Allocator.Persistent);

		xBuffer = ComputeHelper.CreateStructuredBuffer<Vector3>(activeVerts);
		xBuffer.SetData(xNative);

		vBuffer = ComputeHelper.CreateStructuredBuffer<Vector3>(activeVerts);
		vBuffer.SetData(vNative);

		wBuffer = ComputeHelper.CreateStructuredBuffer<float>(activeVerts);
		wBuffer.SetData(wNative);

		pBuffer = ComputeHelper.CreateStructuredBuffer<Vector3>(activeVerts);

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

		stepVelocityBuffer = ComputeHelper.CreateStructuredBuffer<Vector3>(simulatedCloths.Count);
		gravityVectorBuffer = ComputeHelper.CreateStructuredBuffer<Vector3>(simulatedCloths.Count);
		windVectorBuffer = ComputeHelper.CreateStructuredBuffer<Vector3>(simulatedCloths.Count);
		startIndicesBuffer = ComputeHelper.CreateStructuredBuffer<int>(simulatedCloths.Count);

		startIndicesBuffer.SetData(startIndexNative.ToArray());
		clothCompute.SetBuffer(updateVelocityKernel, "startIndex", startIndicesBuffer);
		clothCompute.SetBuffer(predictPositionKernel, "startIndex", startIndicesBuffer);

		//xNative.Dispose();
		//w.Dispose();
		//v.Dispose();
		//lengths.Dispose();
		//startIndex.Dispose();

		return true;
	}

	void OnDestroy() {
		Release();
		if (xNative.IsCreated) xNative.Dispose();
		if (wNative.IsCreated) wNative.Dispose();
		if (vNative.IsCreated) vNative.Dispose();
		if (lengthsNative.IsCreated) lengthsNative.Dispose();
		if (startIndexNative.IsCreated) startIndexNative.Dispose();
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
