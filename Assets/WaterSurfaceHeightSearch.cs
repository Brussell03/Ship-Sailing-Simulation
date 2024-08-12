using System.Collections;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.VisualScripting.FullSerializer;
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;

public class WaterSurfaceHeightSearch
{
	// Public parameters
	//public int resolution = 50;
	public WaterSurface waterSurface = null;

	// Input job parameters
	NativeArray<float3> targetPositionBuffer;

	// Output job parameters
	NativeArray<float> errorBuffer;
	NativeArray<float3> candidatePositionBuffer;
	NativeArray<float3> projectedPositionWSBuffer;
	NativeArray<float3> directionBuffer;
	NativeArray<int> stepCountBuffer;

	// Start is called before the first frame update
	public WaterSurfaceHeightSearch(int gridSizeX, int gridSizeY, WaterSurface waterSurface) {
		this.waterSurface = waterSurface;

		// Allocate the buffers
		targetPositionBuffer = new NativeArray<float3>(gridSizeX * gridSizeY, Allocator.Persistent);
		errorBuffer = new NativeArray<float>(gridSizeX * gridSizeY, Allocator.Persistent);
		candidatePositionBuffer = new NativeArray<float3>(gridSizeX * gridSizeY, Allocator.Persistent);
		projectedPositionWSBuffer = new NativeArray<float3>(gridSizeX * gridSizeY, Allocator.Persistent);
		directionBuffer = new NativeArray<float3>(gridSizeX * gridSizeY, Allocator.Persistent);
		stepCountBuffer = new NativeArray<int>(gridSizeX * gridSizeY, Allocator.Persistent);
	}

	public float[] GetWaterHeights(Vector3[] searchPositions) {
		if (waterSurface == null)
			return null;

		// Try to get the simulation data if available
		WaterSimSearchData simData = new WaterSimSearchData();
		if (!waterSurface.FillWaterSearchData(ref simData))
			return null;

		// Fill the input positions
		//int numElements = resolution * resolution;
		for (int i = 0; i < searchPositions.Length; ++i) {
			targetPositionBuffer[i] = searchPositions[i];
		}

		// Prepare the first band
		WaterSimulationSearchJob searchJob = new WaterSimulationSearchJob();

		// Assign the simulation data
		searchJob.simSearchData = simData;

		// Fill the input data
		searchJob.targetPositionWSBuffer = targetPositionBuffer;
		searchJob.startPositionWSBuffer = targetPositionBuffer;
		searchJob.maxIterations = 8;
		searchJob.error = 0.001f;
		searchJob.includeDeformation = true;
		searchJob.excludeSimulation = false;

		searchJob.errorBuffer = errorBuffer;
		searchJob.candidateLocationWSBuffer = candidatePositionBuffer;
		searchJob.projectedPositionWSBuffer = projectedPositionWSBuffer;
		searchJob.directionBuffer = directionBuffer;
		searchJob.stepCountBuffer = stepCountBuffer;

		// Schedule the job with one Execute per index in the results array and only 1 item per processing batch
		JobHandle handle = searchJob.Schedule(searchPositions.Length, 1);
		handle.Complete();

		float[] positionHeights = new float[searchPositions.Length];
		
		for (int i = 0; i < positionHeights.Length; i++) {
			positionHeights[i] = projectedPositionWSBuffer[i].y;
		}

		return positionHeights;
	}

	public void Destroy() {
		targetPositionBuffer.Dispose();
		errorBuffer.Dispose();
		candidatePositionBuffer.Dispose();
		projectedPositionWSBuffer.Dispose();
		directionBuffer.Dispose();
		stepCountBuffer.Dispose();
	}
}
