using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;
using Random = UnityEngine.Random;

public class GameManager : MonoBehaviour
{

	public WaterSurface waterSurface;
	public Vector3 windDirection = Vector3.forward;
	public float windSpeed = 5f;

	private float windAngle = 0f;
	private Coroutine _windUpdateCoroutine;

	void Start()
	{
		//Application.targetFrameRate = 144;
		waterSurface.gameObject.SetActive(true);
		waterSurface.simulationStart = new DateTime(2008, 5, 1, 8, 30, 52); // HDRP will compute the water simulation as if the program started at that time

		//StarWindUpdateCoroutine();
	}

	// Update is called once per frame
	void Update()
	{
		//Debug.Log(1f / Time.unscaledDeltaTime);
	}

	private void OnDisable() {
		StopWindUpdateCoroutine();
	}

	private void StarWindUpdateCoroutine() {
		if (_windUpdateCoroutine != null) StopCoroutine(_windUpdateCoroutine); // Stop old if running
		_windUpdateCoroutine = StartCoroutine(RunWindUpdateCoroutine());
	}

	private void StopWindUpdateCoroutine() {
		if (_windUpdateCoroutine != null) {
			StopCoroutine(_windUpdateCoroutine);
			_windUpdateCoroutine = null;
		}
	}


	/// <summary>
	/// Randomizes the wind direction to more than 60 degrees away from the previous direction
	/// </summary>
	private IEnumerator RunWindUpdateCoroutine() {
		while (true) {

			float minAngleOffsetRadians = 60f * Mathf.Deg2Rad;

			float randomSign = (Random.Range(0f, 1f) < 0.5) ? 1.0f : -1.0f;
			float randomDeviation = Random.Range(0f, 1f) * (Mathf.PI - minAngleOffsetRadians); // Random between 0 and (PI - minOffset)
			float angleOffsetRadians = minAngleOffsetRadians + randomDeviation; // Now angleOffsetRadians is between minOffset and PI

			angleOffsetRadians *= randomSign;

			float newAngleRadians = windAngle + angleOffsetRadians;
			windAngle = newAngleRadians;

			float x = windSpeed * Mathf.Cos(newAngleRadians);
			float z = windSpeed * Mathf.Sin(newAngleRadians);
			windDirection = new Vector3(x, 0f, z);

			yield return new WaitForSeconds(120);
		}
	}
}
