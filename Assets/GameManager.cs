using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;

public class GameManager : MonoBehaviour
{

	public WaterSurface waterSurface;
	// Start is called before the first frame update
	void Start()
	{
		Application.targetFrameRate = 144;
		waterSurface.simulationStart = new DateTime(2008, 5, 1, 8, 30, 52); // HDRP will compute the water simulation as if the program started at that time
	}

	// Update is called once per frame
	void Update()
	{
		//Debug.Log(1f / Time.unscaledDeltaTime);
	}
}
