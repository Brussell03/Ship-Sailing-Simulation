using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GameManager : MonoBehaviour
{
	// Start is called before the first frame update
	void Start()
	{
		Application.targetFrameRate = 144;

	}

	// Update is called once per frame
	void Update()
	{
		//Debug.Log(1f / Time.unscaledDeltaTime);
	}
}
