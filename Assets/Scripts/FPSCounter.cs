using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

[RequireComponent(typeof(TextMeshProUGUI))]
public class FPS : MonoBehaviour {

	public int maxFrames = 30;  //maximum frames to average over

	private static int lastFPSCalculated = 0;
	private List<float> fpsList = new List<float>();
	private TextMeshProUGUI text;

	// Use this for initialization
	void Start() {
		lastFPSCalculated = 0;
		fpsList.Clear();
		text = GetComponent<TextMeshProUGUI>();
	}

	// Update is called once per frame
	void Update() {
		addFrame();
		lastFPSCalculated = calculateFPS();
		text.text = "FPS: " + lastFPSCalculated.ToString();
	}

	private void addFrame() {
		fpsList.Add(1f / Time.unscaledDeltaTime);
		if (fpsList.Count > maxFrames) {
			fpsList.RemoveAt(0);
		}
	}

	private int calculateFPS() {
		int newFPS = 0;

		float totalFPS = 0;
		foreach (int fps in fpsList) {
			totalFPS += fps;
		}
		newFPS = (int)(totalFPS / fpsList.Count);

		return newFPS;
	}

	public static int GetCurrentFPS() {
		return lastFPSCalculated;
	}
}