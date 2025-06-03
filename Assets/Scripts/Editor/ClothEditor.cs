using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(ClothSimulation))]
public class ClothEditor : Editor {


	private int clickedVertexIndex = -1;
	private bool confirmingClear;

	public override void OnInspectorGUI() {
		base.OnInspectorGUI();

		ClothSimulation cloth = (ClothSimulation)target;
		Color defaultGuiColor = GUI.backgroundColor;
		//EditorGUILayout.IntField("Health", health);

		EditorGUILayout.BeginVertical(EditorStyles.helpBox);
		{
			EditorGUILayout.LabelField("Cloth Visualization", EditorStyles.boldLabel); // Bold text header
			EditorGUILayout.Space(); // Add some space below the header

			EditorGUILayout.BeginHorizontal();
			{
				if (GUILayout.Button(cloth.isShowingCloth ? "Hide Cloth" : "Show Cloth")) {
					if (cloth.isShowingCloth) {
						cloth.HideCloth();
					} else {
						cloth.ShowCloth();
					}

				}

				if (GUILayout.Button("Reset Cloth")) {
					cloth.isInitialized = false;
					cloth._pinnedVertices.Clear();
					cloth.Init();
				}
			}
			EditorGUILayout.EndHorizontal();
		}
		EditorGUILayout.EndVertical();

		EditorGUILayout.BeginVertical(EditorStyles.helpBox);
		{
			EditorGUILayout.LabelField("Cloth Shaping", EditorStyles.boldLabel);
			EditorGUILayout.Space();

			if (GUILayout.Button(cloth.isShowingVerts ? "Hide Shaping Gizmos" : "Show Shaping Gizmos")) {
				cloth.isShowingVerts = !cloth.isShowingVerts;
				cloth.showShapingGizmos = !cloth.showShapingGizmos;
				cloth.selectedVertices.Clear();
				EditorUtility.SetDirty(cloth);
			}

			if (cloth.isShowingVerts) {
				EditorGUILayout.BeginHorizontal();
				{
					if (cloth.selectedVertices != null) {
						if (GUILayout.Button("Clear Selected")) {
							cloth.selectedVertices.Clear();
							EditorUtility.SetDirty(cloth);
						}
					}

					if (cloth.selectedVertices != null) {
						if (GUILayout.Button("Pin/Unpin Selected")) {
							for (int i = 0; i < cloth.selectedVertices.Count; i++) {
								if (cloth._pinnedVertices.Contains(cloth.selectedVertices[i])) {
									// Unpin Vertices
									cloth._pinnedVertices.Remove(cloth.selectedVertices[i]);
								} else {
									// Pin Vertices
									cloth._pinnedVertices.Add(cloth.selectedVertices[i]);
								}
							}
							EditorUtility.SetDirty(cloth);
						}
					}
				}
				EditorGUILayout.EndHorizontal();

			}


			EditorGUILayout.Space();
			EditorGUILayout.LabelField("Quick Pinning Options", EditorStyles.miniBoldLabel);
			EditorGUILayout.Space();

			EditorGUILayout.BeginHorizontal();
			{
				EditorGUILayout.LabelField("Dim X:", GUILayout.Width(40));
				cloth.cornerPinDimX = Mathf.Clamp(EditorGUILayout.IntField(cloth.cornerPinDimX, GUILayout.Width(40)), 1, cloth.subdivisions / 2 + 1);
				EditorGUILayout.LabelField("Dim Y:", GUILayout.Width(40));
				cloth.cornerPinDimY = Mathf.Clamp(EditorGUILayout.IntField(cloth.cornerPinDimY, GUILayout.Width(40)), 1, cloth.subdivisions / 2 + 1);
				if (GUILayout.Button("Pin Corners", GUILayout.MinWidth(100))) {
					cloth.PinCorners();
					EditorUtility.SetDirty(cloth);
				}
			}
			EditorGUILayout.EndHorizontal();


			if (cloth._pinnedVertices != null && cloth._pinnedVertices.Count > 0) {
				EditorGUILayout.Space();
				EditorGUILayout.BeginHorizontal();
				{
					if (confirmingClear) {
						GUI.backgroundColor = Color.red;
					}
					EditorGUILayout.LabelField($"Pinned Vertices: {cloth._pinnedVertices.Count}");
					if (GUILayout.Button(confirmingClear ? "Confirm?" : "Clear Pinned")) {
						if (confirmingClear) {
							cloth._pinnedVertices.Clear();
							confirmingClear = false;
						} else {
							confirmingClear = true;
						}
						EditorUtility.SetDirty(cloth);
					}
					GUI.backgroundColor = defaultGuiColor;
				}
				EditorGUILayout.EndHorizontal();
			}
		}
		EditorGUILayout.EndVertical();
	}

	public void OnSceneGUI() {
		ClothSimulation cloth = (ClothSimulation)target;

		// Only draw clickable Gizmos if the toggle is active
		if (!cloth.showShapingGizmos || cloth.x == null || cloth.x.Length == 0) {
			return;
		}

		clickedVertexIndex = -1;

		Event guiEvent = Event.current;

		for (int i = 0; i < cloth.x.Length / 2; i++) {
			Vector3 worldX = cloth.transform.TransformPoint(cloth.x[i]);
			float handleSize = HandleUtility.GetHandleSize(worldX) * 0.4f;
			if (handleSize > cloth.distBetweenPoints / 1.5f) {
				handleSize = cloth.distBetweenPoints / 1.5f;
			}

			Handles.color = cloth.selectedVertices.Contains(i) ? Color.red : Color.yellow;

			if (Handles.Button(worldX, Quaternion.identity, handleSize, handleSize * 1.5f, Handles.CubeHandleCap)) {
				clickedVertexIndex = i; // Store the index of the clicked vertex
				guiEvent.Use(); // Consume the event so it doesn't propagate further
			}
		}

		if (clickedVertexIndex != -1) {
			// If Ctrl/Command is held, toggle selection
			if (guiEvent.control || guiEvent.command) {
				if (cloth.selectedVertices.Contains(clickedVertexIndex)) {
					cloth.selectedVertices.Remove(clickedVertexIndex);
				} else {
					cloth.selectedVertices.Add(clickedVertexIndex);
				}
			}
			// Otherwise, single select (clear existing and add new)
			else {
				if (cloth.selectedVertices.Count != 0 && cloth.selectedVertices[0] == clickedVertexIndex) {
					cloth.selectedVertices.Clear();
				} else {
					cloth.selectedVertices.Clear();
					cloth.selectedVertices.Add(clickedVertexIndex);
				}
			}
			EditorUtility.SetDirty(cloth); // Mark dirty to save selection state
										   //Repaint(); // Repaint Inspector to update if needed
			SceneView.RepaintAll(); // Force Scene View to redraw with new selection
		}

		if (cloth.selectedVertices != null && cloth.selectedVertices.Count > 0) {
			Handles.color = Color.magenta;
			foreach (int index in cloth.selectedVertices) {
				if (index >= 0 && index < cloth.x.Length) {
					Vector3 worldVertex = cloth.transform.TransformPoint(cloth.x[index]);
					Handles.DrawWireCube(worldVertex, Vector3.one * cloth.distBetweenPoints * 1.5f);
				}
			}
		}
	}
}
