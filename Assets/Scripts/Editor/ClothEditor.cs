using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using System.Linq;
using log4net.Util;

[CustomEditor(typeof(ClothSimulation))]
public class ClothEditor : Editor {


	private int clickedVertexIndex = -1;
	private int pinWidth = 1;
	private bool confirmingClear;

	private SerializedProperty numRowsProp;
	private SerializedProperty numColumnsProp;

	private void OnEnable() {
		numRowsProp = serializedObject.FindProperty("numRows");
		numColumnsProp = serializedObject.FindProperty("numColumns");
	}

	public override void OnInspectorGUI() {
		base.OnInspectorGUI();
		serializedObject.Update();

		ClothSimulation cloth = (ClothSimulation)target;
		Color defaultGuiColor = GUI.backgroundColor;

		//EditorGUILayout.PropertyField(numRowsProp);
		//EditorGUILayout.PropertyField(numColumnsProp);

		EditorGUILayout.BeginVertical(EditorStyles.helpBox);
		{
			EditorGUILayout.LabelField("Cloth Visualization", EditorStyles.boldLabel); // Bold text header
			EditorGUILayout.Space(); // Add some space below the header

			EditorGUILayout.BeginHorizontal();
			{
				if (GUILayout.Button(cloth.isInitialized ? "Hide Cloth" : "Show Cloth")) {
					if (cloth.isInitialized) {
						cloth.HideCloth();
					} else {
						cloth.ShowCloth();
					}

				}

				if (GUILayout.Button("Reset Cloth")) {
					cloth.isInitialized = false;
					cloth.pinnedVertices.Clear();
					cloth.modifiedVertices.Clear();
					cloth.selectedVertices.Clear();

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
					if (GUILayout.Button("Reset Shape")) {
						cloth.isInitialized = false;
						cloth.modifiedVertices.Clear();
						cloth.Init();
					}

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

					if (cloth.selectedVertices != null) {
						if (GUILayout.Button("Reset Selected")) {
							for (int i = 0; i < cloth.selectedVertices.Count; i++) {
								int modifiedListIndex = -1;
								for (int k = 0; k < cloth.modifiedVertices.Count; k++) {
									if (cloth.selectedVertices[i] == cloth.modifiedVertices[i].index) {
										modifiedListIndex = k;
										break;
									}
								}

								if (modifiedListIndex > -1) {
									// Reset Vertices
									cloth.modifiedVertices.RemoveAt(modifiedListIndex);
								}
							}
							EditorUtility.SetDirty(cloth);
						}
					}
				}
				EditorGUILayout.EndHorizontal();

			}

			EditorGUILayout.Space();
			EditorGUILayout.LabelField($"Modified Vertices: {cloth.modifiedVertices.Count}", EditorStyles.miniBoldLabel);

			EditorGUILayout.Space();
			EditorGUILayout.LabelField("Quick Pinning Options", EditorStyles.miniBoldLabel);
			EditorGUILayout.Space();

			EditorGUILayout.LabelField("Corner Pinning", EditorStyles.label);

			EditorGUILayout.BeginHorizontal(GUI.skin.window);
			{
				EditorGUILayout.LabelField("Dim X:", GUILayout.Width(40));
				cloth.cornerPinDimX = Mathf.Clamp(EditorGUILayout.IntField(cloth.cornerPinDimX, GUILayout.Width(40)), 1, cloth.numColumns / 2 + 1);
				EditorGUILayout.LabelField("Dim Y:", GUILayout.Width(40));
				cloth.cornerPinDimY = Mathf.Clamp(EditorGUILayout.IntField(cloth.cornerPinDimY, GUILayout.Width(40)), 1, cloth.numRows / 2 + 1);
				if (GUILayout.Button("Pin Corners", GUILayout.MinWidth(100))) {
					cloth.PinCorners();
					EditorUtility.SetDirty(cloth);
				}
			}
			EditorGUILayout.EndHorizontal();

			EditorGUILayout.Space();
			EditorGUILayout.LabelField("Edge Pinning", EditorStyles.label);

			EditorGUILayout.BeginHorizontal(GUI.skin.window);
			{
				EditorGUILayout.BeginVertical();
				{
					EditorGUILayout.BeginHorizontal();
					{
						EditorGUILayout.LabelField("Edge Pin Width:", GUILayout.Width(100));
						pinWidth = Mathf.Clamp(EditorGUILayout.IntField(pinWidth, GUILayout.Width(40)), 1, cloth.numRows + 1);
					}
					EditorGUILayout.EndHorizontal();

					if (GUILayout.Button("Pin Left Edge", GUILayout.MinWidth(100))) {
						cloth.PinLeftEdge(pinWidth);
						EditorUtility.SetDirty(cloth);
					}

					if (GUILayout.Button("Pin Top Edge", GUILayout.MinWidth(100))) {
						cloth.PinTopEdge(pinWidth);
						EditorUtility.SetDirty(cloth);
					}
				}
				EditorGUILayout.EndVertical();

				EditorGUILayout.BeginVertical();
				{
					if (GUILayout.Button("Pin All Edges", GUILayout.MinWidth(100))) {
						cloth.PinAllEdges(pinWidth);
						EditorUtility.SetDirty(cloth);
					}

					if (GUILayout.Button("Pin Right Edge", GUILayout.MinWidth(100))) {
						cloth.PinRightEdge(pinWidth);
						EditorUtility.SetDirty(cloth);
					}

					if (GUILayout.Button("Pin Bottom Edge", GUILayout.MinWidth(100))) {
						cloth.PinBottomEdge(pinWidth);
						EditorUtility.SetDirty(cloth);
					}
				}
				EditorGUILayout.EndVertical();
			}
			EditorGUILayout.EndHorizontal();

			if (cloth._pinnedVertices != null && cloth._pinnedVertices.Count > 0) {
				EditorGUILayout.Space();
				EditorGUILayout.BeginHorizontal();
				{
					if (confirmingClear) {
						GUI.backgroundColor = Color.red;
					}
					EditorGUILayout.LabelField($"Pinned Vertices: {cloth._pinnedVertices.Count}", EditorStyles.miniBoldLabel);
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

		serializedObject.ApplyModifiedProperties();
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

			if (handleSize > cloth.distBetweenHandles) {
				handleSize = cloth.distBetweenHandles;
			}

			Handles.color = Color.yellow;
			for (int j = 0; j < cloth.connectedVertices.Count; j++) {
				if (cloth.connectedVertices[j].vertIndex == i) {
					Handles.color = Color.blue;
					break;
				}
			}
			Handles.color = cloth.pinnedVertices.Contains(i) ? Color.red : Handles.color;

			bool isSelected = cloth.selectedVertices.Contains(i);

			if (Handles.Button(worldX, Quaternion.identity, handleSize, handleSize * 1.5f, Handles.CubeHandleCap)) {
				clickedVertexIndex = i; // Store the index of the clicked vertex
				guiEvent.Use(); // Consume the event so it doesn't propagate further
			}

			if (isSelected) {
				Vector3 handlePosition = Handles.PositionHandle(worldX, Quaternion.identity);

				if (handlePosition != worldX) {
					Vector3 worldDelta = handlePosition - worldX;

					for (int j = 0; j < cloth.selectedVertices.Count; j++) {
						Vector3 newPos = cloth.x[cloth.selectedVertices[j]] + cloth.transform.InverseTransformVector(worldDelta);

						cloth.x[cloth.selectedVertices[j]] = newPos;
						if (cloth.isDoubleSided) cloth.x[cloth.selectedVertices[j] + cloth.numOneSidedVerts] = newPos;

						int modifiedListIndex = -1;
						for (int k = 0; k < cloth.modifiedVertices.Count; k++) {
							if (cloth.selectedVertices[j] == cloth.modifiedVertices[k].index) {
								modifiedListIndex = k;
								break;
							}
						}

						if (modifiedListIndex > -1) {
							cloth.modifiedVertices[modifiedListIndex] = new IndexAndVector3(cloth.selectedVertices[j], newPos);
						} else {
							cloth.modifiedVertices.Add(new IndexAndVector3(cloth.selectedVertices[j], newPos));
						}
					}
					

					cloth.meshFilter.sharedMesh.vertices = cloth.x;
					cloth.meshFilter.sharedMesh.RecalculateBounds();
					cloth.meshFilter.sharedMesh.RecalculateNormals();

					EditorUtility.SetDirty(cloth);
					SceneView.RepaintAll();
				}
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
					Handles.DrawWireCube(worldVertex, Vector3.one * cloth.distBetweenHandles * 1.6f);
				}
			}
		}

		if (cloth.modifiedVertices != null && cloth.modifiedVertices.Count > 0) {
			Handles.color = Color.blue;
			for (int i = 0; i < cloth.modifiedVertices.Count; i++) {
				Vector3 worldVertex = cloth.transform.TransformPoint(cloth.modifiedVertices[i].vector);
				Handles.DrawWireCube(worldVertex, Vector3.one * cloth.distBetweenHandles * 1.2f);
			}
		}
	}
}
