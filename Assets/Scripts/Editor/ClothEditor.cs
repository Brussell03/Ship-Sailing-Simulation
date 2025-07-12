using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using System.Linq;
using log4net.Util;
using System;

[CustomEditor(typeof(ClothSimulation))]
public class ClothEditor : Editor {


	private int clickedVertexIndex = -1;
	private int pinWidth = 1;
	private int selectedColumn = -1;
	private int selectedRow = -1;
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
					cloth.ResetCloth();
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
						//cloth.isInitialized = false;
						for (int k = 0; k < cloth.pinnedVertices.Count; k++) {
							cloth.pinnedVertices[k] = new IndexAndVector3(cloth.pinnedVertices[k].index, cloth.x[cloth.pinnedVertices[k].index]);
						}
						cloth.meshFilter.sharedMesh.vertices = cloth.x.ToArray();
						//cloth.Init();
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
								bool isPinned = false;
								int index = 0;
								for (int k = 0; k < cloth.pinnedVertices.Count; k++) {
									if (cloth.pinnedVertices[k].index == cloth.selectedVertices[i]) {
										isPinned = true;
										index = k;
										break;
									}
								}
								if (isPinned) {
									// Unpin Vertices
									cloth.pinnedVertices.RemoveAt(index);
								} else {
									// Pin Vertices
									cloth.pinnedVertices.Add(new IndexAndVector3(cloth.selectedVertices[i], cloth.x[cloth.selectedVertices[i]]));
								}
							}
							EditorUtility.SetDirty(cloth);
						}
					}

					if (cloth.selectedVertices != null) {
						if (GUILayout.Button("Reset Selected")) {
							Vector3[] verts = cloth.meshFilter.sharedMesh.vertices;

							for (int i = 0; i < cloth.selectedVertices.Count; i++) {
								for (int k = 0; k < cloth.pinnedVertices.Count; k++) {
									if (cloth.selectedVertices[i] == cloth.pinnedVertices[k].index) {
										cloth.pinnedVertices[k] = new IndexAndVector3(cloth.selectedVertices[i], cloth.x[cloth.selectedVertices[i]]);
										verts[cloth.selectedVertices[i]] = cloth.x[cloth.selectedVertices[i]];
										
										if (cloth.isDoubleSided) {
											verts[cloth.selectedVertices[i] + cloth.numOneSidedVerts] = cloth.x[cloth.selectedVertices[i]];
										}
										break;
									}
								}
							}

							cloth.meshFilter.sharedMesh.vertices = verts;

							EditorUtility.SetDirty(cloth);
						}
					}
				}
				EditorGUILayout.EndHorizontal();

				EditorGUILayout.BeginHorizontal();
				{
					EditorGUILayout.BeginVertical();
					{
						EditorGUILayout.LabelField("Row:", GUILayout.Width(60));
						selectedRow = Mathf.Clamp(EditorGUILayout.IntField(selectedRow, GUILayout.Width(40)), 0, cloth.maxNumRows);

						EditorGUILayout.LabelField("Column:", GUILayout.Width(60));
						selectedColumn = Mathf.Clamp(EditorGUILayout.IntField(selectedColumn, GUILayout.Width(40)), 0, cloth.maxNumColumns);
					}
					EditorGUILayout.EndVertical();

					EditorGUILayout.BeginVertical();
					{
						if (GUILayout.Button("Select Row", GUILayout.MinWidth(100))) {
							cloth.SelectRow(selectedRow);
							EditorUtility.SetDirty(cloth);
						}

						if (GUILayout.Button("Select Column", GUILayout.MinWidth(100))) {
							cloth.SelectColumn(selectedColumn);
							EditorUtility.SetDirty(cloth);
						}
					}
					EditorGUILayout.EndVertical();
				}
				EditorGUILayout.EndHorizontal();
			}

			EditorGUILayout.Space();
			EditorGUILayout.LabelField("Quick Pinning Options", EditorStyles.miniBoldLabel);
			EditorGUILayout.Space();

			EditorGUILayout.LabelField("Corner Pinning", EditorStyles.label);

			EditorGUILayout.BeginHorizontal(GUI.skin.window);
			{
				EditorGUILayout.LabelField("Dim X:", GUILayout.Width(40));
				cloth.cornerPinDimX = Mathf.Clamp(EditorGUILayout.IntField(cloth.cornerPinDimX, GUILayout.Width(40)), 1, cloth.maxNumColumns / 2 + 1);
				EditorGUILayout.LabelField("Dim Y:", GUILayout.Width(40));
				cloth.cornerPinDimY = Mathf.Clamp(EditorGUILayout.IntField(cloth.cornerPinDimY, GUILayout.Width(40)), 1, cloth.maxNumRows / 2 + 1);
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
						pinWidth = Mathf.Clamp(EditorGUILayout.IntField(pinWidth, GUILayout.Width(40)), 1, cloth.maxNumRows + 1);
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

			if (cloth.pinnedVertices != null && cloth.pinnedVertices.Count > 0) {
				EditorGUILayout.Space();
				EditorGUILayout.BeginHorizontal();
				{
					if (confirmingClear) {
						GUI.backgroundColor = Color.red;
					}
					EditorGUILayout.LabelField($"Pinned Vertices: {cloth.pinnedVertices.Count}", EditorStyles.miniBoldLabel);
					if (GUILayout.Button(confirmingClear ? "Confirm?" : "Clear Pinned")) {
						if (confirmingClear) {
							cloth.pinnedVertices.Clear();
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

		Vector3[] verts = cloth.meshFilter.sharedMesh.vertices;
		Vector3 worldX;

		for (int i = 0; i < cloth.x.Length / 2; i++) {
			worldX = cloth.transform.TransformPoint(verts[i]);
			float handleSize = HandleUtility.GetHandleSize(worldX) * 0.3f;

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

			for (int k = 0; k < cloth.pinnedVertices.Count; k++) {
				if (cloth.pinnedVertices[k].index == i) {
					Handles.color = Color.red;
					break;
				}
			}

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
						Vector3 newPos = verts[cloth.selectedVertices[j]] + cloth.transform.InverseTransformVector(worldDelta);

						verts[cloth.selectedVertices[j]] = newPos;
						if (cloth.isDoubleSided) verts[cloth.selectedVertices[j] + cloth.numOneSidedVerts] = newPos;

						int modifiedListIndex = -1;
						for (int k = 0; k < cloth.pinnedVertices.Count; k++) {
							if (cloth.selectedVertices[j] == cloth.pinnedVertices[k].index) {
								modifiedListIndex = k;
								break;
							}
						}

						if (modifiedListIndex > -1) {
							cloth.pinnedVertices[modifiedListIndex] = new IndexAndVector3(cloth.selectedVertices[j], newPos);
						} else {
							cloth.pinnedVertices.Add(new IndexAndVector3(cloth.selectedVertices[j], newPos));
						}
					}

					cloth.meshFilter.sharedMesh.vertices = verts;
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
			//SceneView.RepaintAll(); // Force Scene View to redraw with new selection
		}

		if (cloth.selectedVertices != null && cloth.selectedVertices.Count > 0) {
			Handles.color = Color.magenta;
			foreach (int index in cloth.selectedVertices) {
				if (index >= 0 && index < cloth.x.Length) {
					Vector3 worldVertex = cloth.transform.TransformPoint(cloth.meshFilter.sharedMesh.vertices[index]);
					Handles.DrawWireCube(worldVertex, Vector3.one * cloth.distBetweenHandles * 1.6f);
				}
			}
		}

		if (cloth.pinnedVertices != null && cloth.pinnedVertices.Count > 0) {
			Handles.color = Color.blue;
			for (int i = 0; i < cloth.pinnedVertices.Count; i++) {
				if (cloth.pinnedVertices[i].vector != cloth.x[cloth.pinnedVertices[i].index]) {
					Vector3 worldVertex = cloth.transform.TransformPoint(cloth.pinnedVertices[i].vector);
					Handles.DrawWireCube(worldVertex, Vector3.one * cloth.distBetweenHandles * 1.2f);
				}
			}
		}
	}
}
