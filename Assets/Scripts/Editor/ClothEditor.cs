using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using System.Linq;
using log4net.Util;
using System;
using static ClothSimulation;

[CustomEditor(typeof(ClothSimulation))]
public class ClothEditor : Editor {


	private int clickedVertexIndex = -1;
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

		if (Application.isPlaying) return;

		EditorGUILayout.BeginVertical(EditorStyles.helpBox);
		{
			EditorGUILayout.LabelField("Cloth Visualization", EditorStyles.boldLabel);
			EditorGUILayout.Space();

			EditorGUILayout.BeginHorizontal();
			{
				if (cloth.meshRenderer != null) {
					if (GUILayout.Button(cloth.meshRenderer.enabled ? "Hide Cloth" : "Show Cloth")) {
						if (cloth.meshRenderer.enabled) {
							cloth.HideCloth();
						} else {
							cloth.ShowCloth();
						}

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
			EditorGUILayout.LabelField("Cloth Shaping and Pinning", EditorStyles.boldLabel);
			EditorGUILayout.Space();

			if (cloth.isInitialized) {
				if (GUILayout.Button(cloth.showShapingGizmos ? "Hide Shaping Gizmos" : "Show Shaping Gizmos")) {
					cloth.showShapingGizmos = !cloth.showShapingGizmos;
					cloth.selectedVertices.Clear();
					EditorUtility.SetDirty(cloth);
				}
			}
			
			if (cloth.showShapingGizmos) {

				EditorGUILayout.Space();

				EditorGUILayout.BeginVertical(EditorStyles.helpBox);
				{
					EditorGUILayout.LabelField("Selected Pin Options", EditorStyles.miniBoldLabel);

					EditorGUILayout.BeginHorizontal();
					{
						if (GUILayout.Button("Clear Selected")) {
							cloth.selectedVertices.Clear();
							EditorUtility.SetDirty(cloth);
						}

						if (GUILayout.Button("Reset Shape")) {
							for (int k = 0; k < cloth.pinnedVertices.Count; k++) {
								cloth.pinnedVertices[k] = new IndexAndVector3(cloth.pinnedVertices[k].index, cloth.x[cloth.pinnedVertices[k].index]);
							}
							cloth.meshFilter.sharedMesh.vertices = cloth.x.ToArray();
						}

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
					EditorGUILayout.EndHorizontal();

					EditorGUILayout.BeginHorizontal();
					{
						if (GUILayout.Button("Pin Selected")) {
							for (int i = 0; i < cloth.selectedVertices.Count; i++) {

								bool isPinned = false;
								for (int k = 0; k < cloth.pinnedVertices.Count; k++) {
									if (cloth.pinnedVertices[k].index == cloth.selectedVertices[i]) {
										isPinned = true;
										break;
									}
								}

								if (!isPinned) {

									bool isConnected = false;
									for (int k = 0; k < cloth.connectedVertices.Count; k++) {
										if (cloth.connectedVertices[k].vertIndex == cloth.selectedVertices[i]) {
											isConnected = true;
											break;
										}
									}

									// Pin Vertices
									if (!isConnected) cloth.pinnedVertices.Add(new IndexAndVector3(cloth.selectedVertices[i], cloth.x[cloth.selectedVertices[i]]));
								}
							}
							EditorUtility.SetDirty(cloth);
						}

						if (GUILayout.Button("Unpin Selected")) {
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
								}
							}
							EditorUtility.SetDirty(cloth);
						}
					}
					EditorGUILayout.EndHorizontal();

					EditorGUILayout.Space();

					EditorGUILayout.BeginHorizontal();
					{
						EditorGUILayout.BeginVertical();
						{
							float originalLabelWidth = EditorGUIUtility.labelWidth;
							EditorGUIUtility.labelWidth = 60;
							selectedRow = Mathf.Clamp(EditorGUILayout.IntField("Row:", selectedRow), 0, cloth.baseNumRows);

							selectedColumn = Mathf.Clamp(EditorGUILayout.IntField("Column:", selectedColumn), 0, cloth.baseNumColumns);

							EditorGUIUtility.labelWidth = originalLabelWidth;
						}
						EditorGUILayout.EndVertical();

						EditorGUILayout.BeginVertical();
						{
							if (GUILayout.Button("Select Row", GUILayout.MinWidth(80))) {
								cloth.SelectRow(selectedRow);
								EditorUtility.SetDirty(cloth);
							}

							if (GUILayout.Button("Select Column", GUILayout.MinWidth(80))) {
								cloth.SelectColumn(selectedColumn);
								EditorUtility.SetDirty(cloth);
							}
						}
						EditorGUILayout.EndVertical();
					}
					EditorGUILayout.EndHorizontal();
				}
				EditorGUILayout.EndVertical();
			}
			
			EditorGUILayout.Space();

			EditorGUILayout.BeginVertical(EditorStyles.helpBox);
			{
				EditorGUILayout.LabelField("Quick Pinning Options", EditorStyles.miniBoldLabel);

				EditorGUILayout.BeginHorizontal();
				{
					EditorGUILayout.BeginVertical();
					{
						if (GUILayout.Button("Pin Corners", GUILayout.MinWidth(100))) {
							cloth.PinCorners();
							EditorUtility.SetDirty(cloth);
						}

						if (GUILayout.Button("Pin Left Edge", GUILayout.MinWidth(100))) {
							cloth.PinLeftEdge();
							EditorUtility.SetDirty(cloth);
						}

						if (GUILayout.Button("Pin Top Edge", GUILayout.MinWidth(100))) {
							cloth.PinTopEdge();
							EditorUtility.SetDirty(cloth);
						}
					}
					EditorGUILayout.EndVertical();

					EditorGUILayout.BeginVertical();
					{
						if (GUILayout.Button("Pin All Edges", GUILayout.MinWidth(100))) {
							cloth.PinAllEdges();
							EditorUtility.SetDirty(cloth);
						}

						if (GUILayout.Button("Pin Right Edge", GUILayout.MinWidth(100))) {
							cloth.PinRightEdge();
							EditorUtility.SetDirty(cloth);
						}

						if (GUILayout.Button("Pin Bottom Edge", GUILayout.MinWidth(100))) {
							cloth.PinBottomEdge();
							EditorUtility.SetDirty(cloth);
						}
					}
					EditorGUILayout.EndVertical();
				}
				EditorGUILayout.EndHorizontal();
			}
			EditorGUILayout.EndVertical();

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
							cloth.LODs[cloth.maxQualityLODIndex].pinIndices.Clear();
							cloth.LODs[cloth.maxQualityLODIndex].pinListIndices.Clear();

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
		int every = (int)Mathf.Pow(2, cloth.maxSubdivisions);

		for (int i = 0; i < cloth.x.Length / 2; i++) {

			Handles.color = Color.yellow;
			for (int j = 0; j < cloth.connectedVertices.Count; j++) {
				if (cloth.connectedVertices[j].vertIndex == i) {
					Handles.color = Color.green;
					break;
				}
			}

			bool isPinned = false;
			for (int k = 0; k < cloth.pinnedVertices.Count; k++) {
				if (cloth.pinnedVertices[k].index == i) {
					Handles.color = Color.blue;
					isPinned = true;
					break;
				}
			}

			if (!isPinned) {
				int rowIndex = 0;
				int colIndex = 0;

				if (cloth.clothShape == ClothShape.Trapezoidal) {
					int passed = 0;
					int colToRowDiff = cloth.maxNumColumns - cloth.maxNumRows;
					for (int j = 0; j <= cloth.maxNumRows; j++) {
						int numOnRow = (colToRowDiff > 0 ? colToRowDiff : 0) + j + 1;
						numOnRow = numOnRow > cloth.maxNumColumns + 1 ? cloth.maxNumColumns + 1 : numOnRow;

						if (i > numOnRow - 1 + passed) {
							passed += numOnRow;
						} else {
							rowIndex = j;
							colIndex = cloth.maxNumColumns - numOnRow + 1 + (i - passed) % numOnRow;
							break;
						}
					}
				} else {
					rowIndex = i / (cloth.maxNumColumns + 1);
					colIndex = i % (cloth.maxNumColumns + 1);
				}

				if (!(colIndex % every == 0 && rowIndex % every == 0)) {
					// Point is not a base grid point
					continue;
				}
			}

			worldX = cloth.transform.TransformPoint(verts[i]);
			float handleSize = HandleUtility.GetHandleSize(worldX) * 0.3f;

			if (handleSize > cloth.distBetweenHandles) {
				handleSize = cloth.distBetweenHandles;
			}

			if (Handles.Button(worldX, Quaternion.identity, handleSize, handleSize * 1.5f, Handles.CubeHandleCap)) {
				clickedVertexIndex = i; // Store the index of the clicked vertex
				guiEvent.Use(); // Consume the event so it doesn't propagate further
			}
		}

		for (int i = 0; i < cloth.selectedVertices.Count; i++) {

			int modifiedListIndex = -1;
			for (int k = 0; k < cloth.pinnedVertices.Count; k++) {
				if (cloth.selectedVertices[i] == cloth.pinnedVertices[k].index) {
					modifiedListIndex = k;
					break;
				}
			}

			if (modifiedListIndex > -1) {
				worldX = cloth.transform.TransformPoint(verts[cloth.selectedVertices[i]]);
				Vector3 handlePosition = Handles.PositionHandle(worldX, Quaternion.identity);

				if (handlePosition != worldX) {
					Vector3 worldDelta = handlePosition - worldX;

					for (int j = 0; j < cloth.selectedVertices.Count; j++) {
						Vector3 newPos = verts[cloth.selectedVertices[j]] + cloth.transform.InverseTransformVector(worldDelta);

						verts[cloth.selectedVertices[j]] = newPos;
						if (cloth.isDoubleSided) verts[cloth.selectedVertices[j] + cloth.numOneSidedVerts] = newPos;

						modifiedListIndex = -1;
						for (int k = 0; k < cloth.pinnedVertices.Count; k++) {
							if (cloth.selectedVertices[j] == cloth.pinnedVertices[k].index) {
								modifiedListIndex = k;
								break;
							}
						}

						if (modifiedListIndex > -1) {
							cloth.pinnedVertices[modifiedListIndex] = new IndexAndVector3(cloth.selectedVertices[j], newPos);
						}
					}

					cloth.meshFilter.sharedMesh.vertices = verts;
					cloth.meshFilter.sharedMesh.RecalculateBounds();

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
