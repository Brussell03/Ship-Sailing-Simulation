using System.Collections;
using System.Collections.Generic;
using System.Text.RegularExpressions;
using UnityEditor;
using UnityEngine;

[CustomPropertyDrawer(typeof(ClothSimulation.TighteningGroup))]
public class TighteningGroupsPropertyDrawer : PropertyDrawer {

	public override void OnGUI(Rect position, SerializedProperty property, GUIContent label) {

		using (new EditorGUI.PropertyScope(position, label, property)) {

			// Get references to the serialized properties within the struct
			SerializedProperty slacknessProp = property.FindPropertyRelative("slackness");
			SerializedProperty isSetProp = property.FindPropertyRelative("isSet");
			SerializedProperty verticesProp = property.FindPropertyRelative("vertices");
			SerializedProperty directionProp = property.FindPropertyRelative("tighteningDirection");

			ClothSimulation cloth = property.serializedObject.targetObject as ClothSimulation;

			// --- Layout the elements ---
			float singleLineHeight = EditorGUIUtility.singleLineHeight;
			float spacing = EditorGUIUtility.standardVerticalSpacing;
			//float buttonWidth = 100f; // Adjust button width as needed
			float fieldWidth = (position.width - spacing) / 2f; // Remaining width for two fields

			// Calculate rects for drawing fields
			Rect currentRect = position;
			currentRect.height = singleLineHeight; // For the first line (connectedCloth)

			// Draw connectedCloth field (full width)
			EditorGUI.LabelField(currentRect, "Num Vertices:", verticesProp.arraySize.ToString());

			// Move to the next line for targetListIndex, vertIndex, and the button
			//currentRect.x += fieldWidth;
			//currentRect.width = fieldWidth;

			currentRect.x += fieldWidth + spacing;
			currentRect.width = fieldWidth;
			Rect buttonRect = currentRect;

			// --- The Button ---
			if (GUI.Button(buttonRect, "Group Selected")) {

				if (cloth.selectedVertices.Count > 0) {
					verticesProp.arraySize = cloth.selectedVertices.Count;

					for (int i = 0; i < cloth.selectedVertices.Count; i++) {
						SerializedProperty elementProp = verticesProp.GetArrayElementAtIndex(i);
						elementProp.intValue = cloth.selectedVertices[i];
					}

					isSetProp.boolValue = true;
					Debug.Log($"Tightening group on {cloth.name} was set.");
				} else {
					verticesProp.arraySize = 0;
					isSetProp.boolValue = false;

					Debug.Log($"Tightening group on {cloth.name} was cleared, select vertices to set group.");
				}

				EditorUtility.SetDirty(cloth);
			}

			Rect slacknessRect = position;
			slacknessRect.y += singleLineHeight + spacing;
			slacknessRect.height = singleLineHeight;
			slacknessRect.width = fieldWidth;
			slacknessProp.floatValue = EditorGUI.Slider(slacknessRect, "Slackness", slacknessProp.floatValue, 0f, 1f);

			Rect clearButtonRect = slacknessRect;
			clearButtonRect.x += fieldWidth + spacing;

			// --- The Button ---
			if (GUI.Button(clearButtonRect, "Clear Group")) {

				verticesProp.arraySize = 0;
				isSetProp.boolValue = false;

				Debug.Log($"Tightening group on {cloth.name} was cleared, select vertices to set group.");

				EditorUtility.SetDirty(cloth);
			}

			Rect directionRect = slacknessRect;
			directionRect.y += singleLineHeight + spacing;
			directionRect.width = position.width;

			EditorGUI.PropertyField(directionRect, directionProp, new GUIContent("Tightening Direction"), true);
		}
	}

	public override float GetPropertyHeight(SerializedProperty property, GUIContent label) {
		return EditorGUIUtility.singleLineHeight * 3 + EditorGUIUtility.standardVerticalSpacing * 4;
	}
}
