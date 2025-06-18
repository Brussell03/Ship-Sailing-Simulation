using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEngine;

[CustomPropertyDrawer(typeof(ClothSimulation.ConnectedVertex))]
public class ConnectedVertexPropertyDrawer : PropertyDrawer
{
	public override void OnGUI(Rect position, SerializedProperty property, GUIContent label) {

		using (new EditorGUI.PropertyScope(position, label, property)) {
			// Get references to the serialized properties within the struct
			SerializedProperty connectedClothProp = property.FindPropertyRelative("connectedCloth");
			SerializedProperty targetListIndexProp = property.FindPropertyRelative("targetListIndex");
			SerializedProperty vertIndexProp = property.FindPropertyRelative("vertIndex");
			SerializedProperty isSetProp = property.FindPropertyRelative("isSet");

			// --- Layout the elements ---
			float singleLineHeight = EditorGUIUtility.singleLineHeight;
			float spacing = EditorGUIUtility.standardVerticalSpacing;
			//float buttonWidth = 100f; // Adjust button width as needed
			float fieldWidth = (position.width - spacing) / 2f; // Remaining width for two fields

			// Calculate rects for drawing fields
			Rect currentRect = position;
			currentRect.height = singleLineHeight; // For the first line (connectedCloth)

			// Draw connectedCloth field (full width)
			EditorGUI.PropertyField(currentRect, connectedClothProp);

			// Move to the next line for targetListIndex, vertIndex, and the button
			currentRect.y += singleLineHeight + spacing;

			// Draw targetListIndex
			Rect targetListIndexRect = currentRect;
			targetListIndexRect.width = fieldWidth;
			EditorGUI.PropertyField(targetListIndexRect, targetListIndexProp);

			// Draw vertIndex
			Color defaultColor = GUI.color;

			Rect vertIndexRect = currentRect;
			vertIndexRect.x = targetListIndexRect.xMax + spacing;
			vertIndexRect.width = fieldWidth;
			//EditorGUI.PropertyField(vertIndexRect, vertIndexProp);
			if (!isSetProp.boolValue) GUI.color = Color.yellow;
			EditorGUI.LabelField(vertIndexRect, "Vert Index", vertIndexProp.intValue.ToString());
			GUI.color = defaultColor;

			// Draw the Button
			currentRect.y += singleLineHeight + spacing;
			Rect buttonRect = currentRect;
			//buttonRect.x = vertIndexRect.xMax + spacing;
			//buttonRect.width = buttonWidth;

			// --- The Button ---
			if (GUI.Button(buttonRect, "Set Connected Vertex To First Selected")) {

				ClothSimulation clothSimulation = property.serializedObject.targetObject as ClothSimulation;

				if (clothSimulation.selectedVertices.Count > 0) {
					vertIndexProp.intValue = clothSimulation.selectedVertices[0];
					isSetProp.boolValue = true;
					Debug.Log($"Vertex {vertIndexProp.intValue} on {clothSimulation.name} was set as connected, please match target cloth list index to complete connection.");
					EditorUtility.SetDirty(clothSimulation);
				} else {
					vertIndexProp.intValue = -1;
					isSetProp.boolValue = false;
					Debug.Log($"Select a vertex to set connected index on {clothSimulation.name}.");
				}
			}
		}
	}

	public override float GetPropertyHeight(SerializedProperty property, GUIContent label) {
		return (EditorGUIUtility.singleLineHeight * 3) + EditorGUIUtility.standardVerticalSpacing * 3;
	}
}
