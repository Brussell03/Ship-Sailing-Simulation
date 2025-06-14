using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using System.Linq;

[CustomPropertyDrawer(typeof(ClothSimulation.ClothMaterial))]
public class ClothMaterialPropertyDrawer : PropertyDrawer
{
	public override void OnGUI(Rect position, SerializedProperty property, GUIContent label) {
		//base.OnGUI(position, property, label);

		ClothSimulation clothSimulation = property.serializedObject.targetObject as ClothSimulation;

		ClothDispatcher dispatcher = clothSimulation.dispatcher;

		List<ClothMaterial> materials = null;
		if (dispatcher != null) materials = dispatcher.materials;

		if (materials != null && materials.Count > 0) {
			// Get current selected enum index
			int currentSelectedIndex = property.intValue;

			// Create display names for the dropdown
			string[] materialNames = materials.Select(mat => mat.name).ToArray();

			// Draw the dropdown
			int newSelectedIndex = EditorGUI.Popup(position, label.text, currentSelectedIndex, materialNames);

			// If the selected index changed, update the SerializedProperty
			if (newSelectedIndex != currentSelectedIndex) {
				property.intValue = newSelectedIndex;
			}
		} else {
			// If textures list is not available, fall back to default enum drawing
			EditorGUI.PropertyField(position, property, label);
			if (dispatcher == null) {
				EditorGUI.HelpBox(position, "ClothDispatcher not found or materials list is empty. Enum may not reflect materials.", MessageType.Warning);
			}
		}
	}
}
