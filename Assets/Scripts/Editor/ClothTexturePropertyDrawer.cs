using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using System.Linq;

[CustomPropertyDrawer(typeof(ClothSimulation.ClothTexture))]
public class ClothTexturePropertyDrawer : PropertyDrawer {
	public override void OnGUI(Rect position, SerializedProperty property, GUIContent label) {

		ClothSimulation clothSimulation = property.serializedObject.targetObject as ClothSimulation;

		ClothDispatcher dispatcher = clothSimulation.dispatcher;

		List<Texture2D> textures = null;
		if (dispatcher != null) textures = dispatcher.textures;

		if (textures != null && textures.Count > 0) {
			// Get current selected enum index
			int currentSelectedIndex = property.intValue;

			// Create display names for the dropdown
			string[] texturesNames = textures.Select(tex => tex == null ? "None" : tex.name).ToArray();

			// Draw the dropdown
			int newSelectedIndex = EditorGUI.Popup(position, label.text, currentSelectedIndex, texturesNames);

			// If the selected index changed, update the SerializedProperty
			if (newSelectedIndex != currentSelectedIndex) {
				property.intValue = newSelectedIndex;
			}
		} else {
			// If textures list is not available, fall back to default enum drawing
			EditorGUI.PropertyField(position, property, label);
			if (dispatcher == null) {
				EditorGUI.HelpBox(position, "ClothDispatcher not found or texture list is empty. Enum may not reflect textures.", MessageType.Warning);
			}
		}
	}
}
