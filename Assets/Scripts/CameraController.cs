using System.Collections.Generic;
using UnityEngine;

public class CameraController : MonoBehaviour {
	const float k_MouseSensitivityMultiplier = 0.01f;

	/// <summary>
	/// Rotation speed when using a controller.
	/// </summary>
	public float m_LookSpeedController = 120f;
	/// <summary>
	/// Rotation speed when using the mouse.
	/// </summary>
	public float m_LookSpeedMouse = 4.0f;
	/// <summary>
	/// Movement speed.
	/// </summary>
	public float m_MoveSpeed = 10.0f;
	/// <summary>
	/// Value added to the speed when incrementing.
	/// </summary>
	public float m_MoveSpeedIncrement = 2.5f;
	/// <summary>
	/// Scale factor of the turbo mode.
	/// </summary>
	public float m_Turbo = 10.0f;
	/// <summary>
	/// Scroll speed.
	/// </summary>
	public float m_ScrollSpeed = 1000.0f;
	/// <summary>
	/// Zooming speed.
	/// </summary>
	public float m_ZoomSpeed = 10.0f;
	/// <summary>
	/// Rotation of the camera in orbit mode.
	/// </summary>
	public Vector3 m_CamRotation;
	/// <summary>
	/// Pivot position in following and focused mode.
	/// </summary>
	public Transform m_CameraPivot;
	/// <summary>
	/// Orbit radius.
	/// </summary>
	public float m_OrbitRadius = 60.0f;
	/// <summary>
	/// Target orbit radius.
	/// </summary>
	public float m_TargetOrbitRadius = 60.0f;
	/// <summary>
	/// Target orbit radius.
	/// </summary>
	public float m_OrbitHeight = 13.0f;
	/// <summary>
	/// Camera movement and positioning mode. (0 = Free, 1 = Tracking, 2 = Focused)
	/// </summary>
	public int m_CameraMode = 0;
	/// <summary>
	/// Transform camera is following.
	/// </summary>
	public Transform m_FocusedObject;

	public BuoyantHull m_FocusedShip { get; private set; }

	private static string kMouseX = "Mouse X";
	private static string kMouseY = "Mouse Y";
	private static string kMouseScroll = "Mouse ScrollWheel";
	private static string kRightStickX = "Controller Right Stick X";
	private static string kRightStickY = "Controller Right Stick Y";
	private static string kVertical = "Vertical";
	private static string kHorizontal = "Horizontal";

	private static string kYAxis = "YAxis";
	private static string kSpeedAxis = "Speed Axis";

	void OnEnable()
	{
		RegisterInputs();

		if (m_FocusedObject != null ) {
			m_CameraPivot = m_FocusedObject.transform.Find("CameraHolder");
			transform.parent = m_CameraPivot;
			m_CameraMode = 2;
			m_CamRotation = m_CameraPivot.localEulerAngles;

			m_CameraPivot.position = new Vector3(m_CameraPivot.parent.position.x, m_OrbitHeight, m_CameraPivot.parent.position.z);
			m_OrbitRadius = Vector3.Distance(transform.position, m_CameraPivot.position);
			m_TargetOrbitRadius = m_OrbitRadius;

			m_FocusedShip = m_FocusedObject.GetComponent<BuoyantHull>();
		}

	}

	void RegisterInputs()
	{

#if UNITY_EDITOR
		List<InputManagerEntry> inputEntries = new List<InputManagerEntry>();

		// Add new bindings
		inputEntries.Add(new InputManagerEntry { name = kRightStickX, kind = InputManagerEntry.Kind.Axis, axis = InputManagerEntry.Axis.Fourth, sensitivity = 1.0f, gravity = 1.0f, deadZone = 0.2f });
		inputEntries.Add(new InputManagerEntry { name = kRightStickY, kind = InputManagerEntry.Kind.Axis, axis = InputManagerEntry.Axis.Fifth, sensitivity = 1.0f, gravity = 1.0f, deadZone = 0.2f, invert = true });

		inputEntries.Add(new InputManagerEntry { name = kYAxis, kind = InputManagerEntry.Kind.KeyOrButton, btnPositive = "page up", altBtnPositive = "joystick button 5", btnNegative = "page down", altBtnNegative = "joystick button 4", gravity = 1000.0f, deadZone = 0.001f, sensitivity = 1000.0f });
		inputEntries.Add(new InputManagerEntry { name = kYAxis, kind = InputManagerEntry.Kind.KeyOrButton, btnPositive = "q", btnNegative = "e", gravity = 1000.0f, deadZone = 0.001f, sensitivity = 1000.0f });

		inputEntries.Add(new InputManagerEntry { name = kSpeedAxis, kind = InputManagerEntry.Kind.KeyOrButton, btnPositive = "home", btnNegative = "end", gravity = 1000.0f, deadZone = 0.001f, sensitivity = 1000.0f });
		inputEntries.Add(new InputManagerEntry { name = kSpeedAxis, kind = InputManagerEntry.Kind.Axis, axis = InputManagerEntry.Axis.Seventh, gravity = 1000.0f, deadZone = 0.001f, sensitivity = 1000.0f });

		InputRegistering.RegisterInputs(inputEntries);
#endif
	}

	float inputRotateAxisX, inputRotateAxisY;
	float inputChangeSpeed;
	float inputVertical, inputHorizontal, inputYAxis;
	float inputScrollDelta;
	bool leftShiftBoost, leftShift, fire1;
	float inputMiddleClickRotateAxisX;
	float inputMiddleClickRotateAxisY;

	void UpdateInputs()
	{
		inputRotateAxisX = 0.0f;
		inputRotateAxisY = 0.0f;
		inputScrollDelta = 0.0f;
		inputMiddleClickRotateAxisX = 0.0f;
		inputMiddleClickRotateAxisY = 0.0f;
		leftShiftBoost = false;
		fire1 = false;
		if (Input.GetMouseButton(1))
		{
			leftShiftBoost = true;
			inputRotateAxisX = Input.GetAxis(kMouseX) * m_LookSpeedMouse;
			inputRotateAxisY = Input.GetAxis(kMouseY) * m_LookSpeedMouse;
		}
		inputRotateAxisX += (Input.GetAxis(kRightStickX) * m_LookSpeedController * k_MouseSensitivityMultiplier);
		inputRotateAxisY += (Input.GetAxis(kRightStickY) * m_LookSpeedController * k_MouseSensitivityMultiplier);

		if (Input.GetMouseButton(2)) {
			inputMiddleClickRotateAxisX = Input.GetAxis(kMouseX);
			inputMiddleClickRotateAxisY = Input.GetAxis(kMouseY);
		}

		leftShift = Input.GetKey(KeyCode.LeftShift);
		fire1 = Input.GetAxis("Fire1") > 0.0f;

		inputChangeSpeed = Input.GetAxis(kSpeedAxis);

		inputScrollDelta = Input.GetAxis(kMouseScroll);

		inputVertical = Input.GetAxis(kVertical);
		inputHorizontal = Input.GetAxis(kHorizontal);
		inputYAxis = Input.GetAxis(kYAxis);
	}

	void Update()
	{

		// If the debug menu is running, we don't want to conflict with its inputs.
		if (UnityEngine.Rendering.DebugManager.instance.displayRuntimeUI) {

			if (m_CameraMode == 2) {
				Quaternion QT = Quaternion.Euler(m_CamRotation.y, m_CamRotation.x, 0.0f);
				m_CameraPivot.rotation = QT;
			} else if (m_CameraMode == 1) {
				m_CameraPivot.rotation = Quaternion.identity;
			}

			return;
		}

		UpdateInputs();

		if (Input.GetKeyDown(KeyCode.F)) {
			if (m_FocusedObject != null) { // Enter free mode
				m_CameraMode = 0;
				m_FocusedObject = null;
				m_FocusedShip = null;
				m_CameraPivot = null;
				transform.parent = null;
			} else {
				// Check for focusable object in front of camera
				RaycastHit hit;
				if (Physics.Raycast(transform.position, transform.forward, out hit, 200)) {

					m_CameraPivot = hit.transform.Find("CameraHolder");
					if (m_CameraPivot != null) {
						transform.parent = m_CameraPivot;
						m_FocusedObject = hit.transform;
						m_CameraMode = 2;

						m_OrbitRadius = Vector3.Distance(transform.position, m_CameraPivot.position);
						m_TargetOrbitRadius = Mathf.Clamp(m_OrbitRadius, 20.0f, 100.0f);

						transform.LookAt(m_CameraPivot);
						m_CamRotation = new Vector3(Mathf.Clamp(transform.eulerAngles.x, -5.0f, 80.0f), transform.eulerAngles.y, 0.0f);

						transform.localPosition = Vector3.back * m_OrbitRadius;
						transform.localRotation = Quaternion.identity;

						m_FocusedShip = m_FocusedObject.GetComponent<BuoyantHull>();
					}
				}
			}
		}

		if (Input.GetKeyDown(KeyCode.R) && m_FocusedObject != null) {
			m_CameraMode++;
			if (m_CameraMode > 2) { // Switching to Following mode from Focused
				m_CameraMode = 1;
				Vector3 pos = transform.position;
				Quaternion rot = transform.rotation;
				m_CameraPivot.rotation = Quaternion.identity;
				m_OrbitHeight = 13.0f;
				m_CameraPivot.position = new Vector3(m_CameraPivot.parent.position.x, m_OrbitHeight, m_CameraPivot.parent.position.z);
				transform.position = pos;
				transform.rotation = rot;
			} else { // Switching to Focused mode from Following
				m_OrbitRadius = Vector3.Distance(transform.position, m_CameraPivot.position);
				m_TargetOrbitRadius = Mathf.Clamp(m_OrbitRadius, 20.0f, 100.0f);

				transform.LookAt(m_CameraPivot);
				m_CamRotation = new Vector3(Mathf.Clamp(transform.eulerAngles.x, -5.0f, 80.0f), transform.eulerAngles.y, 0.0f);

				transform.localPosition = Vector3.back * m_OrbitRadius;
				transform.localRotation = Quaternion.identity;
			}
		}

		if (inputChangeSpeed != 0.0f)
		{
			m_MoveSpeed += inputChangeSpeed * m_MoveSpeedIncrement;
			if (m_MoveSpeed < m_MoveSpeedIncrement) m_MoveSpeed = m_MoveSpeedIncrement;
		}

		if (m_CameraMode == 0) {
			// Camera Mode is in free mode
			Move();

		} else if (m_CameraMode == 1) {
			// Camera is in tracking mode
			m_CameraPivot.position = new Vector3(m_CameraPivot.parent.position.x, m_OrbitHeight, m_CameraPivot.parent.position.z);
			m_CameraPivot.rotation = Quaternion.identity;

			Move();

			if (transform.localPosition.sqrMagnitude > 10000.0f) {
				Vector3 normal = transform.localPosition.normalized;
				transform.localPosition = normal * 100.0f;
			}

			if (transform.localPosition.y < -25.0f) {
				transform.localPosition = new Vector3(transform.localPosition.x, -25.0f, transform.localPosition.z);
			}
		} else {
			// Camera is in focused mode
			Orbit();

			if (m_FocusedShip != null) {
				if (Input.GetKey(KeyCode.A)) {
					m_FocusedShip.RotateRudder(true);
				} else if (Input.GetKey(KeyCode.D)) {
					m_FocusedShip.RotateRudder(false);
				} else if (Input.GetKey(KeyCode.Q)) {
					m_FocusedShip.RotateRigging(true);
				} else if (Input.GetKey(KeyCode.E)) {
					m_FocusedShip.RotateRigging(false);
				} else if (Input.GetKey(KeyCode.Z)) {
					m_FocusedShip.RotateSpanker(true);
				} else if (Input.GetKey(KeyCode.C)) {
					m_FocusedShip.RotateSpanker(false);
				}
			}
		}
	}

	void Move() {

		bool moved = inputRotateAxisX != 0.0f || inputRotateAxisY != 0.0f || inputVertical != 0.0f || inputHorizontal != 0.0f || inputYAxis != 0.0f;
		if (moved) {
			float rotationX = transform.localEulerAngles.x;
			float newRotationY = transform.localEulerAngles.y + inputRotateAxisX;

			// Weird clamping code due to weird Euler angle mapping...
			float newRotationX = (rotationX - inputRotateAxisY);
			if (rotationX <= 90.0f && newRotationX >= 0.0f)
				newRotationX = Mathf.Clamp(newRotationX, 0.0f, 90.0f);
			if (rotationX >= 270.0f)
				newRotationX = Mathf.Clamp(newRotationX, 270.0f, 360.0f);

			transform.localRotation = Quaternion.Euler(newRotationX, newRotationY, transform.localEulerAngles.z);

			float moveSpeed = Time.deltaTime * m_MoveSpeed;
			if (fire1 || leftShiftBoost && leftShift)
				moveSpeed *= m_Turbo;
			transform.position += transform.forward * moveSpeed * inputVertical;
			transform.position += transform.right * moveSpeed * inputHorizontal;
			transform.position += Vector3.up * moveSpeed * inputYAxis;
		}
	}

	void Orbit() {

		m_CameraPivot.position = new Vector3(m_CameraPivot.parent.position.x, m_OrbitHeight, m_CameraPivot.parent.position.z);

		if (inputRotateAxisX != 0.0f || inputRotateAxisY != 0.0f) {
			m_CamRotation.y += inputRotateAxisX;
			m_CamRotation.x -= inputRotateAxisY;

			if (m_CamRotation.y > 360.0f) {
				m_CamRotation.y -= 360.0f;
			} else if (m_CamRotation.y < 0.0f) {
				m_CamRotation.y += 360.0f;
			}

			m_CamRotation.x = Mathf.Clamp(m_CamRotation.x, -5.0f, 80.0f);
		}

		Quaternion QT = Quaternion.Euler(m_CamRotation.x, m_CamRotation.y, 0.0f);
		m_CameraPivot.rotation = QT;

		if (inputMiddleClickRotateAxisY != 0.0f) {
			m_OrbitHeight += inputMiddleClickRotateAxisY;
			m_OrbitHeight = Mathf.Clamp(m_OrbitHeight, 5.0f, 60.0f);

			m_CameraPivot.position = new Vector3(m_CameraPivot.parent.position.x, m_OrbitHeight, m_CameraPivot.parent.position.z);
		}

		if (inputScrollDelta != 0.0f) {

			m_TargetOrbitRadius -= inputScrollDelta * m_ScrollSpeed * Time.deltaTime;
			m_TargetOrbitRadius = Mathf.Clamp(m_TargetOrbitRadius, 20.0f, 100.0f);

			m_OrbitRadius = Mathf.Lerp(m_OrbitRadius, m_TargetOrbitRadius, Time.deltaTime * m_ZoomSpeed);
			transform.localPosition = Vector3.back * m_OrbitRadius;

		} else if (m_TargetOrbitRadius - m_OrbitRadius > 0.001f || m_OrbitRadius - m_TargetOrbitRadius > 0.001f) {

			m_OrbitRadius = Mathf.Lerp(m_OrbitRadius, m_TargetOrbitRadius, Time.deltaTime * m_ZoomSpeed);
			transform.localPosition = Vector3.back * m_OrbitRadius;
		}
			
	}
}