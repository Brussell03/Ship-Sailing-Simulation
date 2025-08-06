using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

public class LiftTest : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
		Vector3 relativeWindVelocity = Vector3.forward * 10;
		Vector3 windNormal = relativeWindVelocity.normalized;

		Vector3 vertexNormal = Vector3.Dot(windNormal, transform.forward) < 0 ? -transform.forward : transform.forward;
		Debug.DrawRay(transform.position, vertexNormal * 5, Color.cyan, 0, depthTest: false);

		float relativeWindSpeedSq = Vector3.Dot(relativeWindVelocity, relativeWindVelocity);
		Vector3 windLiftForce = new Vector3(0.0f, 0.0f, 0.0f);

		Vector3 liftAxis = Vector3.Cross(windNormal, vertexNormal).normalized;
		Debug.DrawRay(transform.position, liftAxis * 5, Color.green, 0, depthTest: false);

		if (liftAxis.magnitude > 1e-6f) // Ensure liftAxis is valid
		{
			Vector3 liftDirection = Vector3.Cross(liftAxis, windNormal);
			Debug.DrawRay(transform.position, liftDirection * 5, Color.red, 0, depthTest: false);

			float angleOfAttack = Mathf.Asin(Vector3.Dot(windNormal, vertexNormal));

			float liftCoeff = 0.0f;
			if (angleOfAttack < 0.2617993878f) {
				liftCoeff = 1.1f * Mathf.Sin(6.0f * angleOfAttack);
			} else if (angleOfAttack < 0.3490658504f) {
				liftCoeff = 1.1f - 5.67705682f * (angleOfAttack - 0.2617993878f);
			} else if (angleOfAttack < 0.872664626f) {
				liftCoeff = 0.3f * Mathf.Sin(7.5f * angleOfAttack + 1.919862177f) + 0.9f;
			} else {
				liftCoeff = -1.64f * (angleOfAttack - 1.570796327f);
			}

			//windLiftForce = dragFactor[id.x] * liftCoeff * relativeWindSpeedSq * liftDirection;

			Debug.Log("AoA: " + angleOfAttack * Mathf.Rad2Deg);
			Debug.Log("C_L: " + liftCoeff);
		}
	}
}
