using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Rope : MonoBehaviour
{
    public Vector3[] pos;
    public float[] mass;
    public Vector3[] vel;

    [Range(0, 1)]
    public float stiffness = 0.5f;

    private float w;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

	private void FixedUpdate() {
        float dt = Time.fixedDeltaTime;

        for (int i = 0; i < pos.Length; i++) {

        }
	}

	public void Initialize(Vector3[] pos0, Vector3[] vel0, float unitMass, float stiffness) {
		for (int i = 0; i < pos.Length; i++) {
            pos = pos0;
            vel = vel0;
            w = 1 / unitMass;
		}
	}
}
