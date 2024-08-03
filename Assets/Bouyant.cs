using UnityEngine;
using UnityEngine.Rendering.HighDefinition;

[ExecuteInEditMode]
public class Bouyant : MonoBehaviour
{

    public WaterSurface waterSurface = null;
	//https://docs.unity3d.com/Packages/com.unity.render-pipelines.high-definition@16.0/manual/water-scripting-in-the-water-system.html
	// Internal search params
	WaterSearchParameters searchParameters = new WaterSearchParameters();
	WaterSearchResult searchResult = new WaterSearchResult();

	// Start is called before the first frame update
	void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
		if (waterSurface != null) {
			// Build the search parameters
			searchParameters.startPositionWS = searchResult.candidateLocationWS;
			searchParameters.targetPositionWS = gameObject.transform.position;
			searchParameters.error = 0.000000001f;
			searchParameters.maxIterations = 200;
			searchParameters.includeDeformation = true;

			// Do the search
			if (waterSurface.ProjectPointOnWaterSurface(searchParameters, out searchResult)) {
				//Debug.Log(searchResult.projectedPositionWS);
				//Debug.Log(searchResult.error);
				gameObject.transform.position = searchResult.projectedPositionWS;
			} else Debug.LogError("Can't Find Projected Position");
		}
	}
}
