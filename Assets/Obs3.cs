using UnityEngine;

[RequireComponent(typeof(Reflectivity))]
public class Obs3 : MonoBehaviour
{
    private GameObject wall;

    // Object Position in Meters
    Vector3 obstaclePosition;
    float wallHeight;
    float wallWidth;
    float wallDepth;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        this.name = "Mission 1.3 OBS 3";

        // Process dimensions into scale units
        wallHeight = environmentData.wallHeight; // Cube has default size of 1x1x1
        wallWidth = environmentData.wallWidth;
        wallDepth = environmentData.wallDepth;

        // Set Position
        obstaclePosition = new Vector3(environmentData.course3Width / 2,
                                       0,
                                       2 * environmentData.obstacleDepthSpacing);
        this.transform.position = obstaclePosition;

        // Wall Object
        wall = GameObject.CreatePrimitive(PrimitiveType.Cube);
        wall.AddComponent<Reflectivity>().reflectivity = 20f;
        wall.transform.SetParent(this.transform);
        wall.transform.localPosition = new Vector3(0, environmentData.wallHeight / 2, 0);
        wall.transform.localScale = new Vector3(wallWidth, wallHeight, wallDepth); // x is width, y is height, z is depth
        wall.name = "Wall";

        // Assign physics material
        Collider wallCollider = wall.GetComponent<Collider>();
        PhysicsMaterial concrete = Resources.Load<PhysicsMaterial>("concrete");

        if (concrete != null)
        {
            wallCollider.material = concrete;
        }
        else
        {
            Debug.LogWarning("PhysicMaterial 'concrete' not found in Resources folder.");
        }
    }


    // Update is called once per frame
    void Update()
    {

    }
}
