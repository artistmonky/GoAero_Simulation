using UnityEngine;

[RequireComponent(typeof(Reflectivity))]
public class CourseFloor : MonoBehaviour
{
    // Object Dimensions and Position in Meters
/*    static float floorWidth = environmentData.course3Width; // Right is in +x direction
    static float floorDepth = environmentData.course3Depth; // Depth increases in +z direction*/
    Vector3 floorPosition;
    float floorWidth; // Right is in +x direction
    float floorDepth; // Forward is in +z direction

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        this.name = "Course Floor";

        floorWidth = environmentData.course3Width / 10; // Scale down from meters to default plane size (10x10)
        floorDepth = environmentData.course3Depth / 10; 
        floorPosition = new Vector3(environmentData.course3Width / 2, 0, environmentData.course3Depth / 2);
        this.transform.position = floorPosition;

        // Define plane
        GameObject courseFloor = GameObject.CreatePrimitive(PrimitiveType.Plane);
        courseFloor.AddComponent<Reflectivity>().reflectivity = 20f;
        courseFloor.name = "Course Floor";
        courseFloor.transform.SetParent(this.transform);
        courseFloor.transform.localPosition = new Vector3(0, 0, 0);
        courseFloor.transform.localScale = new Vector3(floorWidth, 1, floorDepth);
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
