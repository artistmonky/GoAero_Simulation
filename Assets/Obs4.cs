using UnityEngine;

public class Obs4 : MonoBehaviour
{
    private GameObject pillar;

    // Object Position in Meters
    Vector3 obstaclePosition;
    float pillarHeight;
    float pillarRadius;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        this.name = "Mission 1.3 OBS 4";

        // Process dimensions into scale units
        pillarHeight = environmentData.pylonHeight / 2; // Actual pillar height = pillarHeight * 2
        pillarRadius = environmentData.pylonDiameter;

        // Set Position
        obstaclePosition = new Vector3(environmentData.course3Width / 2,
                                       0,
                                       environmentData.obstacleDepthSpacing);
        this.transform.position = obstaclePosition;

        // Vertical Pillar
        pillar = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        pillar.transform.SetParent(this.transform);
        pillar.transform.localPosition = new Vector3(0, pillarHeight, 0);
        pillar.transform.localScale = new Vector3(pillarRadius, pillarHeight, pillarRadius);
        pillar.name = "Pillar";

        /*// Horizontal Arm
        arm = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        arm.transform.SetParent(this.transform);
        arm.transform.localPosition = new Vector3(armLength, pillarHeight * 2, 0);
        arm.transform.localScale = new Vector3(armRadius, armLength, armRadius);
        arm.transform.eulerAngles = new Vector3(0, 0, 90);
        arm.name = "Arm";*/

        // Assign physics material
        Collider pillarCollider = pillar.GetComponent<Collider>();
        PhysicsMaterial concrete = Resources.Load<PhysicsMaterial>("concrete");

        if (concrete != null)
        {
            pillarCollider.material = concrete;
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
