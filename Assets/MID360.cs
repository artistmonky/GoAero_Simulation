// Lidar Simulation using Segment Selection


using UnityEngine;
using Unity.Collections;
using System.Collections.Generic;

public struct LidarRayGrid // This struct defines a grid of lidar rays for the MID360, with azimuth and elevation steps.
{
    public int azimuthSteps;   // 360
    public int elevationSteps; // 40
    public float maxElevation; // Maximum elevation angle in degrees
    public float minElevation; // Minimum elevation angle in degrees
    public NativeArray<Vector3> directions;

    public LidarRayGrid(int azimuthSteps, int elevationSteps, float maxElevation, float minElevation, Allocator allocator)
    {
        this.azimuthSteps = azimuthSteps;
        this.elevationSteps = elevationSteps;
        this.maxElevation = maxElevation;
        this.minElevation = minElevation;
        this.directions = new NativeArray<Vector3>(azimuthSteps * elevationSteps, allocator);
        InitializeLidarGrid();
    }

    public void Dispose()
    {
        if (directions.IsCreated)
            directions.Dispose();
    }

    public int Index(int azimuthIndex, int elevationIndex)
    {
        return azimuthIndex * elevationSteps + elevationIndex;
    }

    public Vector3 Get(int azimuthIndex, int elevationIndex)
    {
        return directions[Index(azimuthIndex, elevationIndex)];
    }

    public void Set(int azimuthIndex, int elevationIndex, Vector3 dir)
    {
        directions[Index(azimuthIndex, elevationIndex)] = dir;
    }

    public void InitializeLidarGrid()
    {
        for (int az = 0; az < azimuthSteps; az++)
        {
            float yaw = (float)az / azimuthSteps * 360f;

            for (int el = 0; el < elevationSteps; el++)
            {
                float elevation = Mathf.Lerp(minElevation, maxElevation, (float)el / (elevationSteps - 1));

                // Note the negative sign on elevation. CCW rotations are considered positive.
                // Also note that we need 2 distinct rotations. They are separated because Unity applies rotations in the Z->X->Y Order in a LEFT HANDED COORDINATE SYSTEM.
                // The first rotation is around the Y axis (yaw), and the second is around the X axis (elevation).
                // TODO: CHECK MATH!!!
                Quaternion rotation = Quaternion.AngleAxis(yaw, Vector3.up) * Quarternion.AngleAxis(elevation, Vector3.right); 
                Vector3 direction = rotation * Vector3.forward;
                Set(az, el, direction.normalized);
            }
        }
    }
}



public class MID360 : MonoBehaviour
{
    // MEMBER VARIABLES
    //--------------------------------------------------------------------------------------------------------------------------------------
    // Path in the Assets folder (no extension)
    public string modelPath;

    [Header("LiDAR Settings")]
    public int azimuthSteps; // number of azimuth steps around the sensor's 360-degree horizontal field of view
    public int elevationSteps; // number of elevation steps along the sensor's 62.44 vertical field of view (ranging from -7.22 to +55.22, as per LIVOX simulation https://github.com/Livox-SDK/livox_laser_simulation/blob/main/urdf/livox_mid360.xacro)
    public float azimuthAngle; // the angle by which the lidar scan pattern is rotated horizontally during the az transform
    public float zenithAngle; // the angle by which the lidar scan pattern is rotated vertically during the az transform
    public float maxElevation; // top of lidar vertical FOV, as per datasheet (55.22 degrees)
    public float minElevation; // bottom of lidar vertical FOV, as per datasheet (-7.22 degrees)
    public LidarRayGrid rayGrid;
    public int section; // This determines which section of the MID360's lidar pattern is being casted. It increments from 1 to the physics sim rate, and then loops back to 1.
    public int sectionCount; // Total number of vertical lines of lidar rays in each section
    public int totalSections; // Number of sections in each full 360-degree scan
    public int scanRate = environmentData.simRate; // The scan rate of the MID360, as per data sheet
    public float maxDistance; // Maximum distance for raycasting. TODO: IMPLEMENT REFLECTIVITY / DISTANCE COMPUTATION BASED ON DATASHEET
    public float minDistance; // Minimum distance for raycasting, based on datasheet
    

    [Header("Hit Marker")]
    public GameObject hitMarkerPrefab;
    List<GameObject> activeMarkers;
    //--------------------------------------------------------------------------------------------------------------------------------------
    // END OF MEMBER VARIABLES


    // MEMBER FUNCTIONS
    //--------------------------------------------------------------------------------------------------------------------------------------
    void Start()
    {
        // Set position 
        Vector3 position = new Vector3(0, (float)60.10 / 1000 / 2, 1);
        this.transform.position = position;
        // TODO: Set rotation if needed

        // Initialize Lidar Sensor Grid
        azimuthSteps = 360;
        elevationSteps = 40; // Number of lidar rays in each vertical line, as per datasheet
        maxElevation = 55.22f;
        minElevation = -7.22f;
        rayGrid = new LidarRayGrid(azimuthSteps, elevationSteps, maxElevation, minElevation, Allocator.Persistent);
        scanRate = 10; // Scan rate of the MID360, as per datasheet, expressed in Hz 
        section = 1; // Start at section 1
        sectionCount = azimuthSteps * scanRate / environmentData.simRate; // Total number of vertical lines of lidar rays in each section. TODO: Think about this more: what happens if sim rate is not an integer / clean divisor?
        totalSections = environmentData.simRate / scanRate;
        maxDistance = 70f;
        minDistance = 0.1f;

        // Initialize the list of active markers
        activeMarkers = new List<GameObject>();


        // Load mesh and material from the imported .obj
        modelPath = "Models/mid-360-asm";
        GameObject loadedModel = Resources.Load<GameObject>(modelPath);
        if (loadedModel == null)
        {
            Debug.LogError("Failed to load model from path: " + modelPath + ". Make sure the .obj was imported and placed in a Resources folder.");
            return;
        }

        // Get components from the loaded prefab
        MeshFilter sourceMeshFilter = loadedModel.GetComponentInChildren<MeshFilter>();
        MeshRenderer sourceRenderer = loadedModel.GetComponentInChildren<MeshRenderer>();

        if (sourceMeshFilter == null || sourceRenderer == null)
        {
            Debug.LogError("Loaded model is missing MeshFilter or MeshRenderer.");
            return;
        }

        // Add or replace mesh and renderer on this object
        MeshFilter meshFilter = gameObject.GetComponent<MeshFilter>();
        if (meshFilter == null) meshFilter = gameObject.AddComponent<MeshFilter>();
        meshFilter.sharedMesh = sourceMeshFilter.sharedMesh;

        MeshRenderer meshRenderer = gameObject.GetComponent<MeshRenderer>();
        if (meshRenderer == null) meshRenderer = gameObject.AddComponent<MeshRenderer>();
        meshRenderer.sharedMaterials = sourceRenderer.sharedMaterials;

        Debug.Log("MID360 model loaded and applied to existing GameObject.");
    }



    void Update()
    {
        float t0 = Time.realtimeSinceStartup; // Find real time
        Debug.Log($"Time: {t0:F3}s");

        float prevSecond = Mathf.Floor(t0); // Find previous second and next second
        float nextSecond = Mathf.Ceil(t0);
        int prevSecondInt = Mathf.FloorToInt(prevSecond);
        int nextSecondInt = Mathf.CeilToInt(nextSecond);

        // Row number corresponds to time in seconds
        // Columns: 0-azimuth, 1-zenith
        if (Mathf.Abs(prevSecond - t0) < Time.fixedDeltaTime) // If time is within 1 deltaT to either second, use that second's az value
        {
            azimuthAngle = RotationLoader.data[prevSecondInt][0];
            zenithAngle = RotationLoader.data[prevSecondInt][1];
        }
        else if (Mathf.Abs(nextSecond - t0) < Time.fixedDeltaTime)
        {
            azimuthAngle = RotationLoader.data[nextSecondInt][0];
            zenithAngle = RotationLoader.data[nextSecondInt][1];
        }
        else // Linearly interpolate between the previous and next az values
        {
            float gradient = RotationLoader.data[nextSecondInt][0] - RotationLoader.data[prevSecondInt][0];
            azimuthAngle = RotationLoader.data[prevSecondInt][0] + (t0 - prevSecond) * gradient;

            gradient = RotationLoader.data[nextSecondInt][1] - RotationLoader.data[prevSecondInt][1];
            zenithAngle = RotationLoader.data[prevSecondInt][1] + (t0 - prevSecond) * gradient;
        }


        int start = (section - 1) * sectionCount * elevationSteps;
        int end = Mathf.Min(section * sectionCount * elevationSteps, azimuthSteps * sectionCount * elevationSteps);


        for (int ray = start; ray < end; ray++)
        {
            Vector3 dir = rayGrid.directions[ray];
            dir = transform.TransformVector(dir); // rotate to match lidar's orientation in the world
            // TODO: IMPLEMENT THIS, AFTER YOU CHECK THAT THE MATH ON LINE 61 IS CORRECT!!!
            //azRotation = 
            //dir = Quaternion.Euler(0, azimuthAngle, zenithAngle) * dir;
            //dir =  

            Vector3 origin = transform.position + dir * minDistance; // Start from minimum distance for object detection
            if (Physics.Raycast(origin, dir, out RaycastHit hit, maxDistance))
            {
                Debug.DrawLine(origin, hit.point, Color.red); // Draw up to the hit point      
                //GameObject s = GameObject.CreatePrimitive(PrimitiveType.Sphere); // Create a small sphere at the hit point
                //s.transform.position = hit.point;
                //s.transform.localScale = Vector3.one * 0.05f;
                //s.transform.parent = transform;
                //var col = s.GetComponent<Collider>(); // disable its collider
                //if (col) col.enabled = false;
                //var rend = s.GetComponent<Renderer>();
                //if (rend != null)
                //{
                //    rend.material.color = Color.yellow;
                //}
                //activeMarkers.Add(s);
            }
            //else
            //{
            //    Debug.DrawLine(origin, origin + dir * maxDistance, Color.red); // If there's no hit, draw the full ray in red
            //}
        }


        section = section % totalSections + 1; ; // Update the section for the next simulation frame
        float t1 = Time.realtimeSinceStartup;
        float elapsedMs = (t1 - t0) * 1000f;
        Debug.Log($"Update took {elapsedMs:F3} ms");
    }


    void OnDestroy()
    {
        rayGrid.Dispose();
    }


    //--------------------------------------------------------------------------------------------------------------------------------------
    // END OF MEMBER FUNCTIONS
}
