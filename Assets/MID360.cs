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
                // The first rotation is around the local frame's Y axis (yaw), and the second is around the local frame's X axis (elevation).
                // Note that the x-axis of the local frame is rotated by the first rotation
                Quaternion rotation = Quaternion.Euler(-elevation, yaw, 0f); 
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
    public float maxElevation; // top of lidar vertical FOV, as per datasheet (55.22 degrees). The magnitude of the zenith angle in the az transform is then subtracted from this value.
    public float minElevation; // bottom of lidar vertical FOV, as per datasheet (-7.22 degrees). The magnitude of the zenith angle in the az transform is then subtracted from this value.
    public float shrinkage; // The shrinkage angle for the lidar rays
    public LidarRayGrid rayGrid;
    public int section; // This determines which section of the MID360's lidar pattern is being casted. It increments from 1 to the physics sim rate, and then loops back to 1.
    public int sectionCount; // Total number of vertical lines of lidar rays in each section
    public int totalSections; // Number of sections in each full 360-degree scan
    public int scanRate = environmentData.simRate; // The scan rate of the MID360, as per data sheet, in Hz
    public float maxDistance; // Maximum distance for raycasting. This value is arbitrary.
    public float minDistance; // Minimum distance for raycasting, based on datasheet
    public float hitRegistrationExponent; // hitRegistration determines if an object that is hit by a raycast should be registered according to its reflectivity and distance.
    public float hitRegistrationConstant; // hitRegistration is calculated as: max distance d for a reflectivity r = hitRegistrationConstant * (r ^ hitRegistrationExponent). If the distance to the object is greater than d, it is not registered.
    public float angleSigma; // Standard deviation of the lidar ray angles, as per datasheet
    public float distanceSigma; // Standard deviation of the lidar ray distances, as per datasheet
    public NativeArray<float> noiseVector;

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
        // new Vector3(environmentData.course3Width / 2,
        //             (float)60.10 / 1000 / 2,
        //             2 * environmentData.obstacleDepthSpacing - 3);
        this.transform.position = position;
        // TODO: Set rotation if needed

        // Initialize Lidar Sensor Grid
        azimuthSteps = 360;
        elevationSteps = 40; // Number of lidar rays in each vertical line, as per datasheet
        shrinkage = 1.72f;
        maxElevation = (55.22f - shrinkage);
        minElevation = (-7.22f + shrinkage);
        rayGrid = new LidarRayGrid(azimuthSteps, elevationSteps, maxElevation, minElevation, Allocator.Persistent);
        scanRate = 10;
        section = 1; // Start at section 1
        sectionCount = azimuthSteps * scanRate / environmentData.simRate; // TODO: Think about this more: what happens if sim rate is not an integer / clean divisor?
        totalSections = environmentData.simRate / scanRate;
        maxDistance = 85f;
        minDistance = 0.1f;
        hitRegistrationExponent = 0.369f;
        hitRegistrationConstant = 15.23f;
        angleSigma = 0.15f;
        distanceSigma = 0.03f;

        // Initialize the noise vector for generating random points within the unit circle
        noiseVector = new NativeArray<float>(Mathf.CeilToInt(sectionCount * elevationSteps * 3 / 2), Allocator.Persistent);

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

        for (int i = 0; i < noiseVector.Length / 2;)
        {
            // Generate a random point within the unit circle
            float x = Random.Range(-1f, 1f);
            float y = Random.Range(-1f, 1f);
            float s = x * x + y * y;
            while (s >= 1 || s <= 0) // Ensure the point is within the unit circle
            {
                x = Random.Range(-1f, 1f);
                y = Random.Range(-1f, 1f);
                s = x * x + y * y;
            }
            // Store the generated noise in the noise vector
            noiseVector[i] = x * Mathf.Sqrt(-2f * Mathf.Log(s) / s); // Marsaglia polar method for generating observations of N(0,1)
            noiseVector[i+1] = y * Mathf.Sqrt(-2f * Mathf.Log(s) / s);
            i += 2;
        }

        //if (activeMarkers.Count > 90000)
        //{
        //    for (int i = 0; i < activeMarkers.Count; i++)
        //        Destroy(activeMarkers[i]);
        //    activeMarkers.Clear();
        //}

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

        int iter = 0;
        for (int ray = start; ray < end; ray++)
        {
            float azimuthNoise = noiseVector[iter]; // Get noise measurements
            float zenithNoise = noiseVector[iter + 1];
            float distanceNoise = noiseVector[iter + 2];
            iter++;
            
            Vector3 dir = rayGrid.directions[ray];
            dir = transform.TransformVector(dir); // rotate to match lidar's orientation in the world
            Quaternion azRotation = Quaternion.Euler(-zenithAngle, azimuthAngle, 0f);  // Calculate az transform
            Quaternion noiseRotation = Quaternion.Euler(zenithNoise * angleSigma, azimuthNoise * angleSigma, 0f); // Apply noise to the direction vector
            dir = azRotation * noiseRotation * dir; // Apply azRotation and noiseRotation to the direction vector

            Vector3 origin = transform.position + dir * minDistance; // Start from minimum distance for object detection

            if (Physics.Raycast(origin, dir, out RaycastHit hit, maxDistance) &&
                hit.distance < (hitRegistrationConstant * Mathf.Pow(hit.collider.GetComponent<Reflectivity>().reflectivity, hitRegistrationExponent)))
            {
                //Debug.DrawLine(origin, hit.point, Color.green); // Draw the ray in green if it hits an object that should be registered
                hit.point += dir * (distanceNoise * distanceSigma); // Apply distance noise to the hit point

                //// Create and add a marker at the hit point
                //GameObject s = GameObject.CreatePrimitive(PrimitiveType.Sphere); // Create a small sphere at the hit point
                //s.transform.position = hit.point;
                //s.transform.localScale = Vector3.one * 0.01f;
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
