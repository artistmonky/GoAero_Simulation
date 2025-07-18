using UnityEngine;
using Unity.Collections;

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

                Quaternion rotation = Quaternion.Euler(-elevation, yaw, 0f); // Unity uses a left handed coordinate system for whatever fuck reason
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
    public int azimuthSteps; // number of azimuth steps around the sensor's 360-degree horizontal field of view
    public int elevationSteps; // number of elevation steps along the sensor's 62.44 vertical field of view (ranging from -7.22 to +55.22, as per LIVOX simulation https://github.com/Livox-SDK/livox_laser_simulation/blob/main/urdf/livox_mid360.xacro)
    public float maxElevation;
    public float minElevation;
    public LidarRayGrid rayGrid;
    public int section; // This determines which section of the MID360's lidar pattern is being casted. It increments from 1 to the physics sim rate, and then loops back to 1.
    public int sectionCount; // Total number of vertical lines of lidar rays in each section
    public int scanRate = environmentData.simRate; // The scan rate of the MID360, as per data sheet
    //--------------------------------------------------------------------------------------------------------------------------------------
    // END OF MEMBER VARIABLES


    // MEMBER FUNCTIONS
    //--------------------------------------------------------------------------------------------------------------------------------------
    void Start()
    {
        // Set position 
        Vector3 position = new Vector3(0, (float)60.10 / 1000 / 2, 1);
        this.transform.position = position;
        //TODO: Set rotation if needed

        // Initialize Lidar Sensor Grid
        azimuthSteps = 360;
        elevationSteps = 40;
        maxElevation = 55.22f;
        minElevation = -7.22f;
        rayGrid = new LidarRayGrid(azimuthSteps, elevationSteps, maxElevation, minElevation, Allocator.Persistent);
        scanRate = 10;
        section = 1; // Start at section 1
        sectionCount = azimuthSteps * scanRate / environmentData.simRate; // Total number of vertical lines of lidar rays in each section. TODO: Think about this more: what happens if sim rate is not an integer / clean divisor?

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
        // Visualize all potential lidar rays in green
        for (int az = 0; az < rayGrid.azimuthSteps; az++)
        {
            for (int el = 0; el < rayGrid.elevationSteps; el++)
            {
                Vector3 dir = rayGrid.Get(az, el);
                Debug.DrawRay(transform.position, dir * 0.5f, Color.green);
            }
        }

        // Visualize casted lidar rays in red
        for (int az = (section - 1) * sectionCount; az < (section - 1) * sectionCount; az++)
        {
            for (int el = 0; el < rayGrid.elevationSteps; el++)
            {
                Vector3 dir = rayGrid.Get(az, el);
                //RaycastHit hit;
                //if (Physics.Raycast(transform.position, dir, out hit, Mathf.Infinity))
                //{
                //    Debug.DrawLine(transform.position, hit.point, Color.red);
                //}
                Debug.DrawLine(transform.position, dir * 2f, Color.red);
            }
        }

        updateSection(); // Update the section for the next simulation frame

    }

    void updateSection()
    {
        section++;
        if (section > environmentData.simRate)
        {
            section = 1;
        }
    }

    void OnDestroy()
    {
        rayGrid.Dispose();
    }


    //--------------------------------------------------------------------------------------------------------------------------------------
    // END OF MEMBER FUNCTIONS
}

