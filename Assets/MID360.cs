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

    public LidarRayGrid(int azimuthSteps, int elevationSteps, float maxElevation, float minElevation, int sectionCount, Allocator allocator)
    {
        this.azimuthSteps = azimuthSteps;
        this.elevationSteps = elevationSteps;
        this.maxElevation = maxElevation;
        this.minElevation = minElevation;
        this.directions = new NativeArray<Vector3>(sectionCount * elevationSteps, allocator);
        InitializeLidarGrid(sectionCount);
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

    public void InitializeLidarGrid(int sectionCount)
    {
        for (int az = 0; az < sectionCount; az++)
        {
            float yaw = (float)az / azimuthSteps * 360f;

            for (int el = 0; el < elevationSteps; el++)
            {
                float elevation = Mathf.Lerp(minElevation, maxElevation, (float)el / (elevationSteps - 1));

                Quaternion rotation = Quaternion.Euler(-elevation, yaw, 0f); // Note the negative sign on elevation cuz Unity uses a left handed coordinate system for whatever fuck reason
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
    public LidarRayGrid rayGrid;
    public int azimuthSteps; // number of azimuth steps around the sensor's 360-degree horizontal field of view
    public int elevationSteps; // number of elevation steps along the sensor's 62.44 vertical field of view (ranging from -7.22 to +55.22, as per LIVOX simulation https://github.com/Livox-SDK/livox_laser_simulation/blob/main/urdf/livox_mid360.xacro)
    public float maxElevation;
    public float minElevation;
    public int sectionCount; // Total number of vertical lines of lidar rays in each section
    public int scanRate; // The scan rate of the MID360, as per data sheet, in Hz
    public int scanWidth; // Angular rate of the MID360 between physics simulation frames in degrees. 
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
        this.transform.rotation = Quaternion.Euler(45, 0, 0); // Set initial rotation
        //TODO: Set rotation if needed

        // Initialize Lidar Sensor Grid
        azimuthSteps = 360;
        elevationSteps = 40;
        maxElevation = 55.22f;
        minElevation = -7.22f;
        scanRate = 10;
        scanWidth = (360 * scanRate) / environmentData.simRate; // TODO: What if this isn't a clean divisor of 360? Round it?
        sectionCount = azimuthSteps * scanRate / environmentData.simRate; // TODO: Think about this more: what happens if sim rate is not an integer / clean divisor?
        maxDistance = 70f;
        minDistance = 0.1f;
        rayGrid = new LidarRayGrid(azimuthSteps, elevationSteps, maxElevation, minElevation, sectionCount, Allocator.Persistent);

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
        //// Visualize all potential lidar rays in green
        //for (int az = 0; az < rayGrid.azimuthSteps; az++)
        //{
        //    for (int el = 0; el < rayGrid.elevationSteps; el++)
        //    {
        //        Vector3 dir = rayGrid.Get(az, el);
        //        Debug.DrawRay(transform.position, dir * 0.5f, Color.green);
        //    }
        //}

        if (activeMarkers.Count > 1000)
        {
            for (int i = 0; i < activeMarkers.Count; i++)
                Destroy(activeMarkers[i]);
            activeMarkers.Clear();
        }


        // Interpolate azimuth, zenith values
        // need to create some vector called the supervector that is updated by the az transform every frame?


        // Create rotation for lidar scan pattern, consisting of rotation about lidar axis and the az transform


        // Visualize casted lidar rays in red
        for (int ray = 0; ray < sectionCount * elevationSteps; ray++)
        {
            rayGrid.directions[ray] = // rotate ray by lidar's current rotation


            // rotate lidar scan pattern by scanWidth degrees around lidar's vertical axis, 
            rayGrid.directions[ray].RotateAround(this.transform.position, this.transform.up, scanWidth);

            // apply az transform

            //rayGrid.directions[ray] ; // rotate by 60 degrees around the 
            Vector3 origin = transform.position + dir * minDistance; // Start at minimum detection distance of 10cm
            if (Physics.Raycast(origin, dir, out RaycastHit hit, maxDistance))
            {
                Debug.DrawLine(origin, hit.point, Color.red); // Draw up to the hit point      
                GameObject s = GameObject.CreatePrimitive(PrimitiveType.Sphere); // Create a small sphere at the hit point
                s.transform.position = hit.point;
                s.transform.localScale = Vector3.one * 0.05f;
                s.transform.parent = transform;
                var col = s.GetComponent<Collider>(); // disable its collider
                if (col) col.enabled = false;
                var rend = s.GetComponent<Renderer>();
                if (rend != null)
                {
                    rend.material.color = Color.yellow;
                }
                activeMarkers.Add(s);
            }
            else
            {
                Debug.DrawLine(origin, origin + dir * maxDistance, Color.red); // If there's no hit, draw the full ray in red
            }
        }
    }


    void OnDestroy()
    {
        rayGrid.Dispose();
    }


    //--------------------------------------------------------------------------------------------------------------------------------------
    // END OF MEMBER FUNCTIONS
}

