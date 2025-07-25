using UnityEngine;
using Unity.Collections;
using Unity.Burst;
using Unity.Jobs;
using Unity.Mathematics;
using System.Collections.Generic;

[BurstCompile]
public struct MarsagliaNoiseJob : IJobParallelFor
{
    [WriteOnly] public NativeArray<float2> noisePairs;  // One pair per i
    [ReadOnly] public uint baseSeed;

    public void Execute(int i)
    {
        var rng = new Unity.Mathematics.Random(baseSeed + (uint)i);
        float2 u; float s;

        // generate one Gaussian pair
        do
        {
            u = rng.NextFloat2(new float2(-1f), new float2(1f));
            s = math.lengthsq(u);
        } while (s >= 1f || s == 0f);

        float f = math.sqrt(-2f * math.log(s) / s);

        noisePairs[i] = u * f;
    }
}

[BurstCompile]
public struct VectorRotationJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<Vector3> inputDirections; // Input directions (unrotated)
    [ReadOnly] public float2 azRotation; // az rotation. x is azimuth, y is zenith
    [ReadOnly] public NativeArray<float2> noisePairs; // noise rotation
    [ReadOnly] public float angleSigma;
    [ReadOnly] public quaternion lidarRotation; // Rotation of the MID360 sensor, used to rotate the input directions to match the sensor's orientation in the world
    [WriteOnly] public NativeArray<Vector3> outputDirections; // Output directions (rotated by azTransform and their respecitive angular noise values)


    public void Execute(int index)
    {
        Vector3 dir = inputDirections[index];
        dir = math.rotate(lidarRotation, dir); // rotate to match lidar's orientation in the world
        Quaternion azQuaternion= Quaternion.Euler(-azRotation.y, azRotation.x, 0f);  // Calculate az transform
        Quaternion noiseRotation = Quaternion.Euler(-noisePairs[index].y * angleSigma, noisePairs[index].x * angleSigma, 0f); // Apply noise to the direction vector
        dir = azQuaternion * noiseRotation * dir; // Apply az and noise rotations to the direction vector
        outputDirections[index] = dir.normalized; // Store the normalized direction
    }
}

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
                // Note that the x-axis of the local frame is rotated by the first rotation.
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
    public float2 azRotation; // azimuth and zenith rotation angles for the az transform. azRotation.x = azimuth angle, azRotation.y = zenith angle
    public float maxElevation; // top of lidar vertical FOV, as per datasheet (55.22 degrees). The magnitude of the zenith angle in the az transform is then subtracted from this value.
    public float minElevation; // bottom of lidar vertical FOV, as per datasheet (-7.22 degrees). The magnitude of the zenith angle in the az transform is then subtracted from this value.
    public float shrinkage; // The shrinkage angle for the lidar rays
    public LidarRayGrid rayGrid;
    public NativeArray<Vector3> inputDirections;
    public NativeArray<Vector3> outputDirections;
    public int section; // This determines which section of the MID360's lidar pattern is being casted. It increments from 1 to the physics sim rate, and then loops back to 1.
    public int sectionCount; // Total number of vertical lines of lidar rays in each section
    public int totalSections; // Number of sections in each full 360-degree scan
    public int scanRate; // The scan rate of the MID360, as per data sheet, in Hz
    public float maxDistance; // Maximum distance for raycasting. This value is arbitrary.
    public float minDistance; // Minimum distance for raycasting, based on datasheet
    public float hitRegistrationExponent; // hitRegistration determines if an object that is hit by a raycast should be registered according to its reflectivity and distance.
    public float hitRegistrationConstant; // hitRegistration is calculated as: max distance d for a reflectivity r = hitRegistrationConstant * (r ^ hitRegistrationExponent). If the distance to the object is greater than d, it is not registered.
    public float angleSigma; // Standard deviation of the lidar ray angles, as per datasheet
    public float distanceSigma; // Standard deviation of the lidar ray distances, as per datasheet
    public NativeArray<float2> noisePairs;
    public int desiredNoiseLength; // The length of the noise vector, which is equal to 3 * number of rays being cast every frame
    public int pairCount; // The number of pairs of noise values to generate, which is equal to desiredNoiseLength / 2
    public uint masterSeed; // The master seed for the random number generator, which is used to generate noise values for the lidar rays

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
        Vector3 position = // new Vector3(0, (float)60.10 / 1000 / 2, 1);
         new Vector3(environmentData.course3Width / 2,
                     (float)60.10 / 1000 / 2,
                     2 * environmentData.obstacleDepthSpacing - 3);
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
        inputDirections = new NativeArray<Vector3>(sectionCount * elevationSteps, Allocator.Persistent);
        outputDirections = new NativeArray<Vector3>(sectionCount * elevationSteps, Allocator.Persistent);
        totalSections = environmentData.simRate / scanRate;
        maxDistance = 85f;
        minDistance = 0.1f;
        hitRegistrationExponent = 0.369f; // TODO: Readjust hit registration to match only the 10% and 80% value.
        hitRegistrationConstant = 15.23f;
        angleSigma = 0.15f;
        distanceSigma = 0.03f;

        desiredNoiseLength = Mathf.CeilToInt(sectionCount * elevationSteps * 3); // Number of Gaussian observations needed. Should be equal to 3 * number of rays being cast every frame.
        // Ensure length is even
        if (desiredNoiseLength % 2 != 0)
            desiredNoiseLength++;
        pairCount = desiredNoiseLength / 2;

        // Allocate once; reuse every frame
        noisePairs = new NativeArray<float2>(pairCount, Allocator.Persistent);
        masterSeed = (uint) UnityEngine.Random.Range(int.MinValue, int.MaxValue);

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

        var noiseJob = new MarsagliaNoiseJob
        {
            noisePairs = noisePairs,
            baseSeed = masterSeed
        };
        JobHandle noisehandle = noiseJob.Schedule(pairCount, 64);

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
            azRotation.x = RotationLoader.data[prevSecondInt][0];
            azRotation.y = RotationLoader.data[prevSecondInt][1];
        }
        else if (Mathf.Abs(nextSecond - t0) < Time.fixedDeltaTime)
        {
            azRotation.x = RotationLoader.data[nextSecondInt][0];
            azRotation.y = RotationLoader.data[nextSecondInt][1];
        }
        else // Linearly interpolate between the previous and next az values
        {
            float gradient = RotationLoader.data[nextSecondInt][0] - RotationLoader.data[prevSecondInt][0];
            azRotation.x = RotationLoader.data[prevSecondInt][0] + (t0 - prevSecond) * gradient; // Assign azimuth angle

            gradient = RotationLoader.data[nextSecondInt][1] - RotationLoader.data[prevSecondInt][1];
            azRotation.y = RotationLoader.data[prevSecondInt][1] + (t0 - prevSecond) * gradient; // Assign zenith angle
        }

        int start = (section - 1) * sectionCount * elevationSteps;
        int end = Mathf.Min(section * sectionCount * elevationSteps - 1, azimuthSteps * sectionCount * elevationSteps - 1);
        int length = sectionCount * elevationSteps;
        Debug.Log($"{length} vectors for rotation.");
        Debug.Log($"inputDirection is {inputDirections.Length} long.");
        NativeArray<Vector3>.Copy(rayGrid.directions, start, inputDirections, 0, length);
        
        noisehandle.Complete();
        Debug.Log($"Noise job completed. {noisePairs.Length} pairs generated.");
        Debug.Log($"outputDirections has: {outputDirections.Length} vectors.");
        var rotateJob = new VectorRotationJob
        {
            inputDirections = inputDirections, // TODO: USE THE INPUT ARRAY, NOT ALL DIRECTIONS
            azRotation = azRotation,
            noisePairs = noisePairs,
            angleSigma = angleSigma,
            lidarRotation = this.transform.rotation,
            outputDirections = outputDirections
        };
        JobHandle rotatehandle = rotateJob.Schedule(sectionCount * elevationSteps, 8); // TODO: try playing with the batch size for performance
        rotatehandle.Complete();
        Debug.Log($"Rotate job completed with {outputDirections.Length} vectors.");

        int iter = sectionCount * elevationSteps; // We've already used 1 pair of noise values per ray for azimuth and zenith angles. We will now use the rest of the noise values in noisePairs for distance noise.
        
        for (int ray = 0; ray < sectionCount * elevationSteps; ray++)
        {
            Vector3 dir = outputDirections[ray];
            Vector3 origin = transform.position + dir * minDistance; // Start from minimum distance for object detection

            if (Physics.Raycast(origin, dir, out RaycastHit hit, maxDistance) &&
                hit.distance < (hitRegistrationConstant * Mathf.Pow(hit.collider.GetComponent<Reflectivity>().reflectivity, hitRegistrationExponent))) // TODO: maybe use a lookup table instead of pow to speed up?
            {
                float distanceNoise;
                if (ray % 2 == 0)
                {
                    distanceNoise = noisePairs[iter].x;
                }
                else
                {
                    distanceNoise = noisePairs[iter].y;
                    iter++;
                }


                Debug.DrawLine(origin, hit.point, Color.green); // Draw the ray in green if it hits an object that should be registered
                hit.point += dir * (distanceNoise * distanceSigma); // Apply distance noise to the hit point

                // Create and add a marker at the hit point
                GameObject s = GameObject.CreatePrimitive(PrimitiveType.Sphere); // Create a small sphere at the hit point
                s.transform.position = hit.point;
                s.transform.localScale = Vector3.one * 0.01f;
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
        noisePairs.Dispose();
        inputDirections.Dispose();
        outputDirections.Dispose();
    }


    //--------------------------------------------------------------------------------------------------------------------------------------
    // END OF MEMBER FUNCTIONS
}
