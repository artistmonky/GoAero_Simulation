using UnityEngine;
using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;
using Unity.Mathematics;
using UnityEngine.Rendering;
using UnityEngine.Jobs;
using UnityEngine.Experimental.Rendering;
using UnityEngine.XR;
using UnityEngine.Assertions;
using UnityEngine.SceneManagement;
using UnityEngine.PlayerLoop;
using UnityEngine.UI;
using UnityEngine.AI;
using UnityEngine.Animations;
using UnityEngine.EventSystems;
using UnityEngine.InputSystem;
using UnityEngine.Rendering.Universal;
using UnityEngine.VFX;
using UnityEngine.Splines;
using UnityEngine.LowLevel;
using UnityEngine.Profiling;
using UnityEngine.TerrainUtils;
using UnityEngine.TextCore;
using UnityEngine.U2D;
using UnityEngine.Video;
using System.Collections.Generic;
using static MID360_Optimized;

public class MID360_Optimized : MonoBehaviour
{
    [BurstCompile]
    struct PostRaycastProcessingJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<RaycastHit> hits;
        [ReadOnly] public NativeArray<RaycastCommand> commands;
        [ReadOnly] public NativeArray<float> distanceNoises;
        [ReadOnly] public NativeHashMap<int, float> reflectivityMap;
        [ReadOnly] public NativeArray<int> colliderInstanceIDs;
        [ReadOnly] public float hitRegistrationExponent;
        [ReadOnly] public float hitRegistrationConstant;

        [WriteOnly] public NativeList<float3>.ParallelWriter outputPoints;

        public void Execute(int i)
        {
            int id = colliderInstanceIDs[i];
            if (id == -1) return; // No hit

            float r = reflectivityMap.TryGetValue(id, out float reflectivity) ? reflectivity : 1f;
            float threshold = hitRegistrationConstant * math.pow(r, hitRegistrationExponent);

            var hit = hits[i];
            if (hit.distance < threshold)
            {
                float3 offset = commands[i].direction * distanceNoises[i];
                float3 noisyPoint = (float3)hit.point + offset;
                outputPoints.AddNoResize(noisyPoint);
            }
        }

    }


    [BurstCompile]
    struct LidarRaycastJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<Vector3> inputDirections;
        [ReadOnly] public float2 azRotation;
        [ReadOnly] public float angleSigma;
        [ReadOnly] public float distanceSigma;
        [ReadOnly] public quaternion lidarRotation;
        [ReadOnly] public uint baseSeed;
        [ReadOnly] public float minDistance;
        [ReadOnly] public float maxDistance;
        [ReadOnly] public float3 lidarPosition;
        [WriteOnly] public NativeArray<float> distanceNoises;
        [WriteOnly] public NativeArray<RaycastCommand> rayCommands;

        public void Execute(int index)
        {
            float3 dir = inputDirections[index];
            dir = math.rotate(lidarRotation, dir);

            float3 azAngles = math.radians(new float3(-azRotation.y, azRotation.x, 0f));
            quaternion azQuat = quaternion.EulerXYZ(azAngles);

            var rng = new Unity.Mathematics.Random(baseSeed + (uint)index);

            float2 u;
            float s;
            do { u = rng.NextFloat2(-1f, 1f); s = math.lengthsq(u); } while (s >= 1f || s == 0f);
            float2 angularNoise = u * math.sqrt(-2f * math.log(s) / s);

            float3 noiseAngles = math.radians(new float3(angularNoise.y * angleSigma, angularNoise.x * angleSigma, 0f));
            quaternion noiseQuat = quaternion.EulerXYZ(noiseAngles);
            quaternion finalRot = math.mul(azQuat, noiseQuat);
            dir = math.normalize(math.rotate(finalRot, dir));

            do { u = rng.NextFloat2(-1f, 1f); s = math.lengthsq(u); } while (s >= 1f || s == 0f);
            distanceNoises[index] = u.x * math.sqrt(-2f * math.log(s) / s) * distanceSigma;

            float3 origin = lidarPosition + dir * minDistance; //TODO: Check if the rays are coming out of the right place: is the vertical offset of the ray origin point (as per datasheet) correct?
            rayCommands[index] = new RaycastCommand(origin, dir, maxDistance);
        }
    }

    public struct LidarRayGrid
    {
        public int azimuthSteps;
        public int elevationSteps;
        public float maxElevation;
        public float minElevation;
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

        public int Index(int az, int el) => az * elevationSteps + el;

        public void Set(int az, int el, Vector3 dir) => directions[Index(az, el)] = dir;

        public void InitializeLidarGrid()
        {
            for (int az = 0; az < azimuthSteps; az++)
            {
                float yaw = (float)az / azimuthSteps * 360f;
                for (int el = 0; el < elevationSteps; el++)
                {
                    float elevation = Mathf.Lerp(minElevation, maxElevation, (float)el / (elevationSteps - 1));
                    Quaternion rotation = Quaternion.Euler(-elevation, yaw, 0f);
                    Vector3 direction = rotation * Vector3.forward;
                    Set(az, el, direction.normalized);
                }
            }
        }
    }


    public string modelPath;
    List<GameObject> activeMarkers;
    [Header("LiDAR Settings")]
    public int azimuthSteps = 360;
    public int elevationSteps = 40;
    public float maxElevation = 55.22f;
    public float minElevation = -7.22f;
    public float shrinkage; // The shrinkage angle for the lidar rays
    public float angleSigma = 0.15f;
    public float distanceSigma = 0.03f;
    public float maxDistance = 85f;
    public float minDistance = 0.1f;
    public float hitRegistrationExponent = 0.369f;
    public float hitRegistrationConstant = 15.23f;
    public NativeHashMap<int, float> reflectivityMap;
    public float2 azRotation;

    [Header("Scan Sectioning")]
    public int section = 1;
    public int scanRate = 10;
    public int rayCount;
    public int totalSections;

    [Header("Debug")]
    public uint masterSeed;
    public bool visualizeOn; // Set this to true to turn on the visualization of lidar hit points

    NativeArray<float> distanceNoises;
    NativeArray<Vector3> inputDirections;
    NativeArray<RaycastCommand> rayCommands;
    NativeArray<RaycastHit> rayHits;

    NativeArray<int> colliderInstanceIDs;
    NativeList<float3> acceptedPointsList;
    NativeList<float3>.ParallelWriter acceptedPoints;

    JobHandle raycastHandle;
    LidarRayGrid rayGrid;

    void Start()
    {
        visualizeOn = false;

        // Set position
        Vector3 position = // new Vector3(0, (float)60.10 / 1000 / 2, 1);
        new Vector3(environmentData.course3Width / 2,
                   (float)60.10 / 1000 / 2,
                   2 * environmentData.obstacleDepthSpacing - 3);
        this.transform.position = position;
        // TODO: Set rotation if needed

        // Initialize Lidar Grid Constants
        azimuthSteps = 360;
        elevationSteps = 40; // Number of lidar rays in each vertical line, as per datasheet
        shrinkage = 1.72f;
        maxElevation = (55.22f - shrinkage);
        minElevation = (-7.22f + shrinkage);
        rayGrid = new LidarRayGrid(azimuthSteps, elevationSteps, maxElevation, minElevation, Allocator.Persistent);
        scanRate = 10;
        section = 1; // Start at section 1
        rayCount = azimuthSteps * elevationSteps * scanRate / environmentData.simRate;
        //outputDirections = new NativeArray<Vector3>(sectionCount * elevationSteps, Allocator.Persistent);
        maxDistance = 85f;
        minDistance = 0.1f;
        hitRegistrationExponent = 0.369f; // TODO: Readjust hit registration to match only the 10% and 80% value.
        hitRegistrationConstant = 15.23f;
        angleSigma = 0.15f;
        distanceSigma = 0.03f;
        totalSections = azimuthSteps * elevationSteps / rayCount;
        rayGrid = new LidarRayGrid(azimuthSteps, elevationSteps, maxElevation, minElevation, Allocator.Persistent);
        inputDirections = new NativeArray<Vector3>(rayCount, Allocator.Persistent);
        distanceNoises = new NativeArray<float>(rayCount, Allocator.Persistent);
        masterSeed = (uint)UnityEngine.Random.Range(int.MinValue, int.MaxValue);

        // Create hashmap for fast object reflectiviy lookup
        var reflectives = GameObject.FindObjectsOfType<Reflectivity>();
        reflectivityMap = new NativeHashMap<int, float>(reflectives.Length, Allocator.Persistent);

        foreach (var refl in reflectives)
        {
            var collider = refl.GetComponent<Collider>();
            if (collider != null)
                reflectivityMap.TryAdd(collider.GetInstanceID(), refl.reflectivity);
        }

        rayCommands = new NativeArray<RaycastCommand>(rayCount, Allocator.Persistent);
        rayHits = new NativeArray<RaycastHit>(rayCount, Allocator.Persistent);
        colliderInstanceIDs = new NativeArray<int>(rayCount, Allocator.Persistent);
        acceptedPointsList = new NativeList<float3>(rayCount, Allocator.Persistent);
        acceptedPoints = acceptedPointsList.AsParallelWriter();

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

        // Initialize the list of active markers
        if (visualizeOn) activeMarkers = new List<GameObject>();
    }

    void Update()
    {
        float t0 = Time.realtimeSinceStartup;

       if (visualizeOn)
        {
            if (activeMarkers.Count > 90000)
            {
                for (int i = 0; i < activeMarkers.Count; i++) Destroy(activeMarkers[i]);
                activeMarkers.Clear();
            }
        }

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

        int start = (section - 1) * rayCount; // TODO

        NativeArray<Vector3>.Copy(rayGrid.directions, start, inputDirections, 0, rayCount);

        var rayJob = new LidarRaycastJob
        {
            inputDirections = inputDirections,
            azRotation = azRotation,
            angleSigma = angleSigma,
            distanceSigma = distanceSigma,
            lidarRotation = transform.rotation,
            baseSeed = masterSeed,
            minDistance = minDistance,
            maxDistance = maxDistance,
            lidarPosition = transform.position,
            distanceNoises = distanceNoises,
            rayCommands = rayCommands
        };

        JobHandle rayGenHandle = rayJob.Schedule(rayCount, 32);
        raycastHandle = RaycastCommand.ScheduleBatch(rayCommands, rayHits, 32, rayGenHandle);

        float t1 = Time.realtimeSinceStartup;
        Debug.Log($"Update excuted in {(t1 - t0) * 1000f:F3} ms");
    }

    void LateUpdate()
    {
        float t0 = Time.realtimeSinceStartup;

        raycastHandle.Complete();
        for (int i = 0; i < rayCount; i++)
        {
            colliderInstanceIDs[i] = rayHits[i].collider != null
                ? rayHits[i].collider.GetInstanceID()
                : -1;
        }

        acceptedPointsList.Clear();
        acceptedPoints = acceptedPointsList.AsParallelWriter();

        var postJob = new PostRaycastProcessingJob
        {
            hits = rayHits,
            commands = rayCommands,
            distanceNoises = distanceNoises,
            colliderInstanceIDs = colliderInstanceIDs, // now passed in
            reflectivityMap = reflectivityMap,
            hitRegistrationExponent = hitRegistrationExponent,
            hitRegistrationConstant = hitRegistrationConstant,
            outputPoints = acceptedPoints
        };

        JobHandle postHandle = postJob.Schedule(rayHits.Length, 64);
        postHandle.Complete();

        // TS below for visualizing scan points
        if (visualizeOn)
        {
            for (int i = 0; i < acceptedPointsList.Length; ++i)
            {
                GameObject s = GameObject.CreatePrimitive(PrimitiveType.Sphere); // Create a small sphere at the hit point
                s.transform.position = acceptedPointsList[i];
                s.transform.localScale = Vector3.one * 0.01f;
                //s.transform.parent = transform;
                var col = s.GetComponent<Collider>(); // disable its collider
                if (col) col.enabled = false;
                var rend = s.GetComponent<Renderer>();
                if (rend != null)
                {
                    rend.material.color = Color.yellow;
                }
                activeMarkers.Add(s);
            }
        }

        section = section % totalSections + 1;
        float t1 = Time.realtimeSinceStartup;
        Debug.Log($"LateUpdate executed in {(t1 - t0) * 1000f:F3} ms");
    }

    void OnDestroy()
    {
        rayCommands.Dispose();
        rayHits.Dispose();
        distanceNoises.Dispose();
        colliderInstanceIDs.Dispose(); 
        acceptedPointsList.Dispose();
    }
}