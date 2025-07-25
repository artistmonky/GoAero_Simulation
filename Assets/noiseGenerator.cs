using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;



public class noiseGenerator : MonoBehaviour
{

    NativeArray<float> noiseArray;
    int pairCount;
    uint masterSeed;

    void Update()
    {
        

        // Schedule one Execute() per noise-pair
        
        

        // If you need the data immediately on the main thread, read it now:
        // float[] managedNoise = noiseArray.ToArray();
        // … process managedNoise …

        // Otherwise you could pass noiseArray into other jobs or compute in place
    }

    void OnDestroy()
    {
        if (noiseArray.IsCreated)
            noiseArray.Dispose();
    }
}
