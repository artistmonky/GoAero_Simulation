using UnityEngine;

public class SimManager : MonoBehaviour
{
    [Header("Simulation Settings")]
    [Tooltip("Maximum rendering frame rate (set to -1 for unlimited)")]
    public int targetFrameRate;

    [Tooltip("Physics simulation rate in Hz (FixedUpdate)")]
    public float physicsHz;

    void Start()
    {
        targetFrameRate = environmentData.simRate;
        physicsHz = environmentData.simRate;
        Application.targetFrameRate = targetFrameRate;
        QualitySettings.vSyncCount = 0; // Disable VSync so frame rate cap takes effect

        Time.fixedDeltaTime = 1f / physicsHz;

        Debug.Log($"Simulation rate set: FrameRate = {targetFrameRate} FPS, FixedUpdate = {physicsHz} Hz");
    }

}
