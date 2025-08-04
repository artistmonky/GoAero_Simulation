using UnityEngine;

[RequireComponent(typeof(Camera))]
public class ZED2i : MonoBehaviour
{
    [Header("ZED2i Settings")]
    public float baseline = 0.12f; // 12 cm
    public int imageWidth = 2688;
    public int imageHeight = 1520;
    public float focalLengthPx = 1400f; // Approx. in pixels TODO: ADJUST THIS BASED ON DATASHEET

    [Header("Camera References")]
    public Camera leftCamera;
    public Camera rightCamera;

    [Header("Render Targets")]
    public RenderTexture leftTexture;
    public RenderTexture rightTexture;

    [Header("Output")]
    public Texture2D disparityMap;
    public Texture2D depthMap;

    void Start()
    {
        SetupCameras();
        Vector3 position = new Vector3(0f,
                                       1f,
                                       0f);
        this.transform.position = position;
    }

    void SetupCameras()
    {
        if (!leftCamera || !rightCamera)
        {
            Debug.LogError("Assign both left and right cameras.");
            return;
        }

        // Position cameras with baseline offset
        leftCamera.transform.localPosition = new Vector3(-baseline / 2f, 0f, 0f);
        rightCamera.transform.localPosition = new Vector3(baseline / 2f, 0f, 0f);

        // Set resolution
        leftTexture = new RenderTexture(imageWidth, imageHeight, 24, RenderTextureFormat.ARGB32);
        rightTexture = new RenderTexture(imageWidth, imageHeight, 24, RenderTextureFormat.ARGB32);
        leftCamera.targetTexture = leftTexture;
        rightCamera.targetTexture = rightTexture;

        disparityMap = new Texture2D(imageWidth, imageHeight, TextureFormat.RFloat, false);
        depthMap = new Texture2D(imageWidth, imageHeight, TextureFormat.RFloat, false);
    }

    void LateUpdate()
    {
        RenderAndProcess();
    }

    void RenderAndProcess()
    {
        RenderTexture.active = leftTexture;
        Texture2D leftImage = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);
        leftImage.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
        leftImage.Apply();

        RenderTexture.active = rightTexture;
        Texture2D rightImage = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);
        rightImage.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
        rightImage.Apply();

        RenderTexture.active = null;

        // --- Disparity and Depth Estimation (CPU-based, simple) ---
        for (int y = 0; y < imageHeight; y += 4) // Subsample for speed
        {
            for (int x = 0; x < imageWidth; x += 4)
            {
                Color leftPixel = leftImage.GetPixel(x, y);
                int bestOffset = 0;
                float bestScore = float.MaxValue;

                // Search window (up to 64 pixels left)
                for (int d = 0; d < 64 && (x - d) >= 0; d++)
                {
                    Color rightPixel = rightImage.GetPixel(x - d, y);
                    float diff = (leftPixel - rightPixel).grayscale;
                    float score = diff * diff;

                    if (score < bestScore)
                    {
                        bestScore = score;
                        bestOffset = d;
                    }
                }

                float disparity = bestOffset;
                float depth = (disparity > 0f) ? (baseline * focalLengthPx) / disparity : 0f;

                // Write depth to map
                Color depthColor = new Color(depth / 100f, 0, 0); // Normalize for visualization
                depthMap.SetPixel(x, y, depthColor);
            }
        }

        depthMap.Apply();
        Debug.Log("Depth estimation complete (subsampled).");
    }
}
