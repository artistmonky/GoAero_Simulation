using UnityEngine;

public class CameraSpawner : MonoBehaviour
{


    void Start()
    {
        // Add a Camera component to this GameObject
        Camera cam = gameObject.GetComponent<Camera>();
        if (cam == null)
        {
            cam = gameObject.AddComponent<Camera>();
        }

        // Set position and rotation
        transform.position = Vector3.zero;
        transform.rotation = Quaternion.Euler(0, 0, 0);

        // Set tag to MainCamera if needed
        if (Camera.main == null)
        {
            gameObject.tag = "MainCamera";
        }

        Debug.Log("Camera set up at " + transform.position);
    }
}
