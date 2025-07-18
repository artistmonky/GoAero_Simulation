using UnityEngine;

public class MeshSizeLogger : MonoBehaviour
{
    void Start()
    {
        Renderer rend = GetComponentInChildren<Renderer>();
        if (rend != null)
        {
            Vector3 size = rend.bounds.size;
            Debug.Log("Model Size (in meters): " + size);
        }
        else
        {
            Debug.LogWarning("No Renderer found in children of " + gameObject.name);
        }
    }
}
