using UnityEngine;

[DisallowMultipleComponent]
public class Reflectivity : MonoBehaviour
{
    [Header("Simulation Properties")]
    [Range(0f, 100f)]
    [Tooltip("How reflective this object is, 0 = no reflection, 100 = perfect reflection")]
    public float reflectivity = 20f; // Using an arbitary value of 20% reflectivity for concrete.

}
