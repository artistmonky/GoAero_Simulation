using UnityEngine;

public class SingleRayLidar : MonoBehaviour
{
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        // Set position 
        Vector3 position = new Vector3(environmentData.course3Width / 2,
                                       0,
                                       2 * environmentData.obstacleDepthSpacing - 3); ;//new Vector3(0, (float)60.10 / 1000 / 2, 1);
        this.transform.position = position;
    }

    // Update is called once per frame
    void Update()
    {
        // Cast single lidar ray straight forward
        if (Physics.Raycast(transform.position, Vector3.forward, out RaycastHit hit, 5))
        {
            Debug.DrawLine(transform.position, hit.point, Color.red);

            // Get a point that lies within the unit circle
            // (i.e. 2 observations of a random variable uniformly distributed along range [-1,1] whose sum of squares obey:  0 < sum < 1)
            float x = Random.Range(-1f, 1f);
            float y = Random.Range(-1f, 1f);
            float s = x * x + y * y;
            while (true)
            {
                if (s < 1 && s > 0)
                {
                    break;
                }
                x = Random.Range(-1f, 1f);
                y = Random.Range(-1f, 1f);
                s = x * x + y * y;
            }
            x = Random.Range(-1f, 1f);
            y = Random.Range(-1f, 1f);
            while (true)
            {
                if (s < 1 && s > 0)
                {
                    break;
                }
                x = Random.Range(-1f, 1f);
                y = Random.Range(-1f, 1f);
                s = x * x + y * y;
            }

            // Calculate 2 observations of a standard normal distribution N ~ (0,1) using Marsaglia polar method
            float angular1 = x * Mathf.Sqrt(-2f * Mathf.Ln(s) / s);
            float distance = y * Mathf.Sqrt(-2f * Mathf.Ln(s) / s);

            // Scale the observations to the angular and distance distributions of the MID360
            float angleMeasurement = 




        }


    }
}
