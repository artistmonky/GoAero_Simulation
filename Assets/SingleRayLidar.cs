using UnityEngine;
using System.IO;

public class SingleRayLidar : MonoBehaviour
{
    private string filePath;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        // Set position 
        Vector3 position = new Vector3(environmentData.course3Width / 2,
                                       0,
                                       2 * environmentData.obstacleDepthSpacing - 3); ;//new Vector3(0, (float)60.10 / 1000 / 2, 1);
        this.transform.position = position;


        filePath = Path.Combine(Application.persistentDataPath, "noiseLog.csv");
        // If file doesn’t exist yet, write headers first
        if (!File.Exists(filePath))
        {
            File.WriteAllText(filePath, "angular1,angular2,distance\n");
        }
        Debug.Log("CSV path: " + Application.persistentDataPath);
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
            float angular1 = x * Mathf.Sqrt(-2f * Mathf.Log(s) / s); // Marsaglia polar method for generating observations of standard normal distributions N(0,1)
            float angular2 = y * Mathf.Sqrt(-2f * Mathf.Log(s) / s); // These 2 floats correspond to the angular noise distribution

            x = Random.Range(-1f, 1f);
            y = Random.Range(-1f, 1f);
            s = x * x + y * y;
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
            float distance = x * Mathf.Sqrt(-2f * Mathf.Log(s) / s); // Calculate another observation of N(0,1) for distance noise

            string line = string.Format("{0},{1:F2},{2:F2}\n",
                                        angular1,
                                        angular2,
                                        distance);

            File.AppendAllText(filePath, line);


            // Scale the observations to the angular and distance distributions of the MID360
            //float angleMeasurement = ;
        }


    }
}
