using UnityEngine;
using System.Globalization;  // for parsing with invariant culture

public class RotationLoader : MonoBehaviour
{
    public static float[][] data;

    void Start()
    {
        // Load the CSV as plain text
        TextAsset csvAsset = Resources.Load<TextAsset>("mid360");
        if (csvAsset == null)
        {
            Debug.LogError("mid360.csv not found in Resources folder!");
            return;
        }

        // Split into lines
        string[] lines = csvAsset.text.Split('\n');

        // Allocate jagged array of floats
        data = new float[lines.Length][];

        for (int i = 0; i < lines.Length; i++)
        {
            // Trim carriage returns and skip empty lines
            string line = lines[i].Trim('\r');
            if (string.IsNullOrWhiteSpace(line))
            {
                data[i] = new float[0];
            }
            else
            {
                // Split into string fields
                var cols = line.Split(',');

                // Parse each field as float
                var row = new float[cols.Length];
                for (int j = 0; j < cols.Length; j++)
                {
                    // using InvariantCulture to ensure "3.14" works regardless of locale
                    row[j] = float.Parse(cols[j], CultureInfo.InvariantCulture);
                }
                data[i] = row;
            }
        }

        // Example: access row 1, column 2 (zero-based indices)
        if (data.Length > 0 && data[0].Length > 1)
        {
            float firstRowSecondCol = data[0][1];
            Debug.Log($"Row 0, Col 1 = {firstRowSecondCol}");
        }
    }

    void Update()
    {

    }
}
