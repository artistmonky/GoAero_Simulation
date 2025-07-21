using UnityEngine;

public class RotationLoader : MonoBehaviour
{
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

        // Allocate jagged array
        string[][] data = new string[lines.Length][];

        for (int i = 0; i < lines.Length; i++)
        {
            // Trim carriage returns and skip empty lines
            string line = lines[i].Trim('\r');
            if (string.IsNullOrWhiteSpace(line))
            {
                data[i] = new string[0];
            }
            else
            {
                // Split into fields
                data[i] = line.Split(',');
            }
        }

        // Example: access row 1, column 2 (zero-based indices)
        if (data.Length > 0 && data[0].Length > 1)
        {
            string firstRowSecondCol = data[0][1];
            Debug.Log($"Row 1, Col 2 = {firstRowSecondCol}");
        }
    }
}