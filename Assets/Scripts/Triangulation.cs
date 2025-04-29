using UnityEngine;

public class Triangulation : MonoBehaviour
{
    public Transform beacon1; // Cube1
    public Transform beacon2; // Cube2
    public Transform beacon3; // Cube3
    
    public Transform capsule;

    void Update()
    {
        // Hitung jarak ke masing-masing beacon
        float d1 = Vector3.Distance(transform.position, beacon1.position);
        float d2 = Vector3.Distance(transform.position, beacon2.position);
        float d3 = Vector3.Distance(transform.position, beacon3.position);

        // Ambil koordinat dari masing-masing beacon
        float x1 = beacon1.position.x;
        float y1 = beacon1.position.z; // Gunakan z karena ini ground plane
        
        float x2 = beacon2.position.x;
        float y2 = beacon2.position.z;
        
        float x3 = beacon3.position.x;
        float y3 = beacon3.position.z;

        // Hitung posisi menggunakan rumus triangulasi
        float A = 2 * (x1 - x2);
        float B = 2 * (y1 - y2);
        float C = d2*d2 - d1*d1 - x2*x2 + x1*x1 - y2*y2 + y1*y1;
        float D = 2 * (x1 - x3);
        float E = 2 * (y1 - y3);
        float F = d3*d3 - d1*d1 - x3*x3 + x1*x1 - y3*y3 + y1*y1;

        // Hitung x dan y
        float x = (C*E - F*B) / (E*A - B*D);
        float y = (C*D - A*F) / (B*D - A*E);

        // Update posisi objek (opsional, tergantung kebutuhan)
        capsule.position = new Vector3(x, capsule.position.y, y);   

        // Visualisasi untuk debugging
        Debug.DrawLine(transform.position, beacon1.position, Color.red);
        Debug.DrawLine(transform.position, beacon2.position, Color.green);
        Debug.DrawLine(transform.position, beacon3.position, Color.blue);
        
        // Debug output
        // Debug.Log($"Calculated Position: ({x}, {y})");
    }
}