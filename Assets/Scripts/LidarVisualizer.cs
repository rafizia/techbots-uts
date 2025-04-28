using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LidarVisualizer : MonoBehaviour
{
    [SerializeField] private int rayCount = 360;
    [SerializeField] private float maxDistance = 10f;
    [SerializeField] private float drawDuration = 0.05f;
    [SerializeField] private Color rayColor = Color.green;
    [SerializeField] private Color hitColor = Color.red;
    [SerializeField] private LayerMask obstacleLayer;
    
    private void Update()
    {
        VisualizeLidar();
    }
    
    private void VisualizeLidar()
    {
        for (int i = 0; i < rayCount; i++)
        {
            float angle = i * (360f / rayCount);
            Vector3 direction = Quaternion.Euler(0, angle, 0) * Vector3.forward;
            
            Ray ray = new Ray(transform.position + Vector3.up * 0.5f, direction);
            RaycastHit hit;
            
            if (Physics.Raycast(ray, out hit, maxDistance, obstacleLayer))
            {
                Debug.DrawRay(ray.origin, direction * hit.distance, hitColor, drawDuration);
            }
            else
            {
                Debug.DrawRay(ray.origin, direction * maxDistance, rayColor, drawDuration);
            }
        }
    }
}
