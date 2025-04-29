using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BombSeekerRobot : MonoBehaviour
{
    public List<Waypoint> candidateLocations;
    public AStarPathfinding pathfinder;
    public float speed = 2f;
    public float waypointReachedThreshold = 0.1f;

    private int currentTargetIndex = 0;
    private List<Waypoint> currentPath;
    private int pathIndex = 0;
    private bool foundBomb = false;

    private RobotController robotController;

    void Start()
    {
        robotController = GetComponent<RobotController>();
        if (pathfinder == null)
        {
            pathfinder = GetComponent<AStarPathfinding>();
            if (pathfinder == null)
            {
                pathfinder = gameObject.AddComponent<AStarPathfinding>();
                Debug.LogWarning("AStarPathfinding component added automatically");
            }
        }
        GoToNextTarget();
    }

    void Update()
    {
        if (foundBomb || currentPath == null || pathIndex >= currentPath.Count) return;

        // Mendapatkan waypoint target berikutnya dari jalur A*
        Transform target = currentPath[pathIndex].transform;
        
        // Tetapkan waypoint untuk controller robot
        robotController.Waypoint = target;
        
        // Navigasi menuju waypoint
        robotController.Navigate();

        // Cek jika mencapai waypoint
        if (Vector3.Distance(transform.position, target.position) < waypointReachedThreshold)
        {
            // Pindah ke waypoint berikutnya dalam jalur
            pathIndex++;
            
            if (pathIndex >= currentPath.Count)
            {
                // Sampai di kandidat lokasi, tunggu hasil pengecekan trigger
                // Jika tidak ada bom, lanjut ke kandidat berikutnya
                if (!foundBomb)
                {
                    currentTargetIndex++;
                    if (currentTargetIndex < candidateLocations.Count)
                        GoToNextTarget();
                    else
                        Debug.Log("Bomb not found in any location!");
                }
            }
        }
    }

    void GoToNextTarget()
    {
        // Dapatkan waypoint terdekat untuk titik awal A*
        pathfinder.startWaypoint = GetClosestWaypoint();
        
        // Tetapkan lokasi kandidat berikutnya sebagai titik akhir A*
        pathfinder.endWaypoint = candidateLocations[currentTargetIndex];
        
        // Gunakan algoritma A* untuk menemukan jalur
        currentPath = pathfinder.FindPath();
        
        if (currentPath == null || currentPath.Count == 0)
        {
            Debug.LogError("A* pathfinding failed to find a path!");
            return;
        }
        
        // Mulai dari waypoint pertama dalam jalur
        pathIndex = 0;
        
        Debug.Log("Menuju lokasi kandidat " + currentTargetIndex + 
                  " melalui jalur dengan " + currentPath.Count + " waypoints");
    }

    Waypoint GetClosestWaypoint()
    {
        Waypoint[] allWaypoints = FindObjectsOfType<Waypoint>();
        Waypoint closest = null;
        float minDist = Mathf.Infinity;
        
        foreach (var wp in allWaypoints)
        {
            float dist = Vector3.Distance(transform.position, wp.transform.position);
            if (dist < minDist)
            {
                minDist = dist;
                closest = wp;
            }
        }
        return closest;
    }

    // Fungsi ini dipanggil dari trigger lokasi bom
    public void OnBombDetected()
    {
        foundBomb = true;
        Debug.Log("Bomb found! Robot stops.");
        // Lakukan aksi defuse atau animasi di sini
    }
}