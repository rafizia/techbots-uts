using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BombSeekerRobot : MonoBehaviour
{
    public List<Waypoint> candidateLocations;
    public AStarPathfinding pathfinder;
    public float speed = 2f;

    private int currentTargetIndex = 0;
    private List<Waypoint> currentPath;
    private int pathIndex = 0;
    private bool foundBomb = false;

    private RobotController robotController;

    void Start()
    {
        robotController = GetComponent<RobotController>();
        GoToNextTarget();
    }

    void Update()
    {
        if (foundBomb || currentPath == null || pathIndex >= currentPath.Count) return;

        Transform target = currentPath[pathIndex].transform;
        // transform.position = Vector3.MoveTowards(transform.position, target.position, speed * Time.deltaTime);
        robotController.Waypoint = target;
        robotController.Navigate();

        if (Vector3.Distance(transform.position, target.position) < 0.1f)
        {
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
        pathfinder.startWaypoint = GetClosestWaypoint();
        pathfinder.endWaypoint = candidateLocations[currentTargetIndex];
        currentPath = pathfinder.FindPath();
        pathIndex = 0;
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