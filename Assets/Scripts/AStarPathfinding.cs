using System.Collections.Generic;
using UnityEngine;

public class AStarPathfinding : MonoBehaviour
{
    public Waypoint startWaypoint;
    public Waypoint endWaypoint;

    public List<Waypoint> FindPath()
    {
        var openSet = new List<Waypoint> { startWaypoint };
        var cameFrom = new Dictionary<Waypoint, Waypoint>();
        var gScore = new Dictionary<Waypoint, float>();
        var fScore = new Dictionary<Waypoint, float>();

        foreach (var wp in FindObjectsOfType<Waypoint>())
        {
            gScore[wp] = float.MaxValue;
            fScore[wp] = float.MaxValue;
        }
        gScore[startWaypoint] = 0;
        fScore[startWaypoint] = Heuristic(startWaypoint, endWaypoint);

        while (openSet.Count > 0)
        {
            Waypoint current = openSet[0];
            foreach (var node in openSet)
                if (fScore[node] < fScore[current]) current = node;

            if (current == endWaypoint)
                return ReconstructPath(cameFrom, current);

            openSet.Remove(current);

            foreach (var neighbor in current.neighbors)
            {
                float tentativeGScore = gScore[current] + Vector3.Distance(current.transform.position, neighbor.transform.position);
                if (tentativeGScore < gScore[neighbor])
                {
                    cameFrom[neighbor] = current;
                    gScore[neighbor] = tentativeGScore;
                    fScore[neighbor] = gScore[neighbor] + Heuristic(neighbor, endWaypoint);
                    if (!openSet.Contains(neighbor))
                        openSet.Add(neighbor);
                }
            }
        }
        return null; // Tidak ada jalur
    }

    float Heuristic(Waypoint a, Waypoint b)
    {
        return Vector3.Distance(a.transform.position, b.transform.position);
    }

    List<Waypoint> ReconstructPath(Dictionary<Waypoint, Waypoint> cameFrom, Waypoint current)
    {
        var totalPath = new List<Waypoint> { current };
        while (cameFrom.ContainsKey(current))
        {
            current = cameFrom[current];
            totalPath.Insert(0, current);
        }
        return totalPath;
    }
}