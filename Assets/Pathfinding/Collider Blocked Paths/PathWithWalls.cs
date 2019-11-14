using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PathWithWalls : MonoBehaviour
{
    [SerializeField] Transform start;
    [SerializeField] Transform end;
    //List<Vector2> path = new List<Vector2>();

    void OnDrawGizmos () {
        // BFS
        Gizmos.color = Color.green;
        Gizmos.DrawCube(start.position, Vector3.one * 0.5f);
        Gizmos.color = Color.red;
        Gizmos.DrawCube(end.position, Vector3.one * 0.5f);
        for (int i = 0; i < PathfindingVisual.BFSPath.Count - 1; i++) {
            Gizmos.color = Color.cyan;
            Gizmos.DrawLine (PathfindingVisual.BFSPath[i], PathfindingVisual.BFSPath[i + 1]);
        }
        for (int i = 0; i < PathfindingVisual.BFSSearched.Count; i++) {
            Gizmos.color = Color.blue;
            Gizmos.DrawSphere (PathfindingVisual.BFSSearched[i], 0.15f);
        }
        for (int i = 0; i < PathfindingVisual.AStarPath.Count - 1; i++) {
            Gizmos.color = Color.green;
            Gizmos.DrawLine (PathfindingVisual.AStarPath[i], PathfindingVisual.AStarPath[i + 1]);
        }
        for (int i = 0; i < PathfindingVisual.AStarSearched.Count; i++) {
            Gizmos.color = Color.green;
            Gizmos.DrawWireSphere (PathfindingVisual.AStarSearched[i], 0.15f);
        }
    }

    // Start is called before the first frame update
    void Start()
    {
        //path = Pathfinding.AStar(start.transform.position, end.transform.position, PathGrid.nodes, true, true);
        StartCoroutine(PathfindingVisual.instance.BreadthFirstSearch(start.transform.position, end.transform.position, PathGrid.BFSnodes, 1, true, true));
        PathfindingVisual.instance.AStar(start.transform.position, end.transform.position, PathGrid.nodes, true, true);
    }
}
