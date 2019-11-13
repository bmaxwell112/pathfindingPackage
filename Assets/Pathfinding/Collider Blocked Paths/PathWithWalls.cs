using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PathWithWalls : MonoBehaviour
{
    [SerializeField] Transform start;
    [SerializeField] Transform end;
    List<Vector2> path = new List<Vector2>();

    void OnDrawGizmos () {
        for (int i = 0; i < path.Count - 1; i++) {
            Gizmos.color = Color.blue;
            Gizmos.DrawLine (path[i], path[i + 1]);
        }
    }

    // Start is called before the first frame update
    void Start()
    {
        path = Pathfinding.AStar(start.transform.position, end.transform.position, PathGrid.nodes, true, true);
    }
}
