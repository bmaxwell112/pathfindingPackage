using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PathfindingVisual : MonoBehaviour
{
    public static PathfindingVisual instance;

    private void Awake() {
        instance = this;
    }

    const float WAITTIME = 0.15f;
    static Vector2[] cDirections = {
        Vector2.up,
        Vector2.right,
        Vector2.down,
        Vector2.left
    };
    static Vector2[] directions = {
        (Vector2.up + Vector2.right),
        (Vector2.down + Vector2.right),
        (Vector2.down + Vector2.left),
        (Vector2.up + Vector2.left)
    };

    #region BFS
    public static List<Vector2> BFSPath = new List<Vector2>();
    public static List<Vector2> BFSSearched = new List<Vector2>();

    public IEnumerator BreadthFirstSearch (Vector2 startPos, Vector2 endPos, List<Vector2> grid, int gridScale = 1, bool includeDiagonals = true, bool wallBlock = false) {
        BFSPath.Clear();
        BFSSearched.Clear();
        Dictionary<Vector2, BFSNode> nodes = new Dictionary<Vector2, BFSNode> ();
        Vector2 start = startPos;
        Vector2 end = endPos;

        float shortestStartDistance = 100;
        float shortestEndDistance = 100;
        // Set Vector 2 to closest valid nodes
        Vector2ToNodes (nodes, ref start, ref end, ref shortestStartDistance, ref shortestEndDistance, startPos, endPos, grid);
        Queue<BFSNode> queue = new Queue<BFSNode> ();
        bool endFound = false;
        BFSNode current;

        queue.Enqueue (nodes[start]);

        while (queue.Count > 0 && !endFound) {            
            //Debug.Log("running");
            yield return new WaitForSeconds(WAITTIME);
            current = queue.Dequeue ();

            if (current == nodes[end]) {
                endFound = true;
            }
            // Explor Neighbors
            ExploreNeighbors (nodes, queue, current, gridScale, includeDiagonals, wallBlock);
            current.explored = true;
            BFSSearched.Add(current.position);
            BFSPath = GetPath (nodes, start, current.position);
            // yield return new WaitForSeconds (0.05f);
        }
    }

    private static List<Vector2> GetPath (Dictionary<Vector2, BFSNode> nodes, Vector2 start, Vector2 end) {
        List<Vector2> path = new List<Vector2> ();
        path.Add (end);
        BFSNode previous = nodes[end].parent;
        //Debug.Log(previous.parent);
        while (previous != nodes[start] && previous != null) {
            path.Add (previous.position);
            previous = previous.parent;
        }
        path.Add (nodes[start].position);
        path.Reverse ();
        return path;
    }

    private static void Vector2ToNodes (Dictionary<Vector2, BFSNode> nodes, ref Vector2 start, ref Vector2 end, ref float shortestStartDistance, ref float shortestEndDistance, Vector2 startPos, Vector2 endPos, List<Vector2> grid) {
        foreach (var item in grid) {
            nodes.Add (item, new BFSNode (item));
            float sdistance = Vector2.Distance (startPos, item);
            float edistance = Vector2.Distance (endPos, item);
            if (shortestStartDistance > sdistance) {
                shortestStartDistance = sdistance;
                start = item;
            }
            if (shortestEndDistance > edistance) {
                shortestEndDistance = edistance;
                end = item;
            }
        }
    }

    private static void ExploreNeighbors (Dictionary<Vector2, BFSNode> nodes, Queue<BFSNode> queue, BFSNode current, int gridScale, bool includeDiagonals, bool wallBlock) {
        bool[] found = new bool[4];
        for (int i = 0; i < cDirections.Length; i++) {    
            Vector2 neighborCoordinate = current.position + cDirections[i] * gridScale;
            if (nodes.ContainsKey (neighborCoordinate) && !wallBlocking(current.position, neighborCoordinate, wallBlock)) {
                found[i] = true;
                QueueNewNeighbors (nodes, queue, current, neighborCoordinate);
            }
        }
        if (includeDiagonals) {
            for (int i = 0; i < directions.Length; i++) {
                Vector2 neighborCoordinate = current.position + directions[i] * gridScale;
                if (nodes.ContainsKey (neighborCoordinate) && DiagonalCheck (i, found) && !wallBlocking(current.position, neighborCoordinate, wallBlock)) {
                    found[i] = true;
                    QueueNewNeighbors (nodes, queue, current, neighborCoordinate);
                }
            }
        }
    }

    private static void QueueNewNeighbors (Dictionary<Vector2, BFSNode> nodes, Queue<BFSNode> queue, BFSNode current, Vector2 neighborCoordinate) {
        BFSNode neighbor = nodes[neighborCoordinate];
        if (neighbor.explored || queue.Contains (neighbor)) {
            //do nothing
        } else {
            neighbor.parent = current;

            queue.Enqueue (neighbor);
        }
    }

    static bool DiagonalCheck (int i, bool[] found) {
        int j = (i >= 3) ? 0 : i + 1;
        //Debug.Log(i);
        return (found[i] && found[j]);
    }

    static bool wallBlocking (Vector2 start, Vector2 end, bool wallBlock) {
        if (wallBlock) {
            var hit = Physics2D.Linecast (start, end);
            if (hit) {
                if(hit.collider.GetComponent<Obstacle>())
                {
                    return true;
                }
            }
        }
        return false;
    }

    #endregion
    #region Astar

    public static List<Vector2> AStarPath = new List<Vector2>();
    public static List<Vector2> AStarSearched = new List<Vector2>();

    public void AStar (Vector2 startPos, Vector2 endPos, List<Vector2> grid, bool includeDiagonals = true, bool checkForWalls = false) {

        Dictionary<Vector2, AStarNode> allNodes = new Dictionary<Vector2, AStarNode> ();
        Vector2 start = startPos;
        Vector2 end = endPos;
        float shortestStartDistance = 100;
        float shortestEndDistance = 100;
        // Turn provided vector2 into spots on the grid
        foreach (var item in grid) {
            AStarVector v = new AStarVector (item);
            allNodes.Add (item, new AStarNode (v, checkForWalls));
            float sdistance = Vector2.Distance (startPos, item);
            float edistance = Vector2.Distance (endPos, item);
            if (shortestStartDistance > sdistance) {
                shortestStartDistance = sdistance;
                start = item;
            }
            if (shortestEndDistance > edistance) {
                shortestEndDistance = edistance;
                end = item;
            }
        }

        StartCoroutine(AStarAlgorithm (includeDiagonals, allNodes, start, end));
    }
    public void AStar (Vector2 startPos, Vector2 endPos, List<AStarVector> grid, bool includeDiagonals = true, bool checkForWalls = false) {

        Dictionary<Vector2, AStarNode> allNodes = new Dictionary<Vector2, AStarNode> ();
        Vector2 start = startPos;
        Vector2 end = endPos;
        float shortestStartDistance = 100;
        float shortestEndDistance = 100;
        // Turn provided vector2 into spots on the grid
        foreach (var item in grid) {

            if (!allNodes.ContainsKey (item.pos)) {
                allNodes.Add (item.pos, new AStarNode (item, checkForWalls));
            }
            float sdistance = Vector2.Distance (startPos, item.pos);
            float edistance = Vector2.Distance (endPos, item.pos);
            if (shortestStartDistance > sdistance) {
                shortestStartDistance = sdistance;
                start = item.pos;
            }
            if (shortestEndDistance > edistance) {
                shortestEndDistance = edistance;
                end = item.pos;
            }
        }

        StartCoroutine(AStarAlgorithm (includeDiagonals, allNodes, start, end));
    }

    private IEnumerator AStarAlgorithm (bool includeDiagonals, Dictionary<Vector2, AStarNode> allNodes, Vector2 start, Vector2 end) {
        List<AStarNode> openSet = new List<AStarNode> ();
        Dictionary<Vector2, AStarNode> closedSet = new Dictionary<Vector2, AStarNode> ();
        bool endFound = false;
        Debug.Log(start + " " + allNodes.Count );
        AStarNode current = allNodes[start];

        openSet.Add (allNodes[start]);
        while (openSet.Count > 0) {
            // Find best guess f   
            yield return new WaitForSeconds(WAITTIME);
            int winner = 0;
            for (int i = 0; i < openSet.Count; i++) {
                if (openSet[i].f < openSet[winner].f) {
                    winner = i;
                }
            }
            current = openSet[winner];
            // End If Done
            if (current.pos == end) {
                endFound = true;
                Debug.Log ("Found Path");
                break;
            }
            // Add current to closed set 
            openSet.Remove (current);
            closedSet.Add (current.pos, current);

            // Set Neighbors
            current.AddNeighbors (allNodes, directions, cDirections, includeDiagonals);
            for (int i = 0; i < current.neighbors.Count; i++) {
                var neighbor = current.neighbors[i];
                if (!closedSet.ContainsKey (neighbor.pos)) {
                    //Debug.Log("Comparing " + current.pos + " Neighbor " + neighbor.pos);
                    // For varience change 1 to number associated with node
                    // For node n, gScore[n] is the cost of the cheapest path from start to n currently known.
                    var tempG = current.g + Vector2.Distance (current.pos, neighbor.pos) * current.w;
                    bool newPath = false;
                    if (openSet.Contains (neighbor)) {
                        if (tempG < neighbor.g) {
                            neighbor.g = tempG;
                            newPath = true;
                        }
                    } else {
                        neighbor.g = tempG;
                        newPath = true;
                        openSet.Add (neighbor);
                    }
                    if (newPath) {
                        neighbor.h = Heuristic (neighbor, end);
                        neighbor.f = neighbor.g + neighbor.h;
                        neighbor.parent = current;
                    }
                }
            }
            List<Vector2> path = new List<Vector2> ();
            var temp = current;
            path.Add (temp.pos);
            while (temp.parent != null) {
                path.Add (temp.parent.pos);
                temp = temp.parent;
            }
            AStarPath = path;
            AStarSearched.Add(current.pos);
        }
        if (!endFound) {
            Debug.Log ("No path found");
        } else {
            List<Vector2> path = new List<Vector2> ();
            var temp = current;
            path.Add (temp.pos);
            while (temp.parent != null) {
                path.Add (temp.parent.pos);
                temp = temp.parent;
            }
            AStarPath = path;
            AStarSearched.Add(current.pos);
        }
    }

    private static float Heuristic (AStarNode neighbor, Vector2 end) {
        // euclidean distance
        // var d = Vector2.Distance (neighbor.pos, end);
        // manhattan distance
        var d = Mathf.Abs (neighbor.pos.x - end.x) + Mathf.Abs (neighbor.pos.y - end.y);
        return d;
    }

    #endregion
}
