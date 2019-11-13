using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CharacterMovement : MonoBehaviour {
    Queue<Vector2> path = new Queue<Vector2> ();
    Vector2 current;
    List<Vector2> newPath = new List<Vector2>();
    bool followingPath = false;
    [SerializeField] Transform target;
    [SerializeField] float speed;
    [SerializeField] bool includeDiagonals = true;
    float speedMod = 1;

    // Start is called before the first frame update
    void Start () {
        current = transform.position;
    }

    void OnDrawGizmos () {
        for (int i = 0; i < newPath.Count - 1; i++) {
            Gizmos.color = Color.blue;
            Gizmos.DrawLine (newPath[i], newPath[i + 1]);
        }
    }

    void Update () {
        if (target) {
            // If there is a path and a current place to move towards
            if (followingPath) {
                // set speed for the frame
                float step = (speed * speedMod) * Time.deltaTime;
                if (Vector3.Distance (transform.position, current) > step + 0.05f) {
                    // move toward node if far enough and if it's set
                    transform.position = Vector3.MoveTowards (transform.position, current, step);
                } else {
                    // Pull next node from path if close enough or stop path if no more nodes
                    if(path.Count > 0){
                        current = path.Dequeue();
                    } else {
                        followingPath = false;
                    }
                    
                }
            } else {                
                if (Vector3.Distance (transform.position, target.transform.position) > 3) {
                    newPath = Pathfinding.AStar (transform.position, target.transform.position, PathGrid.nodes, includeDiagonals);
                    foreach (var node in newPath) {
                        path.Enqueue (node);
                    }
                    followingPath = true;
                }
            }

        }
    }

    void OnTriggerEnter2D(Collider2D col)
    {
        if(col.GetComponent<WeightedObstacle>())
        {
            var o = col.GetComponent<WeightedObstacle>();
            speedMod = 1/o.GetWeight();
        }
    }
    void OnTriggerExit2D(Collider2D col)
    {
        if(col.GetComponent<WeightedObstacle>())
        {
            speedMod = 1;
        }
    }
}