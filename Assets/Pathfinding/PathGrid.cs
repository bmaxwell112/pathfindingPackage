using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PathGrid : MonoBehaviour {
    [SerializeField] Transform start;
    [SerializeField] Transform end;
    [SerializeField] bool weighted;
    public const float gridOffset = 0.5f;
    // public static List<Vector2> path = new List<Vector2> ();
    // public static List<Vector2> path2 = new List<Vector2> ();
    public static List<AStarVector> nodes = new List<AStarVector> ();

    // Start is called before the first frame update
    void Start () {
        nodes = new List<AStarVector> ();
        for (int i = Mathf.FloorToInt (start.position.x); i < Mathf.FloorToInt (end.position.x) + 1; i++) {
            for (int j = Mathf.FloorToInt (start.position.y); j < Mathf.FloorToInt (end.position.y) + 1; j++) {
                Vector3 node = new Vector3 (i + gridOffset, j + gridOffset, 0);
                RaycastHit2D hit = Physics2D.Raycast (node, Vector2.zero);
                bool wall = false;
                if (hit) {
                    if (hit.collider.GetComponent<Obstacle>()) {
                        wall = true;
                    }
                    if(hit.collider.GetComponent<WeightedObstacle>()){
                        var o = hit.collider.GetComponent<WeightedObstacle>();
                        nodes.Add (new AStarVector(new Vector2 (i + gridOffset, j + gridOffset), o.GetWeight()));
                    }
                }
                if (!wall) {
                    nodes.Add (new AStarVector(new Vector2 (i + gridOffset, j + gridOffset), 1));
                }
            }
        }        
    }
    void OnDrawGizmos () {
        // Draw a yellow sphere at the transform's position
        Gizmos.color = Color.black;
        foreach (var item in nodes) {
            Gizmos.DrawSphere (item.pos, 0.05f);
        }
    }

    // Update is called once per frame
    void Update () {
        if (Input.GetMouseButtonDown (0)) {
            Vector2 pz = Camera.main.ScreenToWorldPoint (Input.mousePosition);
            RaycastHit2D hit = Physics2D.Raycast (pz, Vector2.zero);
            if (hit) {
                if (hit.collider.tag == "obsticle") {
                    Debug.Log ("Hit");
                }
            }
        }
    }
}