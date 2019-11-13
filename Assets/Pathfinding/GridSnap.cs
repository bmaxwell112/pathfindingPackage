using UnityEngine;

[ExecuteInEditMode]
public class GridSnap : MonoBehaviour
{
    // Update is called once per frame
    void Update()
    {
        SnapToGrid();
    }

    private void SnapToGrid()
    {
        transform.position = new Vector3(   
            
            Mathf.Floor(transform.position.x) + 0.5f,
            Mathf.Floor(transform.position.y) + 0.5f,
            0
            );
    }
}
