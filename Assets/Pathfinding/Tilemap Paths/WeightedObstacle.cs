using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Collider2D))]
public class WeightedObstacle : MonoBehaviour
{
    [SerializeField] float weight = 1;

    public float GetWeight() { return weight; }
}
