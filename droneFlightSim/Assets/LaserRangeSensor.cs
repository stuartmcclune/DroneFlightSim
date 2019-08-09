using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LaserRangeSensor : MonoBehaviour
{
    public float Range = 5;

    public float GetDistance()
    {
        int layerMask = 1 << 8;

        RaycastHit hit;
        // Casts ray in local Z axis
        if (Physics.Raycast(transform.position, transform.TransformDirection(Vector3.forward), out hit, Range, layerMask))
        {
            float distance = hit.distance;
            // TODO: add gaussian error term. Maybe scale sd with angle of incidence.
            return distance;
        }
        return -1;
    }
}
