using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SonarSensor : MonoBehaviour
{
    private List<Collider> intersectingColliders = new List<Collider>();

    private void OnTriggerEnter(Collider other)
    {
        intersectingColliders.Add(other);
    }

    private void OnTriggerExit(Collider other)
    {
        intersectingColliders.Remove(other);
    }

    public float GetDistance()
    {
        // Can't use raycast since sonar sees in a cone.
        // Maintain list of objects within cone shape collider.
        // Can't just look for distance to centre or edge of obstacles though...
        return -1;
    }
}
