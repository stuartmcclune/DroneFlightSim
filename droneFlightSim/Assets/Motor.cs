using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Motor : MonoBehaviour
{
    public GameObject Drone;
    public bool SpinsClockwise;

    public void SetForce(float magnitude)
    {
        Vector3 force = magnitude * transform.up;

        Drone.GetComponent<Rigidbody>().AddForceAtPosition(force, transform.position);

        if (SpinsClockwise)
        {
            Drone.GetComponent<Rigidbody>().AddRelativeTorque(Vector3.up * (magnitude / 50));
        } else
        {
            Drone.GetComponent<Rigidbody>().AddRelativeTorque(-Vector3.up * (magnitude / 50));
        }

        
    }
    
}
