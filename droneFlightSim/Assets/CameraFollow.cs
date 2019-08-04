using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraFollow : MonoBehaviour
{

    public GameObject target;
    public float distance = 5;
    public float height = 2;
    private Vector3 offset;
    public float speed = 0.1f;
    // Start is called before the first frame update
    void Start()
    {
    }

    // Update is called once per frame
    void Update()
    {
        float velX = target.GetComponent<Rigidbody>().velocity.x;
        float velZ = target.GetComponent<Rigidbody>().velocity.z;
        Vector2 vel2D = new Vector2(velX, velZ);
        if (vel2D.SqrMagnitude() > 0.5)
        {
            Vector2 offset2D = (-vel2D.normalized) * distance;
            offset = new Vector3(offset2D.x, height, offset2D.y);
        } else
        {
            //Vector3 dir = target.transform.forward;
            //Vector2 dir2D = new Vector2(dir.x, dir.z);
            //Vector2 offset2D = (-dir2D.normalized) * distance;
            //offset = new Vector3(offset2D.x, height, offset2D.y);
        }
        

        transform.position = Vector3.Slerp(transform.position, target.transform.position + offset, speed);
        transform.forward = target.transform.position - transform.position;
    }
}
