using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Target : MonoBehaviour
{
    public float flySpeed = 1.0f;
    public float rotateSpeed = 2.0f;

    private float yaw = 0.0f;
    private float pitch = 0.0f;

    // Update is called once per frame
    void LateUpdate()
    {
        gameObject.transform.Translate(new Vector3(Input.GetAxis("Horizontal") * flySpeed, Input.GetAxis("Fly") * flySpeed, Input.GetAxis("Vertical") * flySpeed));

        //yaw += rotateSpeed * Input.GetAxis("Mouse X");
        //pitch -= rotateSpeed * Input.GetAxis("Mouse Y");
        //gameObject.transform.eulerAngles = new Vector3(pitch, yaw, 0.0f);
    }
}
