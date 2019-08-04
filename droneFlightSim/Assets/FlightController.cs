using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FlightController : MonoBehaviour
{
    public Motor MotorFL;
    public Motor MotorFR;
    public Motor MotorBL;
    public Motor MotorBR;

    public Target Target;
    //public Vector3 Target;
    public float TargetYaw;
    public float Kp = 1;
    public float Ki = 1;
    public float Kd = 1;
    public float MaxForce = 20;

    public Rigidbody Body;

    private float prevErrorY = 0;
    private float integralY = 0;

    private float prevErrorZ = 0;
    private float integralZ = 0;

    private float prevErrorX = 0;
    private float integralX = 0;

    private float prevErrorPitch = 0;
    private float integralPitch = 0;

    private float prevErrorRoll = 0;
    private float integralRoll = 0;

    private float prevErrorYaw = 0;
    private float integralYaw = 0;

    void FixedUpdate()
    {

        float forceY = getForceY();
        Debug.Log(forceY);
        float forceZ = getForceZ();
        float forceX = getForceX();

        //Note: forces are calculated in world axis. yaw will misalign the drone with these axis. Maybe redesign to use InverseTransformPoint to find target in local space, and find local forces. However, may only want rotation of axis in Y axis.
        //currently keeps yaw at 0.
        float yaw = getYaw();
        
        //Bug: if forces are too great, causes wrong rotation. Need to clamp resultant first... carefully. Maybe clamp (global) Y, then use rest of power between X,Z.
        float pitch = getPitch(forceZ, forceY);
        
        float roll = getRoll(forceX, forceY);

        //TODO: system to ensure drone will NEVER hit objects e.g. floor. Needs to limit velocity in a given direction given distance to obstacle s.t. force to avoid is always possible. Multiple directions must be considered.
        //TODO: fix flying down with negative forceY - might be bigger issue with all negative forces.
        
        float force = fly(forceY);

        force = Mathf.Min(MaxForce, force);
        force = Mathf.Max(-MaxForce, force);

        MotorFL.SetForce(force + yaw - pitch + roll);
        MotorFR.SetForce(force - yaw - pitch - roll);
        MotorBL.SetForce(force - yaw + pitch + roll);
        MotorBR.SetForce(force + yaw + pitch - roll);
    }

    private float getForceY()
    {
        float errorY = Target.gameObject.transform.position.y - transform.position.y;
        integralY += errorY * Time.fixedDeltaTime;
        float derivativeY = (errorY - prevErrorY) / Time.fixedDeltaTime;
        float momentumY = Body.velocity.y * Body.mass;

        float pidOutY = Kp * errorY + Ki * integralY + Kd * derivativeY;

        prevErrorY = errorY;

        float forceY = Body.mass * pidOutY + (9.81f * Body.mass) - momentumY;
        return forceY;
    }

    private float getForceZ()
    {
        float errorZ = Target.gameObject.transform.position.z - transform.position.z;
        integralZ += errorZ * Time.fixedDeltaTime;
        float derivativeZ = (errorZ - prevErrorZ) / Time.fixedDeltaTime;
        float momentumZ = Body.velocity.z * Body.mass;

        float pidOutZ = Kp * errorZ + Ki * integralZ + Kd * derivativeZ;

        prevErrorZ = errorZ;

        float forceZ = Body.mass * pidOutZ - momentumZ;
        
        return forceZ;
    }

    private float getForceX()
    {
        float errorX = Target.gameObject.transform.position.x - transform.position.x;
        integralX += errorX * Time.fixedDeltaTime;
        float derivativeX = (errorX - prevErrorX) / Time.fixedDeltaTime;
        float momentumX = Body.velocity.x * Body.mass;

        float pidOutX = Kp * errorX + Ki * integralX + Kd * derivativeX;

        prevErrorX = errorX;

        float forceX = Body.mass * pidOutX - momentumX;
        return forceX;
    }

    private float getPitch(float forceZ, float forceY)
    {
        float pitchTarget = Mathf.Atan2(forceZ, forceY);

        float error = pitchTarget - Mathf.Atan2(transform.up.z, transform.up.y);
        
        integralPitch += error * Time.fixedDeltaTime;
        float derivativePitch = (error - prevErrorPitch) / Time.fixedDeltaTime;
        float angularMomentum = Body.inertiaTensor.x * Body.angularVelocity.x;

        float pidOut = Kp * error + Ki * integralPitch + Kd * derivativePitch;
        
        prevErrorPitch = error;
        
        float torque = (Body.inertiaTensor.x * pidOut - angularMomentum) / 4;
        return torque;
    }

    private float getRoll(float forceX, float forceY)
    {
        float rollTarget = Mathf.Atan2(forceX, forceY);

        float error = rollTarget - Mathf.Atan2(transform.up.x, transform.up.y);

        integralRoll += error * Time.fixedDeltaTime;
        float derivativeRoll = (error - prevErrorRoll) / Time.fixedDeltaTime;
        float angularMomentum = Body.inertiaTensor.z * Body.angularVelocity.z;

        float pidOut = Kp * error + Ki * integralRoll + Kd * derivativeRoll;

        prevErrorRoll = error;

        float torque = (Body.inertiaTensor.z * pidOut - angularMomentum) / 4;
        return torque;
    }

    private float getYaw()
    {
        float error = TargetYaw - transform.rotation.eulerAngles.y;

        while(error < -180)
        {
            error += 360;
        }

        integralYaw += error * Time.fixedDeltaTime;
        float derivativeYaw = (error - prevErrorYaw) / Time.fixedDeltaTime;
        float angularMomentum = Body.inertiaTensor.y * Body.angularVelocity.y;

        float pidOut = Kp * error + Ki * integralYaw + Kd * derivativeYaw;
        prevErrorYaw = error;

        float torque = (Body.inertiaTensor.y * pidOut - angularMomentum) / 40;
        return torque;
    }

    private float fly(float force)
    {      
        float rotX = transform.rotation.eulerAngles.x;
        float rotZ = transform.rotation.eulerAngles.z;

        //TODO: Check bigRot logic
        float bigRot = Mathf.Max(rotX, rotZ);
        float forceMag = (force / 4) / Mathf.Cos(bigRot * Mathf.Deg2Rad);
        return forceMag;
    }
}
