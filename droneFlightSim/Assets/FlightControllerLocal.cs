using UnityEngine;

public class FlightControllerLocal : MonoBehaviour
{
    public Motor MotorFL;
    public Motor MotorFR;
    public Motor MotorBL;
    public Motor MotorBR;

    public LaserRangeSensor LaserRangeSensorX;
    public LaserRangeSensor LaserRangeSensorNX;
    public LaserRangeSensor LaserRangeSensorY;
    public LaserRangeSensor LaserRangeSensoNY;
    public LaserRangeSensor LaserRangeSensorZ;
    public LaserRangeSensor LaserRangeSensorNZ;

    public Target Target;
    //public Vector3 Target;
    public float TargetYaw;
    public float KpDist = 0.8f;
    public float KiDist = 0;
    public float KdDist = 2.8f;
    public float KpAngle = 1;
    public float KiAngle = 2;
    public float KdAngle = 5;
    public float KpYaw = 1;
    public float KiYaw = 1;
    public float KdYaw = 1;
    public float MaxForce = 8;

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
        Debug.Log(LaserRangeSensorZ.GetDistance());

        float forceY = getForceY();
        float forceZ = getForceZ();
        float forceX = getForceX();

        Vector3 forceLocal = transform.InverseTransformVector(forceX, forceY, forceZ);
       
        //currently keeps yaw at 0.
        float yaw = getYaw();

        //Bug: if forces are too great, causes wrong rotation. Need to clamp resultant first... carefully. Maybe clamp (global) Y, then use rest of power between X,Z. Maybe add gravity after getForceY() too.
        float pitch = getPitch(forceLocal.z, forceLocal.y);

        float roll = getRoll(forceLocal.x, forceLocal.y);

        //TODO: system to ensure drone will NEVER hit objects e.g. floor. Needs to limit velocity in a given direction given distance to obstacle s.t. force to avoid is always possible. Multiple directions must be considered.

        float force = forceLocal.y / 4;

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

        float pidOutY = KpDist * errorY + KiDist * integralY + KdDist * derivativeY;

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

        float pidOutZ = KpDist * errorZ + KiDist * integralZ + KdDist * derivativeZ;

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

        float pidOutX = KpDist * errorX + KiDist * integralX + KdDist * derivativeX;

        prevErrorX = errorX;

        float forceX = Body.mass * pidOutX - momentumX;
        return forceX;
    }

    private float getPitch(float forceZ, float forceY)
    {
        float error = Vector3.SignedAngle(Vector3.up, new Vector3(0, Mathf.Abs(forceY), forceZ), Vector3.right) * Mathf.Deg2Rad;

        integralPitch += error * Time.fixedDeltaTime;
        float derivativePitch = (error - prevErrorPitch) / Time.fixedDeltaTime;
        float angularMomentum = Body.inertiaTensor.x * Body.angularVelocity.x;

        float pidOut = KpAngle * error + KiAngle * integralPitch + KdAngle * derivativePitch;

        prevErrorPitch = error;

        float torque = (Body.inertiaTensor.x * pidOut - angularMomentum) / 4;
        return torque;
    }

    private float getRoll(float forceX, float forceY)
    {
        float error = Vector3.SignedAngle(new Vector3(forceX, Mathf.Abs(forceY), 0), Vector3.up, Vector3.forward) * Mathf.Deg2Rad;

        integralRoll += error * Time.fixedDeltaTime;
        float derivativeRoll = (error - prevErrorRoll) / Time.fixedDeltaTime;
        float angularMomentum = Body.inertiaTensor.z * Body.angularVelocity.z;

        float pidOut = KpAngle * error + KiAngle * integralRoll + KdAngle * derivativeRoll;

        prevErrorRoll = error;

        float torque = (Body.inertiaTensor.z * pidOut - angularMomentum) / 4;
        return torque;
    }

    private float getYaw()
    {
        float error = TargetYaw - transform.rotation.eulerAngles.y;

        while (error < -180)
        {
            error += 360;
        }

        integralYaw += error * Time.fixedDeltaTime;
        float derivativeYaw = (error - prevErrorYaw) / Time.fixedDeltaTime;
        float angularMomentum = Body.inertiaTensor.y * Body.angularVelocity.y;

        float pidOut = KpYaw * error + KiYaw * integralYaw + KdYaw * derivativeYaw;
        prevErrorYaw = error;

        float torque = (Body.inertiaTensor.y * pidOut - angularMomentum) / 40;
        return torque;
    }

}
