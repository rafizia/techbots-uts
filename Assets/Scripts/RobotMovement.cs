using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RobotMovement : MonoBehaviour
{
    private float horizontalInput, verticalInput;
    private float currentSteerAngle, currentbreakForce;
    private bool isBreaking;

    [SerializeField] public float motorForce, breakForce, maxSteerAngle;
    public WheelCollider frontLeftWheelCollider, frontRightWheelCollider;
    private WheelCollider rearLeftWheelCollider, rearRightWheelCollider;
    private Transform frontLeftWheelTransform, frontRightWheelTransform;
    private Transform rearLeftWheelTransform, rearRightWheelTransform;

    private bool isAutonomousMode = false;

    private void Awake()
    {
        frontLeftWheelCollider = transform.Find("WheelCollider").Find("FrontLeftCollider").GetComponent<WheelCollider>();
        frontRightWheelCollider = transform.Find("WheelCollider").Find("FrontRightCollider").GetComponent<WheelCollider>();
        rearLeftWheelCollider = transform.Find("WheelCollider").Find("BackLeftCollider").GetComponent<WheelCollider>();
        rearRightWheelCollider = transform.Find("WheelCollider").Find("BackRightCollider").GetComponent<WheelCollider>();

        frontLeftWheelTransform = transform.Find("tire_FL_Disctr");
        frontRightWheelTransform = transform.Find("tire_FR_disctr");
        rearLeftWheelTransform = transform.Find("Tire_BL_Disctr");
        rearRightWheelTransform = transform.Find("tire_BR_Disctr");
    }

    private void Update()
    {
        if (!isAutonomousMode)
        {
            GetInput();
        }
    }

    private void FixedUpdate()
    {
        if (!isAutonomousMode)
        {
            HandleMotor();
            HandleSteering();
            UpdateWheels();
        }
    }

    private void GetInput()
    {
        horizontalInput = Input.GetAxis("Horizontal");
        verticalInput = Input.GetAxis("Vertical");
        isBreaking = Input.GetKey(KeyCode.Space);
    }

    private void HandleMotor()
    {
        frontLeftWheelCollider.motorTorque = verticalInput * motorForce;
        frontRightWheelCollider.motorTorque = verticalInput * motorForce;
        currentbreakForce = isBreaking ? breakForce : 0f;
        ApplyBreaking();
    }

    private void HandleSteering()
    {
        currentSteerAngle = maxSteerAngle * horizontalInput;
        frontLeftWheelCollider.steerAngle = currentSteerAngle;
        frontRightWheelCollider.steerAngle = currentSteerAngle;
    }

    public IEnumerator ApplyBreaking()
    {
        // Aktifkan rem
        frontRightWheelCollider.brakeTorque = breakForce;
        frontLeftWheelCollider.brakeTorque = breakForce;
        rearLeftWheelCollider.brakeTorque = breakForce;
        rearRightWheelCollider.brakeTorque = breakForce;

        yield return new WaitForSeconds(1); // Delay selama 0.2 detik

        // Lepaskan rem
        frontRightWheelCollider.brakeTorque = 0;
        frontLeftWheelCollider.brakeTorque = 0;
        rearLeftWheelCollider.brakeTorque = 0;
        rearRightWheelCollider.brakeTorque = 0;
    }

    private void UpdateWheels()
    {
        UpdateSingleWheel(frontLeftWheelCollider, frontLeftWheelTransform);
        UpdateSingleWheel(frontRightWheelCollider, frontRightWheelTransform);
        UpdateSingleWheel(rearRightWheelCollider, rearRightWheelTransform);
        UpdateSingleWheel(rearLeftWheelCollider, rearLeftWheelTransform);
    }

    private void UpdateSingleWheel(WheelCollider wheelCollider, Transform wheelTransform)
    {
        Vector3 pos;
        Quaternion rot;
        wheelCollider.GetWorldPose(out pos, out rot);
        wheelTransform.rotation = rot;
        wheelTransform.position = pos;
    }

    // ---- FUNCTION UNTUK AUTONOMOUS ----
    public void MoveForward()
    {
        frontLeftWheelCollider.motorTorque = motorForce;
        frontRightWheelCollider.motorTorque = motorForce;
    }

    public void MoveBackward()
    {
        frontLeftWheelCollider.motorTorque = -motorForce;
        frontRightWheelCollider.motorTorque = -motorForce;
    }

    public void TurnLeft(float amount)
    {
        frontLeftWheelCollider.steerAngle = -amount;
        frontRightWheelCollider.steerAngle = -amount;
    }

    public void TurnRight(float amount)
    {
        frontLeftWheelCollider.steerAngle = amount;
        frontRightWheelCollider.steerAngle = amount;
    }

    public void StopMovement()
    {
        frontLeftWheelCollider.motorTorque = 0f;
        frontRightWheelCollider.motorTorque = 0f;
    }

    public void SetAutonomousMode(bool isAuto)
    {
        isAutonomousMode = isAuto;
    }
}
