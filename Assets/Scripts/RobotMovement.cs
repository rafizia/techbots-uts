using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RobotMovement : MonoBehaviour
{
    // Kontrol (Manual): WASD, Rem: Spacebar, Tombol Panah Kanan Kiri untuk Rotasi
    private float horizontalInput, verticalInput;
    private float currentSteerAngle, currentbreakForce;
    private bool isBreaking;

    [SerializeField] private float motorForce, breakForce, maxSteerAngle;
    private WheelCollider frontLeftWheelCollider, frontRightWheelCollider;
    private WheelCollider rearLeftWheelCollider, rearRightWheelCollider;
    private Transform frontLeftWheelTransform, frontRightWheelTransform;
    private Transform rearLeftWheelTransform, rearRightWheelTransform;

    // Rotation
    private bool isRotating = false;
    private float rotationAngle = 90f;

    [SerializeField] private float rotationSpeed = 10f;
    [SerializeField] private List<Transform> leftWheels;
    [SerializeField] private List<Transform> rightWheels;

    private void Start()
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
        // Rotasi
        if (Input.GetKeyDown(KeyCode.RightArrow) && !isRotating)
        {
            StartCoroutine(Rotate(Vector3.up * 30f, -1)); // 30f --> sudut rotasi
        }
        if (Input.GetKeyDown(KeyCode.LeftArrow) && !isRotating)
        {
            StartCoroutine(Rotate(Vector3.down * 30f, 1));
        }
    }

    private void FixedUpdate()
    {
        GetInput();
        HandleMotor();
        HandleSteering();
        UpdateWheels();
    }

    private void GetInput() // Manual Control
    {
        // Belok
        horizontalInput = Input.GetAxis("Horizontal");

        // Maju dan Mundur
        verticalInput = Input.GetAxis("Vertical");

        // Rem
        isBreaking = Input.GetKey(KeyCode.Space);
    }

    private void HandleMotor() // Method untuk maju mundur
    {
        frontLeftWheelCollider.motorTorque = verticalInput * motorForce;
        frontRightWheelCollider.motorTorque = verticalInput * motorForce;
        currentbreakForce = isBreaking ? breakForce : 0f;
        ApplyBreaking();
    }

    private void HandleSteering() // Method untuk belok
    {
        currentSteerAngle = maxSteerAngle * horizontalInput;
        frontLeftWheelCollider.steerAngle = currentSteerAngle;
        frontRightWheelCollider.steerAngle = currentSteerAngle;
    }

    private IEnumerator Rotate(Vector3 byAngle, int direction) // Method untuk rotasi
    {
        isRotating = true;

        Quaternion from = transform.rotation;
        Quaternion to = Quaternion.Euler(transform.eulerAngles + byAngle);
        float duration = Mathf.Abs(rotationAngle / rotationSpeed);
        float elapsed = 0f;

        while (elapsed < duration)
        {
            transform.rotation = Quaternion.Slerp(from, to, elapsed / duration);

            foreach (Transform wheel in leftWheels)
            {
                wheel.Rotate(Vector3.right * 360 * Time.deltaTime * direction, Space.Self);
            }
            foreach (Transform wheel in rightWheels)
            {
                wheel.Rotate(Vector3.right * -360 * Time.deltaTime * direction, Space.Self);
            }

            elapsed += Time.deltaTime;
            yield return null;
        }

        transform.rotation = to;
        isRotating = false;
    }

    private void ApplyBreaking()
    {
        frontRightWheelCollider.brakeTorque = currentbreakForce;
        frontLeftWheelCollider.brakeTorque = currentbreakForce;
        rearLeftWheelCollider.brakeTorque = currentbreakForce;
        rearRightWheelCollider.brakeTorque = currentbreakForce;
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
}