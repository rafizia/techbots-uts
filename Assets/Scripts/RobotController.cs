using System;
using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;

public class RobotController : MonoBehaviour
{
    [Header("Robot Movement")]
    private RobotMovement movement;
    private bool isAutonomousMode = true;

    [Header("LIDAR Settings")]
    [SerializeField] private float lidarMaxDistance = 5f;
    [SerializeField] private float minObstacleDistance = 1.5f;
    [SerializeField] private int lidarRayCount = 9;
    [SerializeField] private float lidarAngle = 160f;
    [SerializeField] private LayerMask obstacleLayer = -1;

    [Header("Navigation")]
    [SerializeField] private float steeringAmount = 40f;
    [SerializeField] private float heightDetectionOffset = 0.5f;

    private RaycastHit[] lidarHits;
    private Vector3[] lidarDirections;
    private bool[] unsafeDirections; // Array untuk menandai arah yang tidak aman untuk dibelok
    private bool isTurning = false;
    private float turnTimer = 0f;
    private float turnDuration = 0.5f;

    private bool isReversing = false; // Flag untuk mendeteksi apakah sedang mundur
    private float reverseTimer = 0f; // Timer untuk mundur
    private float reverseDuration = 4f; // Durasi mundur dalam detik

    private Vector3 lastValidPosition; // Posisi terakhir yang valid
    private Quaternion lastValidRotation; // Rotasi terakhir yang valid

    public Transform Waypoint;
    [SerializeField] private float arrivalThreshold = 1f; // seberapa dekat ke target dianggap sampai

    private bool isMovingToTarget = false;

    private bool reverseDirection = false;

    [SerializeField] private LayerMask bombLayer;
    private Transform bombTargetTransform;
    private Transform detectedBombTarget;
    [SerializeField] private Transform cetiranyabomb;
    private Boolean toBomb = false;
    private BombSeekerRobot bombSeekerRobot;

    public Transform currentWaypoint;


    private void Start()
    {
        bombSeekerRobot = GetComponent<BombSeekerRobot>();
        movement = GetComponent<RobotMovement>();

        if (movement == null)
        {
            UnityEngine.Debug.LogError("RobotMovement component not found!");
            return;
        }

        // Initialize LIDAR rays
        InitializeLidar();

        // Set robot to autonomous mode
        movement.SetAutonomousMode(isAutonomousMode);

        // Move forward initially
        movement.MoveForward();

        // Simpan posisi dan rotasi awal sebagai posisi valid
        lastValidPosition = transform.position;
        lastValidRotation = transform.rotation;
    }

    void Update()
    {
        
    }

    IEnumerator DelayAction()
    {
        yield return new WaitForSeconds(0f);
        // Aksi setelah 0.2 detik
        Debug.Log("Sudah lewat 0.2 detik");
        Navigate();
        if (!bombSeekerRobot.isBombFound)
        {
            Waypoint = currentWaypoint;
            Navigate();
        }
    }

    public void DetectBombAndNavigate()
    {
        for (int i = 0; i < lidarRayCount; i++)
        {
            float startAngle = -lidarAngle / 2;
            float angleStep = lidarAngle / (lidarRayCount - 1);
            float currentAngle = startAngle + (angleStep * i);

            float currentMinObstacleDistance = (Mathf.Abs(currentAngle) <= 7.5f) ? 1.5f : 1f;

            Vector3 rayStart = transform.position + Vector3.up * heightDetectionOffset;
            Vector3 rayDirection = transform.TransformDirection(lidarDirections[i]);

            if (Physics.Raycast(rayStart, rayDirection, out RaycastHit hit, lidarMaxDistance, bombLayer))
            {
                lidarHits[i] = hit;
                Debug.Log("BOMB DETECTED");
                UnityEngine.Debug.DrawRay(rayStart, rayDirection * hit.distance, Color.red);

                if (detectedBombTarget == null)
                {
                    StartCoroutine(DelayAction());
                    detectedBombTarget = hit.transform;
                    Debug.Log(detectedBombTarget);
                    bombSeekerRobot.isBombFound = true;
                    minObstacleDistance = 1;
                    currentWaypoint = Waypoint;
                    Waypoint = detectedBombTarget;

                    
                    

                }
            }
            else
            {
                RaycastHit emptyHit = new RaycastHit();
                emptyHit.distance = float.PositiveInfinity;
                lidarHits[i] = emptyHit;
                UnityEngine.Debug.DrawRay(rayStart, rayDirection * lidarMaxDistance, Color.green);
                detectedBombTarget = null;
            }
        }
    }

    private void InitializeLidar()
    {
        lidarDirections = new Vector3[lidarRayCount];
        lidarHits = new RaycastHit[lidarRayCount];
        unsafeDirections = new bool[lidarRayCount]; // Inisialisasi array untuk menandai arah yang tidak aman

        // Calculate directions for each ray
        float startAngle = -lidarAngle / 2;
        float angleStep = lidarAngle / (lidarRayCount - 1);

        for (int i = 0; i < lidarRayCount; i++)
        {
            float currentAngle = startAngle + (angleStep * i);
            lidarDirections[i] = Quaternion.Euler(0, currentAngle, 0) * Vector3.forward;
            unsafeDirections[i] = false; // Awalnya semua arah aman
        }
    }

    public void Navigate()
    {
        // For turning duration tracking
        if (isTurning)
        {
            turnTimer += Time.deltaTime;
            if (turnTimer >= turnDuration)
            {
                isTurning = false;
                turnTimer = 0f;
            }
        }

        if (isAutonomousMode)
        {
            // Scan surroundings with LIDAR
            ScanWithLidar();

            Navigate(Waypoint);

            // Update wheel visuals
            UpdateVisuals();

            // Simpan posisi dan rotasi terakhir yang valid
            lastValidPosition = transform.position;
            lastValidRotation = transform.rotation;
        }  
    }

    private void UpdateVisuals()
    {
        // This ensures wheels visually update when in autonomous mode
        Quaternion rot;
        Vector3 pos;

        // We're just visualizing wheel rotations, not affecting physics
        Transform[] wheels = new Transform[4];
        WheelCollider[] colliders = new WheelCollider[4];

        if (transform.Find("WheelCollider"))
        {
            colliders[0] = transform.Find("WheelCollider").Find("FrontLeftCollider").GetComponent<WheelCollider>();
            colliders[1] = transform.Find("WheelCollider").Find("FrontRightCollider").GetComponent<WheelCollider>();
            colliders[2] = transform.Find("WheelCollider").Find("BackLeftCollider").GetComponent<WheelCollider>();
            colliders[3] = transform.Find("WheelCollider").Find("BackRightCollider").GetComponent<WheelCollider>();

            wheels[0] = transform.Find("tire_FL_Disctr");
            wheels[1] = transform.Find("tire_FR_disctr");
            wheels[2] = transform.Find("Tire_BL_Disctr");
            wheels[3] = transform.Find("tire_BR_Disctr");

            for (int i = 0; i < 4; i++)
            {
                if (wheels[i] != null && colliders[i] != null)
                {
                    colliders[i].GetWorldPose(out pos, out rot);
                    wheels[i].position = pos;
                    wheels[i].rotation = rot;
                }
            }
        }
    }

    private void ScanWithLidar()
    {
        for (int i = 0; i < lidarRayCount; i++)
        {
            float startAngle = -lidarAngle / 2;
            float angleStep = lidarAngle / (lidarRayCount - 1);
            float currentAngle = startAngle + (angleStep * i);

            float currentMinObstacleDistance = (Mathf.Abs(currentAngle) <= 7.5f) ? 1.5f : 1f;

            Vector3 rayStart = transform.position + Vector3.up * heightDetectionOffset;
            Vector3 rayDirection = transform.TransformDirection(lidarDirections[i]);

            if (Physics.Raycast(rayStart, rayDirection, out RaycastHit hit, lidarMaxDistance, obstacleLayer))
            {
                lidarHits[i] = hit;

                // Tandai arah yang tidak aman jika jarak terlalu dekat dengan obstacle
                unsafeDirections[i] = hit.distance <= currentMinObstacleDistance;
                
                Color rayColor = unsafeDirections[i] ? Color.red : Color.green;
                UnityEngine.Debug.DrawRay(rayStart, rayDirection * hit.distance, rayColor);
            }
            else
            {
                RaycastHit emptyHit = new RaycastHit();
                emptyHit.distance = lidarMaxDistance + 1;
                lidarHits[i] = emptyHit;
                unsafeDirections[i] = false; // Arah aman
                UnityEngine.Debug.DrawRay(rayStart, rayDirection * lidarMaxDistance, Color.green);
            }
        }
    }

    private void Navigate(Transform targetWaypoint)
    {
        Vector3 directionToTarget = (targetWaypoint.position - transform.position);
        directionToTarget.y = 0;

        float distance = directionToTarget.magnitude;

        if (distance < arrivalThreshold)
        {
            isMovingToTarget = false;
            movement.StopMovement();
            Debug.Log("Robot has reached the target!");
            Waypoint = currentWaypoint;
            StartCoroutine(movement.ApplyBreaking());
            if (bombSeekerRobot.isBombFound)
            {
                Debug.Log("mndur woy");
                
                transform.Rotate(0f, 180f, 0f);


            }
            bombSeekerRobot.isBombFound = false;
            minObstacleDistance = 2.5f;
        }
        Debug.Log("lagi navigasi");
        if (isReversing)
        {
            Debug.Log("lagi reverse");
            reverseTimer += Time.deltaTime;
            if (reverseTimer >= reverseDuration)
            {
                isReversing = false;
                reverseTimer = 0f;
                movement.StopMovement();
            }
            else
            {
                var steerAngle = movement.frontLeftWheelCollider.steerAngle;
                if (steerAngle > 0 && !reverseDirection)
                {
                    movement.TurnLeft(Mathf.Abs(steeringAmount));
                    reverseDirection = true;
                }
                else if (steerAngle < 0 && !reverseDirection)
                {
                    movement.TurnRight(Mathf.Abs(steeringAmount));
                    reverseDirection = true;
                }
                movement.TurnLeft(0);
                movement.MoveBackward();
            }
            return;
        }

        if (isTurning)
        {
            Debug.Log("lagi turning");
            return;
        }

        foreach (var hit in lidarHits)
        {
            if (hit.distance > 0 && hit.distance < 0.5f)
            {
                Debug.Log("Too close to obstacle, reversing...");
                isReversing = true;
                reverseTimer = 0f;
                return;
            }
        }

        int centerRayIndex = lidarRayCount / 2;

        bool obstacleAhead = false;
        int middleRayStart = centerRayIndex - 1;
        int middleRayEnd = centerRayIndex + 1;

        for (int i = middleRayStart; i <= middleRayEnd; i++)
        {
            if (i >= 0 && i < lidarRayCount && lidarHits[i].distance <= minObstacleDistance)
            {
                obstacleAhead = true;
                UnityEngine.Debug.Log($"Obstacle detected ahead at distance: {lidarHits[i].distance}");
                break;
            }
        }

        

        if (obstacleAhead)
        {
            Debug.Log("OAHYO");
            float leftDistance = CalculateAverageDistance(0, centerRayIndex - 2);
            float rightDistance = CalculateAverageDistance(centerRayIndex + 2, lidarRayCount - 1);

            UnityEngine.Debug.Log($"Obstacle ahead! Left space: {leftDistance}, Right space: {rightDistance}");

            movement.StopMovement();

            // Periksa apakah terdapat hambatan di kiri atau kanan berdasarkan flag unsafeDirections
            bool leftUnsafe = IsDirectionUnsafe(0, centerRayIndex - 2);
            bool rightUnsafe = IsDirectionUnsafe(centerRayIndex + 2, lidarRayCount - 1);

            if ((leftDistance < minObstacleDistance && rightDistance < minObstacleDistance) || 
                (leftUnsafe && rightUnsafe))
            {
                Debug.Log("No space to turn, reversing...");
                isReversing = true;
                reverseTimer = 0f;
                return;
            }

            isTurning = true;
            turnTimer = 0f;

            // Prioritaskan belok ke arah yang aman
            if (leftUnsafe && !rightUnsafe)
            {
                UnityEngine.Debug.Log("Left is unsafe, turning RIGHT");
                movement.TurnRight(steeringAmount);
            }
            else if (!leftUnsafe && rightUnsafe)
            {
                UnityEngine.Debug.Log("Right is unsafe, turning LEFT");
                movement.TurnLeft(steeringAmount);
            }
            else if (leftDistance > rightDistance)
            {
                UnityEngine.Debug.Log("Turning LEFT");
                movement.TurnLeft(steeringAmount);
            }
            else
            {
                UnityEngine.Debug.Log("Turning RIGHT");
                movement.TurnRight(steeringAmount);
            }

            movement.MoveForward();
        }
        else
        {
            Debug.Log("DSDSDS");
            directionToTarget = (targetWaypoint.position - transform.position);
            directionToTarget.y = 0;

            distance = directionToTarget.magnitude;
            Vector3 dirNormalized = directionToTarget.normalized;

            float angle = Vector3.SignedAngle(transform.forward, dirNormalized, Vector3.up);

            if (Mathf.Abs(angle) > 5f)
            {
                // Cek apakah arah yang ingin dibelok aman
                if (angle > 0 && !IsRightDirectionUnsafe())
                    movement.TurnRight(Mathf.Clamp(Mathf.Abs(angle), 10f, steeringAmount));
                else if (angle < 0 && !IsLeftDirectionUnsafe())
                    movement.TurnLeft(Mathf.Clamp(Mathf.Abs(angle), 10f, steeringAmount));
                else if (angle > 0 && IsRightDirectionUnsafe() && !IsLeftDirectionUnsafe())
                    movement.TurnLeft(Mathf.Clamp(Mathf.Abs(angle), 10f, steeringAmount)); // Belok kiri jika kanan tidak aman
                else if (angle < 0 && IsLeftDirectionUnsafe() && !IsRightDirectionUnsafe())
                    movement.TurnRight(Mathf.Clamp(Mathf.Abs(angle), 10f, steeringAmount)); // Belok kanan jika kiri tidak aman
                else
                    movement.TurnLeft(0); // Tidak belok jika kedua arah tidak aman
            }
            else
            {
                movement.TurnLeft(0);
            }

            movement.MoveForward();
        }

        HandleElevationChanges();
    }

    private float CalculateAverageDistance(int startIdx, int endIdx)
    {
        if (startIdx < 0) startIdx = 0;
        if (endIdx >= lidarRayCount) endIdx = lidarRayCount - 1;

        float totalDistance = 0;
        int count = 0;

        for (int i = startIdx; i <= endIdx; i++)
        {
            totalDistance += lidarHits[i].distance;
            count++;
        }

        return (count > 0) ? totalDistance / count : 0;
    }

    private void HandleElevationChanges()
    {
        Vector3 downRayStart = transform.position + Vector3.up * 0.5f + transform.forward * 1.0f;

        if (Physics.Raycast(downRayStart, Vector3.down, out RaycastHit hit, 2f))
        {
            float groundDistance = hit.distance;

            if (groundDistance > 1.2f)
            {
                movement.StopMovement();
                StartCoroutine(SlowApproach());
            }
            else if (groundDistance < 0.3f)
            {
                movement.MoveForward();
            }

            UnityEngine.Debug.DrawRay(downRayStart, Vector3.down * groundDistance, Color.blue);
        }
    }

    private IEnumerator SlowApproach()
    {
        float originalSpeed = 0.5f;
        float currentSpeed = originalSpeed;

        while (currentSpeed > 0)
        {
            currentSpeed -= 0.1f;
            if (currentSpeed < 0) currentSpeed = 0;

            yield return new WaitForSeconds(0.1f);
        }
    }

    private void OnDrawGizmos()
    {
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireSphere(transform.position, minObstacleDistance);
    }

    // Fungsi untuk memeriksa apakah suatu rentang arah tidak aman
    private bool IsDirectionUnsafe(int startIdx, int endIdx)
    {
        if (startIdx < 0) startIdx = 0;
        if (endIdx >= lidarRayCount) endIdx = lidarRayCount - 1;

        for (int i = startIdx; i <= endIdx; i++)
        {
            if (unsafeDirections[i])
                return true;
        }
        return false;
    }

    // Fungsi untuk memeriksa apakah arah kiri tidak aman
    private bool IsLeftDirectionUnsafe()
    {
        int centerRayIndex = lidarRayCount / 2;
        return IsDirectionUnsafe(0, centerRayIndex - 2);
    }

    // Fungsi untuk memeriksa apakah arah kanan tidak aman
    private bool IsRightDirectionUnsafe()
    {
        int centerRayIndex = lidarRayCount / 2;
        return IsDirectionUnsafe(centerRayIndex + 2, lidarRayCount - 1);
    }
}