using System;
using System.Collections;
using System.Collections.Generic;
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
    private bool isTurning = false;
    private float turnTimer = 0f;
    private float turnDuration = 0.5f;

    private bool isReversing = false; // Flag untuk mendeteksi apakah sedang mundur
    private float reverseTimer = 0f; // Timer untuk mundur
    private float reverseDuration = 3f; // Durasi mundur dalam detik

    private Vector3 lastValidPosition; // Posisi terakhir yang valid
    private Quaternion lastValidRotation; // Rotasi terakhir yang valid

    public Transform Waypoint;
    [SerializeField] private float arrivalThreshold = 1f; // seberapa dekat ke target dianggap sampai

    private bool isMovingToTarget = false;

    [SerializeField] private float sideMinObstacleDistance = 0.5f; // Jarak minimal aman samping
    
    // Variabel untuk mendeteksi dan menangani situasi stuck
    private bool isStuck = false;
    private float stuckTimer = 0f;
    private float stuckTimeThreshold = 1.0f; // Lebih cepat mendeteksi stuck
    private float stuckRotationAmount = 120f; // Sudut rotasi lebih besar untuk keluar dari situasi stuck
    private bool isStuckTurning = false;
    private float stuckTurnTimer = 0f;
    private float stuckTurnDuration = 1.5f;

    // Obstacle memory untuk menghindari area yang pernah menyebabkan stuck
    private List<Vector3> problemAreas = new List<Vector3>();
    private float problemAreaRadius = 2.0f; // Radius area yang dihindari
    private float minDistanceToProblemArea = 2.5f; // Jarak minimal ke area masalah

    // Force stop flag - untuk mencegah robot bergerak sama sekali jika tabrakan tak terelakkan
    private bool forceStop = false;
    private float forceStopTimer = 0f;
    private float forceStopDuration = 2.0f;

    private void Start()
    {
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

        // Stop movement initially and wait for navigation decisions
        movement.StopMovement();

        // Simpan posisi dan rotasi awal sebagai posisi valid
        lastValidPosition = transform.position;
        lastValidRotation = transform.rotation;
        
        // Inisialisasi list problem areas
        problemAreas = new List<Vector3>();
    }

    private void InitializeLidar()
    {
        lidarDirections = new Vector3[lidarRayCount];
        lidarHits = new RaycastHit[lidarRayCount];

        // Calculate directions for each ray
        float startAngle = -lidarAngle / 2;
        float angleStep = lidarAngle / (lidarRayCount - 1);

        for (int i = 0; i < lidarRayCount; i++)
        {
            float currentAngle = startAngle + (angleStep * i);
            lidarDirections[i] = Quaternion.Euler(0, currentAngle, 0) * Vector3.forward;
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

        // Untuk deteksi stuck turning
        if (isStuckTurning)
        {
            stuckTurnTimer += Time.deltaTime;
            if (stuckTurnTimer >= stuckTurnDuration)
            {
                isStuckTurning = false;
                stuckTurnTimer = 0f;
            }
        }

        // Force stop timer
        if (forceStop)
        {
            forceStopTimer += Time.deltaTime;
            if (forceStopTimer >= forceStopDuration)
            {
                forceStop = false;
                forceStopTimer = 0f;
            }
        }

        if (isAutonomousMode && !forceStop)
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
        else if (forceStop)
        {
            // Force robot to completely stop if force stop is active
            movement.StopMovement();
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

                UnityEngine.Debug.DrawRay(rayStart, rayDirection * hit.distance, Color.red);
            }
            else
            {
                RaycastHit emptyHit = new RaycastHit();
                emptyHit.distance = float.PositiveInfinity;
                lidarHits[i] = emptyHit;
                UnityEngine.Debug.DrawRay(rayStart, rayDirection * lidarMaxDistance, Color.green);
            }
        }
    }

    private void Navigate(Transform targetWaypoint)
    {
        // Jika kita dipaksa berhenti, jangan navigasi sama sekali
        if (forceStop)
        {
            movement.StopMovement();
            return;
        }

        if (isReversing)
        {
            reverseTimer += Time.deltaTime;
            if (reverseTimer >= reverseDuration)
            {
                isReversing = false;
                reverseTimer = 0f;
                
                // Setelah mundur, coba belok untuk keluar dari situasi
                if (isStuck)
                {
                    // Catat lokasi ini sebagai area bermasalah
                    Vector3 problemLocation = transform.position;
                    if (!IsNearExistingProblemArea(problemLocation))
                    {
                        problemAreas.Add(problemLocation);
                        UnityEngine.Debug.Log($"Added problem area at {problemLocation}. Total: {problemAreas.Count}");
                    }
                
                    UnityEngine.Debug.Log("After reversing, attempting to rotate to escape stuck situation");
                    isStuckTurning = true;
                    stuckTurnTimer = 0f;

                    // Belok ke arah yang umumnya lebih luas
                    float leftDistance = CalculateAverageDistance(0, lidarRayCount / 2 - 1);
                    float rightDistance = CalculateAverageDistance(lidarRayCount / 2 + 1, lidarRayCount - 1);
                    
                    if (leftDistance > rightDistance)
                    {
                        movement.TurnLeft(stuckRotationAmount);
                    }
                    else
                    {
                        movement.TurnRight(stuckRotationAmount);
                    }

                    // Reset stuck status setelah mencoba keluar
                    isStuck = false;
                    stuckTimer = 0f;
                }
                else
                {
                    movement.StopMovement();
                }
            }
            else
            {
                movement.TurnLeft(0);
                movement.MoveBackward();
            }
            return;
        }

        if (isTurning || isStuckTurning)
        {
            return;
        }

        int centerRayIndex = lidarRayCount / 2;
        int middleRayStart = centerRayIndex - 1;
        int middleRayEnd = centerRayIndex + 1;

        // Cek jika ada zona masalah di sekitar yang harus dihindari
        bool nearProblemArea = IsNearAnyProblemArea(transform.position);
        if (nearProblemArea)
        {
            // Jika dekat dengan area bermasalah, cari jalan alternatif
            UnityEngine.Debug.Log("Near a known problem area! Finding alternative path...");
            
            // Cari arah yang paling jauh dari area masalah terdekat
            Vector3 avoidDirection = GetAvoidDirectionFromNearestProblemArea();
            
            // Belok ke arah yang berlawanan dengan area masalah
            float angle = Vector3.SignedAngle(transform.forward, avoidDirection, Vector3.up);
            
            if (angle > 0)
            {
                movement.TurnRight(Mathf.Min(Mathf.Abs(angle), steeringAmount * 1.5f));
            }
            else
            {
                movement.TurnLeft(Mathf.Min(Mathf.Abs(angle), steeringAmount * 1.5f));
            }
            
            // Bergerak perlahan saat menghindari area masalah
            movement.MoveForward();
            return;
        }

        // DETEKSI TABRAKAN IMMINENT - Objek sangat dekat dari arah manapun
        bool aboutToCrash = false;
        foreach (var hit in lidarHits)
        {
            if (hit.distance > 0 && hit.distance < 0.3f) // Threshold lebih kecil untuk deteksi tabrakan imminent
            {
                aboutToCrash = true;
                break;
            }
        }

        if (aboutToCrash)
        {
            // PAKSA BERHENTI TOTAL JIKA HAMPIR NABRAK
            UnityEngine.Debug.LogWarning("IMMINENT COLLISION DETECTED! Force stopping all movement!");
            forceStop = true;
            forceStopTimer = 0f;
            movement.StopMovement();
            return;
        }

        // Deteksi obstacle di sisi kiri dan kanan
        bool obstacleLeft = false;
        bool obstacleRight = false;
        int sideRayCount = Mathf.Max(2, lidarRayCount / 3); // Lebih banyak ray untuk samping
        
        // Cek sisi kiri (ray paling kiri)
        for (int i = 0; i < sideRayCount; i++) {
            if (i < lidarHits.Length && lidarHits[i].distance < sideMinObstacleDistance) {
                obstacleLeft = true;
                break;
            }
        }
        // Cek sisi kanan (ray paling kanan)
        for (int i = lidarRayCount - sideRayCount; i < lidarRayCount; i++) {
            if (i < lidarHits.Length && lidarHits[i].distance < sideMinObstacleDistance) {
                obstacleRight = true;
                break;
            }
        }

        // Deteksi obstacle sangat dekat di depan (khusus ray tengah)
        bool obstacleVeryCloseFront = false;
        for (int i = middleRayStart; i <= middleRayEnd; i++) {
            if (i >= 0 && i < lidarRayCount && lidarHits[i].distance > 0 && lidarHits[i].distance < 0.5f) {
                obstacleVeryCloseFront = true;
                break;
            }
        }

        // Jika obstacle sangat dekat di depan, reverse
        if (obstacleVeryCloseFront) {
            UnityEngine.Debug.Log("Too close to obstacle in front, reversing...");
            isReversing = true;
            reverseTimer = 0f;
            movement.StopMovement();
            return;
        }

        // Cek jika robot stuck (terhalang dari banyak arah)
        bool potentiallyStuck = obstacleLeft && obstacleRight; // Jika kiri dan kanan ada obstacle dekat
        
        if (potentiallyStuck)
        {
            // Increment stuck timer jika terjebak
            stuckTimer += Time.deltaTime;
            
            // Jika timer melebihi threshold, anggap benar-benar stuck
            if (stuckTimer >= stuckTimeThreshold && !isStuck)
            {
                isStuck = true;
                UnityEngine.Debug.Log("Robot is STUCK! Initiating escape maneuver");
                
                // Catat lokasi ini sebagai area bermasalah
                problemAreas.Add(transform.position);
                
                // Mundur untuk keluar dari situasi stuck
                isReversing = true;
                reverseTimer = 0f;
                movement.StopMovement();
                return;
            }
        }
        else
        {
            // Reset stuck timer jika tidak potensial stuck
            stuckTimer = 0f;
            isStuck = false;
        }

        // Jika ada obstacle sangat dekat di kiri/kanan, JANGAN MAJU/BLOK, tapi JANGAN reverse
        if (obstacleLeft || obstacleRight) {
            UnityEngine.Debug.Log("Obstacle detected on the side! Robot adjusts direction");
            
            // Jika hanya satu sisi yang terhalang, coba belok ke sisi lain dengan lebih agresif
            if (obstacleLeft && !obstacleRight) {
                // Belok kanan karena kiri terhalang, dengan sudut lebih besar
                movement.TurnRight(steeringAmount * 1.5f);
                movement.MoveForward();
                return;
            }
            else if (!obstacleLeft && obstacleRight) {
                // Belok kiri karena kanan terhalang, dengan sudut lebih besar
                movement.TurnLeft(steeringAmount * 1.5f);
                movement.MoveForward();
                return;
            }
            else {
                // Kedua sisi terhalang, BERHENTI TOTAL untuk mencegah tabrakan
                movement.StopMovement();
                // Pencegah reverse terlalu cepat:
                if (stuckTimer < stuckTimeThreshold * 0.5f) {
                    return; // Keluar sebelum deteksi obstacle di depan
                }
            }
        }

        bool obstacleAhead = false;
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
            float leftDistance = CalculateAverageDistance(0, centerRayIndex - 2);
            float rightDistance = CalculateAverageDistance(centerRayIndex + 2, lidarRayCount - 1);

            UnityEngine.Debug.Log($"Obstacle ahead! Left space: {leftDistance}, Right space: {rightDistance}");

            movement.StopMovement();

            // Jika kiri dan kanan juga sempit, mundur untuk keluar dari situasi
            if (leftDistance < minObstacleDistance && rightDistance < minObstacleDistance)
            {
                UnityEngine.Debug.Log("No space to turn or move, initiating reverse to escape!");
                
                // Catat lokasi ini sebagai area bermasalah
                problemAreas.Add(transform.position);
                
                isReversing = true;
                reverseTimer = 0f;
                return;
            }

            // Jika salah satu sisi lebih longgar, belok ke arah yang lebih aman dengan lebih agresif
            isTurning = true;
            turnTimer = 0f;

            if (leftDistance > rightDistance)
            {
                UnityEngine.Debug.Log("Turning LEFT");
                movement.TurnLeft(steeringAmount * 1.2f); // Belok lebih agresif
            }
            else
            {
                UnityEngine.Debug.Log("Turning RIGHT");
                movement.TurnRight(steeringAmount * 1.2f); // Belok lebih agresif
            }

            movement.MoveForward();
        }
        else
        {
            // Jalur aman, navigasi ke waypoint seperti biasa
            Vector3 directionToTarget = (targetWaypoint.position - transform.position);
            directionToTarget.y = 0;

            float distance = directionToTarget.magnitude;
            Vector3 dirNormalized = directionToTarget.normalized;

            float angle = Vector3.SignedAngle(transform.forward, dirNormalized, Vector3.up);

            if (Mathf.Abs(angle) > 5f)
            {
                if (angle > 0)
                    movement.TurnRight(Mathf.Clamp(Mathf.Abs(angle), 10f, steeringAmount));
                else
                    movement.TurnLeft(Mathf.Clamp(Mathf.Abs(angle), 10f, steeringAmount));
            }
            else
            {
                movement.TurnLeft(0);
            }

            movement.MoveForward();

            if (distance < arrivalThreshold)
            {
                isMovingToTarget = false;
                movement.StopMovement();
                Debug.Log("Robot has reached the target!");
            }
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

    // Cek apakah posisi saat ini dekat dengan area bermasalah yang sudah diketahui
    private bool IsNearAnyProblemArea(Vector3 position)
    {
        foreach (var area in problemAreas)
        {
            if (Vector3.Distance(position, area) < minDistanceToProblemArea)
            {
                return true;
            }
        }
        return false;
    }

    // Cek apakah lokasi baru dekat dengan area bermasalah yang sudah ada
    private bool IsNearExistingProblemArea(Vector3 position)
    {
        foreach (var area in problemAreas)
        {
            if (Vector3.Distance(position, area) < problemAreaRadius)
            {
                return true;
            }
        }
        return false;
    }

    // Dapatkan arah untuk menghindari area masalah terdekat
    private Vector3 GetAvoidDirectionFromNearestProblemArea()
    {
        Vector3 nearestProblemArea = transform.position;
        float minDistance = float.MaxValue;
        
        foreach (var area in problemAreas)
        {
            float distance = Vector3.Distance(transform.position, area);
            if (distance < minDistance)
            {
                minDistance = distance;
                nearestProblemArea = area;
            }
        }
        
        // Arah berlawanan dari area masalah
        Vector3 avoidDirection = (transform.position - nearestProblemArea).normalized;
        return avoidDirection;
    }
}