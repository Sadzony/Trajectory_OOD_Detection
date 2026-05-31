using UnityEngine;
using System.Collections.Generic;

public class BicycleModelLaneMovement : MonoBehaviour
{
    [Header("Movement")]
    [SerializeField] private float targetVelocity = 10f;
    [SerializeField] private float maxAcceleration = 3f;

    [SerializeField] private float wheelbase = 2.5f;
    [SerializeField] private float maxCruiseSteeringAngle = 30f;

    [Header("Lane Change")]
    [SerializeField] private float laneChangeDuration = 2.5f;
    [SerializeField] private float laneAlignmentTolerance = 0.05f;

    [Header("Path")]
    [SerializeField] private Transform currentCentreLine;



    // Lane change state
    private bool changingLane;

    private float laneChangeTime;
    private float laneChangeStartX;
    private float targetX;

    // Bicycle model state
    private float velocity;
    private float acceleration;
    private float steeringAngle;
    private float heading; // radians

    private void Start()
    {
        if (currentCentreLine == null)
        {
            Debug.LogError("No centre line assigned.");
            enabled = false;
            return;
        }

        velocity = targetVelocity;
        acceleration = 0f;
        heading = transform.eulerAngles.y * Mathf.Deg2Rad;
    }

    private void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;

        HandleVelocity(dt);

        if (!changingLane && CheckLaneChange())
            StartLaneChange();

        if (!changingLane)
        {
            Cruise(dt);
        }
        else
        {
            FollowLaneChange(dt);
        }
    }

    // ----------------------------
    // VELOCITY CONTROL (stable)
    // ----------------------------
    private void HandleVelocity(float dt)
    {
        float velocityError = targetVelocity - velocity;

        if (Mathf.Abs(velocityError) < 0.001f)
        {
            acceleration = 0f;
            velocity = targetVelocity;
            return;
        }

        acceleration = Mathf.Clamp(
            velocityError / dt,
            -maxAcceleration,
            maxAcceleration);

        velocity += acceleration * dt;
    }

    // ----------------------------
    // LANE CHECK
    // ----------------------------
    private bool CheckLaneChange()
    {
        float lateralError =
            currentCentreLine.position.x - transform.position.x;

        if (Mathf.Abs(lateralError) > laneAlignmentTolerance)
        {
            return true;
        }
        else return false;
    }

    // ----------------------------
    // START LANE CHANGE
    // ----------------------------
    private void StartLaneChange()
    {
        changingLane = true;

        laneChangeTime = 0f;

        laneChangeStartX = transform.position.x;
        targetX = currentCentreLine.position.x;
    }

    // ----------------------------
    // CRUISE (straight kinematics)
    // ----------------------------
    private void Cruise(float dt)
    {
        // --- LATERAL ERROR (world-space kinematic correction) ---
        float lateralError =
            currentCentreLine.position.x - transform.position.x;

        // Convert lateral error into desired heading correction
        float maxSteeringAngle = maxCruiseSteeringAngle * Mathf.Deg2Rad;
        float desiredSteering =
            Mathf.Clamp(lateralError * wheelbase,
                -maxSteeringAngle,
                maxSteeringAngle);

        // Apply kinematic bicycle heading update (stabilised cruise)
        heading += (velocity / wheelbase)
                   * Mathf.Tan(desiredSteering)
                   * dt;

        // Integrate motion
        float x = transform.position.x;
        float z = transform.position.z;

        x += velocity * Mathf.Sin(heading) * dt;
        z += velocity * Mathf.Cos(heading) * dt;

        transform.position = new Vector3(x, transform.position.y, z);

        transform.rotation = Quaternion.Euler(
            0f,
            heading * Mathf.Rad2Deg,
            0f);
    }

    // ----------------------------
    // LANE CHANGE (QUINTIC, TIME FIXED)
    // ----------------------------
    private void FollowLaneChange(float dt)
    {
        laneChangeTime += dt;

        float u = Mathf.Clamp01(laneChangeTime / laneChangeDuration);

        // Quintic polynomial
        float s =
            10f * u * u * u
            - 15f * u * u * u * u
            + 6f * u * u * u * u * u;

        float desiredX =
            laneChangeStartX +
            (targetX - laneChangeStartX) * s;

        float ds =
            30f * u * u
            - 60f * u * u * u
            + 30f * u * u * u * u;

        float desiredHeading =
            Mathf.Atan2(
                (targetX - laneChangeStartX) * ds,
                velocity * laneChangeDuration);

        heading = desiredHeading;

        float x = transform.position.x;
        float z = transform.position.z;

        x += velocity * Mathf.Sin(heading) * dt;
        z += velocity * Mathf.Cos(heading) * dt;

        transform.position = new Vector3(x, transform.position.y, z);

        transform.rotation = Quaternion.Euler(
            0f,
            heading * Mathf.Rad2Deg,
            0f);

        // End condition
        if (u >= 1f)
        {
            u = 1f;
            changingLane = false;
        }
    }
}
