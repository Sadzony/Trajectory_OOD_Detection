using UnityEngine;
using System.Collections.Generic;

public class LaneDefinedMovement : MonoBehaviour
{
    [SerializeField] private float targetVelocity = 10f;
    [SerializeField] private float maxAcceleration = 3f;
    [SerializeField] private float wheelbase = 2.5f;
    [SerializeField] private float maxSteeringAngle = 30f;

    [SerializeField] private float laneChangeDuration = 2.5f;
    [SerializeField] private float laneAlignmentTolerance = 0.05f;

    [SerializeField] private Transform currentCentreLine;

    private float velocity;
    private float acceleration;
    private float heading;

    private float laneChangeTime;
    private float laneChangeStartX;
    private float laneChangeTargetX;

    private enum BehaviourState
    {
        Cruise,
        LaneChange
    }

    private BehaviourState behaviourState;

    //accessor functions
    public float GetCurrentVelocity() => velocity;
    public float GetCurrentAcceleration() => acceleration;
    public float GetTargetVelocity() => targetVelocity;
    public float GetMaxAcceleration() => maxAcceleration;
    public float GetHeading() => heading;

    private void Start()
    {
        velocity = targetVelocity;
        heading = transform.eulerAngles.y * Mathf.Deg2Rad;

        behaviourState = BehaviourState.Cruise;
    }

    // ==================================================
    // UPDATE LOOP
    // ==================================================

    private void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;

        HandleVelocity(dt);

        UpdateBehaviourState();

        Debug.Log(laneChangeTime);

        if (behaviourState == BehaviourState.LaneChange)
            FollowLaneChange(dt);

        ApplyBicycle(dt);
    }

    // ==================================================
    // VELOCITY
    // ==================================================

    private void HandleVelocity(float dt)
    {
        float error = targetVelocity - velocity;

        acceleration = Mathf.Clamp(error / dt, -maxAcceleration, maxAcceleration);
        velocity += acceleration * dt;
    }

    // ==================================================
    // STATE TRANSITION
    // ==================================================

    private void UpdateBehaviourState()
    {
        float lateralError =
            currentCentreLine.position.x - transform.position.x;

        if (behaviourState == BehaviourState.Cruise &&
            Mathf.Abs(lateralError) > laneAlignmentTolerance)
        {
            behaviourState = BehaviourState.LaneChange;

            laneChangeTime = 0f;
            laneChangeStartX = transform.position.x;
            laneChangeTargetX = currentCentreLine.position.x;
        }
    }

    // ==================================================
    // LANE CHANGE (QUINTIC + CURVATURE CONSISTENT)
    // ==================================================

    private void FollowLaneChange(float dt)
    {
        laneChangeTime += dt;

        float u = laneChangeTime / laneChangeDuration;

        if (u >= 1f)
        {
            u = 1f;
            behaviourState = BehaviourState.Cruise;
        }

        // ==================================================
        // QUINTIC
        // ==================================================

        float s =
            10f * u * u * u
            - 15f * u * u * u * u
            + 6f * u * u * u * u * u;

        float ds =
            30f * u * u
            - 60f * u * u * u
            + 30f * u * u * u * u;

        float deltaX =
            laneChangeTargetX - laneChangeStartX;

        float xRef =
            laneChangeStartX + deltaX * s;

        float xDotRef =
            (deltaX * ds) / laneChangeDuration;

        // ==================================================
        // CURVATURE (STATE CONSISTENT FIX)
        // ==================================================

        float v = Mathf.Max(velocity, 0.01f);

        float lateralVelocity =
            velocity * Mathf.Sin(heading);

        float lateralVelocityError =
            xDotRef - lateralVelocity;

        float curvature =
            (2f * lateralVelocityError) / (v * v);

        // ==================================================
        // STEERING
        // ==================================================

        float steering =
            Mathf.Atan(curvature * wheelbase);

        steering = Mathf.Clamp(
            steering,
            -maxSteeringAngle * Mathf.Deg2Rad,
            maxSteeringAngle * Mathf.Deg2Rad);

        // store steering for bicycle step
        currentSteering = steering;
    }

    // ==================================================
    // BICYCLE MODEL
    // ==================================================

    private float currentSteering;

    private void ApplyBicycle(float dt)
    {
        heading +=
            (velocity / wheelbase)
            * Mathf.Tan(currentSteering)
            * dt;

        float x = transform.position.x;
        float z = transform.position.z;

        x += velocity * Mathf.Sin(heading) * dt;
        z += velocity * Mathf.Cos(heading) * dt;

        transform.position = new Vector3(x, transform.position.y, z);

        transform.rotation = Quaternion.Euler(
            0f,
            heading * Mathf.Rad2Deg,
            0f
        );
    }
}
