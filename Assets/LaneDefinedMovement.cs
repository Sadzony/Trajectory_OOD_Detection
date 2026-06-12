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

    private void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;

        HandleVelocity(dt);

        UpdateBehaviourState();

        float steering = ComputeSteering(dt);

        ApplyBicycleModel(dt, steering);
    }

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

    private void UpdateBehaviourState()
    {
        float lateralError =
            currentCentreLine.position.x - transform.position.x;

        switch (behaviourState)
        {
            case BehaviourState.Cruise:

                if (Mathf.Abs(lateralError) > laneAlignmentTolerance)
                {
                    StartLaneChange();
                }

                break;

            case BehaviourState.LaneChange:

                // optional: allow external re-triggering if lane target changes
                break;
        }
    }
    private void StartLaneChange()
    {
        behaviourState = BehaviourState.LaneChange;

        laneChangeTime = 0f;

        laneChangeStartX =
            transform.position.x;

        laneChangeTargetX =
            currentCentreLine.position.x;
    }

    private float ComputeSteering(float dt)
    {
        float x = transform.position.x;
        float v = Mathf.Max(velocity, 0.01f);

        if (behaviourState == BehaviourState.LaneChange)
        {
            laneChangeTime += dt;

            float u = laneChangeTime / laneChangeDuration;

            if (u >= 1f)
            {
                u = 1f;
                behaviourState = BehaviourState.Cruise;
            }

            float s =
                10f * u * u * u
                - 15f * u * u * u * u
                + 6f * u * u * u * u * u;

            float ds =
                30f * u * u
                - 60f * u * u * u
                + 30f * u * u * u * u;

            float dds =
                60f * u
                - 180f * u * u
                + 120f * u * u * u;

            float deltaX = laneChangeTargetX - laneChangeStartX;

            float xDDotRef =
                (deltaX * dds) /
                (laneChangeDuration * laneChangeDuration);

            float curvature =
                xDDotRef / (v * v);

            return Mathf.Atan(curvature * wheelbase);
        }
        else if (behaviourState == BehaviourState.Cruise)
        {
            float lateralError =
                currentCentreLine.position.x - x;

            return Mathf.Clamp(
                lateralError * 0.5f,
                -maxSteeringAngle * Mathf.Deg2Rad,
                maxSteeringAngle * Mathf.Deg2Rad);
        }
        else return 0;
    }
    private void ApplyBicycleModel(float dt, float steering)
    {
        heading +=
            (velocity / wheelbase)
            * Mathf.Tan(steering)
            * dt;

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
}
