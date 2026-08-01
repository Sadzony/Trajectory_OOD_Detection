using UnityEngine;
using System.Collections.Generic;

public class LossOfControlBehaviour : MonoBehaviour
{
    [SerializeField] public LaneDefinedMovementQuintic vehicleController;
    [SerializeField] public Transform currentCentreLine;
    [SerializeField] public List<Transform> centreLines;


    private float velocity;
    private float acceleration;
    private float heading;

    private bool rotating = false;

    [SerializeField] private float lateralSpeed = 0.75f;
    [SerializeField] private float rotationStartProgress = 0.5f;
    [SerializeField] private float firstOffsetDistance = 0.5f;
    [SerializeField] private float secondOffsetDistance = 1.0f;

    private int initialDirection;      // -1 = left, +1 = right
    private float lateralTravel;
    private float headingTarget;

    [SerializeField] private float headingSmoothSpeed = 180f; // degrees/sec

    private float rotationTarget;      // radians
    private int rotationDirection;     // -1 = left, +1 = right

    private Vector3 previousPosition;

    private float rotationDuration;
    private float rotationElapsed;
    private float rotationAngularSpeed;
    private float rotationStartHeading;

    private enum LateralState
    {
        InitialOffset,     // 0.5 m to random side
        CrossOver,         // 1.0 m to opposite side
        Rotate,            // Continue crossing while rotating
        Finished
    }

    private LateralState lateralState;


    private void OnEnable()
    {
        //select the closest centre line
        if (centreLines == null || centreLines.Count == 0)
        {
            currentCentreLine = null;
            return;
        }

        Transform closest = null;
        float closestDist = float.MaxValue;

        Vector3 position = transform.position;

        foreach (Transform centreLine in centreLines)
        {
            if (centreLine == null)
                continue;

            float dist = (new Vector3(centreLine.position.x, position.y, position.z) - position).magnitude;
            // If this line is almost the same distance as the current closest,
            // randomly decide whether to keep the existing one or use this one.
            if (dist < closestDist - 0.1f)
            {
                // Definitely closer.
                closestDist = dist;
                closest = centreLine;
            }

            else if (Mathf.Abs(dist - closestDist) < 0.1f)
            {
                if (Random.value < 0.5f)
                {
                    closestDist = dist;
                    closest = centreLine;
                }
            }
        }

        currentCentreLine = closest;


        rotating = false;
        heading = vehicleController.GetHeading();
        velocity = vehicleController.GetCurrentVelocity();
        acceleration = -1.0f;

        initialDirection = Random.value < 0.5f ? -1 : 1;

        lateralState = LateralState.InitialOffset;
        lateralTravel = 0f;
        rotating = false;

        previousPosition = transform.position;

        rotationDirection = -initialDirection;

        // Random target between 90° and 130°.
        float targetDegrees = Random.Range(75f, 130f);

        rotationTarget = rotationDirection > 0
            ? targetDegrees * Mathf.Deg2Rad
            : -targetDegrees * Mathf.Deg2Rad;
    }


    private void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;
        HandleLongitudinal(dt);
        HandleLateral(dt);
        if (!rotating)
        {
            UpdateHeading();
        }
        else
        {
            UpdateRotation();
        }

        vehicleController.heading = heading;
        vehicleController.velocity = velocity;
        vehicleController.acceleration = acceleration;
    }

    private void HandleLongitudinal(float dt)
    {
        float error = vehicleController.GetTargetVelocity() - velocity;

        acceleration = -1.0f;

        //correct so that velocity does not get negative
        float minAcceleration = -velocity / dt;

        if (acceleration < minAcceleration)
            acceleration = minAcceleration;

        if (rotating)
        {
            acceleration = -4.0f;
        }

        velocity += acceleration * dt;

        velocity = Mathf.Max(0f, velocity);

        Vector3 pos = transform.position;
        pos += new Vector3(0f, 0f, velocity * dt);
        transform.position = pos;
    }

    private void HandleLateral(float dt)
    {
        float move = lateralSpeed * dt;

        switch (lateralState)
        {
            case LateralState.InitialOffset:
                {
                    MoveLaterally(initialDirection, move);

                    lateralTravel += move;

                    if (lateralTravel >= firstOffsetDistance)
                    {
                        lateralTravel = 0f;
                        lateralState = LateralState.CrossOver;
                    }

                    break;
                }

            case LateralState.CrossOver:
                {
                    MoveLaterally(-initialDirection, move);

                    lateralTravel += move;

                    // Start rotating halfway through the crossover.
                    if (!rotating &&
                        lateralTravel >= secondOffsetDistance * rotationStartProgress)
                    {
                        rotating = true;

                        float targetDegrees = Random.Range(90f, 130f);

                        rotationTarget = initialDirection > 0
                            ? targetDegrees * Mathf.Deg2Rad
                            : -targetDegrees * Mathf.Deg2Rad;

                        rotationStartHeading = heading;

                        // Time until velocity reaches zero with -4m/s²
                        rotationDuration = velocity / 4f;

                        rotationElapsed = 0f;


                        float angleDifference = Mathf.DeltaAngle(
                            rotationStartHeading * Mathf.Rad2Deg,
                            rotationTarget * Mathf.Rad2Deg
                        ) * Mathf.Deg2Rad;


                        // radians per second
                        rotationAngularSpeed = angleDifference / rotationDuration;

                        lateralState = LateralState.Rotate;
                    }

                    break;
                }

            case LateralState.Rotate:
                {
                    MoveLaterally(-initialDirection, move);

                    lateralTravel += move;

                    if (lateralTravel >= secondOffsetDistance)
                    {
                        lateralState = LateralState.Finished;
                    }

                    break;
                }
        }

    }
    private void MoveLaterally(int direction, float distance)
    {
        transform.position += Vector3.right * direction * distance;
    }

    private void UpdateHeading()
    {
        Vector3 delta = transform.position - previousPosition;

        if (delta.magnitude > 0.001f && velocity > 0.1f)
        {
            float targetHeading = Mathf.Atan2(delta.x, delta.z);

            float smoothHeading = Mathf.MoveTowardsAngle(
                heading * Mathf.Rad2Deg,
                targetHeading * Mathf.Rad2Deg,
                headingSmoothSpeed * Time.fixedDeltaTime
            );

            heading = smoothHeading * Mathf.Deg2Rad;

            transform.rotation = Quaternion.Euler(
                0f,
                smoothHeading,
                0f
            );
        }

        previousPosition = transform.position;
    }

    private void UpdateRotation()
    {
        float dt = Time.fixedDeltaTime;

        rotationElapsed += dt;


        float rotationStep = rotationAngularSpeed * dt;

        heading += rotationStep;


        transform.rotation = Quaternion.Euler(
            0f,
            heading * Mathf.Rad2Deg,
            0f
        );


        // stop exactly at target
        if (rotationElapsed >= rotationDuration)
        {
            heading = rotationTarget;

            transform.rotation = Quaternion.Euler(
                0f,
                heading * Mathf.Rad2Deg,
                0f
            );

            velocity = 0f;
            acceleration = 0f;

            rotating = false;
        }
    }



}