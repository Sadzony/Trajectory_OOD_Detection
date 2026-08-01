using UnityEngine;
using System.Collections.Generic;
public class TrailOffBehaviour : MonoBehaviour
{
    [SerializeField] public Transform currentTarget;
    [SerializeField] public List<Transform> targets;
    [SerializeField] public LaneDefinedMovementQuintic vehicleController;

    [SerializeField] public float velocity;
    [SerializeField] public float acceleration;

    [Header("Lateral Bicycle Model")]
    [SerializeField] private float wheelBase = 2.7f;
    [SerializeField] public float maxWheelRotation = 30f;
    [SerializeField] private float steeringResponse = 3f;

    [SerializeField] private float brakingAcceleration = -10f;
    [SerializeField] private float stopThreshold = 0.05f;

    private float heading;
    private float steeringAngle;

    private bool braking;

    private void OnEnable()
    {
        //select one of the targets at random
        if (targets == null || targets.Count == 0)
        {
            currentTarget = null;
            return;
        }

        currentTarget = targets[Random.Range(0, targets.Count)];
        velocity = vehicleController.GetCurrentVelocity();
        acceleration = -1.0f;
        heading = transform.eulerAngles.y * Mathf.Deg2Rad;
        braking = false;
    }
    private void OnDisable()
    {
        braking = false;
        vehicleController.heading = heading;
        vehicleController.velocity = velocity;
        vehicleController.acceleration = acceleration;
        acceleration = -1.0f;
        steeringAngle = 0;
    }

    private void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;


        UpdateLongitudinal(dt);

        UpdateSteering(dt);

        UpdateBicycleModel(dt);

        CheckPassedTarget();

        vehicleController.heading = heading;
        vehicleController.velocity = velocity;
        vehicleController.acceleration = acceleration;
    }



    private void UpdateLongitudinal(float dt)
    {
        if (braking)
        {
            acceleration = brakingAcceleration;
        }
        else
        {
            acceleration = -1.0f;
            float minAcceleration = -velocity / dt;

            if (acceleration < minAcceleration)
                acceleration = minAcceleration;

        }


        velocity += acceleration * dt;


        // prevent reverse motion
        if (velocity < 0f)
        {
            velocity = 0f;
            acceleration = 0f;
        }
    }



    private void UpdateSteering(float dt)
    {
        Vector3 targetDirection =
            new Vector3(currentTarget.position.x, transform.position.y, transform.position.z+10) -
            transform.position;


        float desiredHeading =
            Mathf.Atan2(
                targetDirection.x,
                targetDirection.z);



        float desiredHeadingDegrees =
            desiredHeading *
            Mathf.Rad2Deg;


        float currentHeadingDegrees =
            heading *
            Mathf.Rad2Deg;



        float headingError =
            Mathf.DeltaAngle(
                currentHeadingDegrees,
                desiredHeadingDegrees);



        float targetSteering =
            Mathf.Clamp(
                headingError,
                -maxWheelRotation,
                maxWheelRotation);



        steeringAngle =
            Mathf.Lerp(
                steeringAngle,
                targetSteering,
                steeringResponse * dt);
    }



    private void UpdateBicycleModel(float dt)
    {
        /*
         * Bicycle model
         *
         * steering angle -> yaw rate
         */

        float yawRate =
            (velocity / wheelBase) *
            Mathf.Tan(
                steeringAngle *
                Mathf.Deg2Rad);



        /*
         * Integrate yaw
         *
         * yaw rate -> heading
         */

        heading += yawRate * dt;



        /*
         * Heading -> velocity vector
         */

        Vector3 velocityVector =
            new Vector3(
                Mathf.Sin(heading),
                0f,
                Mathf.Cos(heading))
            *
            velocity;



        /*
         * Apply movement
         */

        transform.position +=
            velocityVector * dt;



        transform.rotation =
            Quaternion.Euler(
                0f,
                heading *
                Mathf.Rad2Deg,
                0f);
    }



    private void CheckPassedTarget()
    {
        if (braking)
            return;


        float targetX = currentTarget.position.x;


        // Target is on the left side
        if (targetX < 0f)
        {
            if (transform.position.x <= targetX)
            {
                braking = true;
            }
        }
        // Target is on the right side
        else if (targetX > 0f)
        {
            if (transform.position.x >= targetX)
            {
                braking = true;
            }
        }
    }
}
