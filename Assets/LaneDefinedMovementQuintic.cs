using UnityEngine;

public class LaneDefinedMovementQuintic : MonoBehaviour
{


    [SerializeField] private float vehicleLength;

    [Header("Longitudinal")]
    [SerializeField] private float targetLongitudinalVelocity = 20f;
    [SerializeField] private float maxLongitudinalAcceleration = 3f;



    [SerializeField] public float speedLimitMinimum = 10f;
    [SerializeField] public float speedLimitMaximum = 100f;


    [Header("Lateral")]
    [SerializeField] private float manouvreDuration = 2.5f;
    [SerializeField] private float laneAlignmentTolerance = 0.05f;
    [SerializeField] public Transform currentCentreLine;


    [Header("Lane Changing Noise")]
    [SerializeField] public float motionBias = 1f;

    [Header("Cruising Noise")]
    [SerializeField] private float cruiseOscillationPeriod = 5f;
    [SerializeField] private float cruiseOscillationMagnitude = 0.00f;
    private float cruiseStartTime;


    [Header("Braking")]
    [SerializeField] private float brakingDeceleration = 8f;
    [SerializeField] private float stopThreshold = 0.05f;
    private float laneChangeInitialVelocity;

    private float heading;

    private float velocity;
    private float acceleration;

    //accessor functions
    public float GetManouvreDuration() => manouvreDuration;
    public float GetTargetVelocity() => targetLongitudinalVelocity;

    public float GetMaxAcceleration() => maxLongitudinalAcceleration;
    public float GetCurrentVelocity() => velocity;
    public float GetCurrentAcceleration() => acceleration;
    public float GetHeading() => heading;

    public float GetVehicleLength() => vehicleLength;

    public float GetSpeedLimitMin() => speedLimitMinimum;
    public float GetSpeedLimitMax() => speedLimitMaximum;

    public enum LongitudinalState
    {
        Cruise,
        Braking,
        Stopped,
    }
    public LongitudinalState longitudinalState = LongitudinalState.Cruise;

    public enum LateralState
    {
        Cruise,
        LaneChange,
    }

    public LateralState lateralState;

    private float laneChangeTime;
    private float laneChangeStartX;
    private float laneChangeTargetX;


    private void Start()
    {
        velocity = targetLongitudinalVelocity;
        heading = transform.eulerAngles.y * Mathf.Deg2Rad;
        cruiseStartTime = Time.time;

        longitudinalState = LongitudinalState.Cruise;
    }

    private void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;

        UpdateLongitudinal(dt);
        UpdateLateralState();
        UpdateLateralTrajectory(dt);
        UpdateHeading();
    }


    //longitudinal motion is independent from lateral trajectories - simple velocity controller
    private void UpdateLongitudinal(float dt)
    {
        switch (longitudinalState)
        {
            case LongitudinalState.Cruise:
                {
                    float error = targetLongitudinalVelocity - velocity;

                    acceleration = Mathf.Clamp(
                        error / dt,
                        -maxLongitudinalAcceleration,
                        maxLongitudinalAcceleration);

                    break;
                }

            case LongitudinalState.Braking:
                {
                    acceleration = -brakingDeceleration;

                    break;
                }

            case LongitudinalState.Stopped:
                {
                    acceleration = 0f;
                    velocity = 0f;
                    break;
                }
        }

        velocity += acceleration * dt;

        if (longitudinalState == LongitudinalState.Braking &&
            velocity <= stopThreshold)
        {
            velocity = 0f;
            acceleration = 0f;
            longitudinalState = LongitudinalState.Stopped;
        }

        velocity = Mathf.Max(0f, velocity);

        Vector3 pos = transform.position;
        pos += new Vector3(0f, 0f, velocity * dt);
        transform.position = pos;
    }


    //finds out if we should engage a lane change
    private void UpdateLateralState()
    {
        float lateralError = currentCentreLine.position.x - transform.position.x;

        if (lateralState == LateralState.Cruise)
        {
            if (Mathf.Abs(lateralError) > laneAlignmentTolerance)
            {
                StartLaneChange();
            }
        }
    }

    private void StartLaneChange()
    {
        lateralState = LateralState.LaneChange;

        laneChangeTime = 0f;
        laneChangeStartX = transform.position.x;
        laneChangeTargetX = currentCentreLine.position.x;

        laneChangeInitialVelocity = velocity;
    }


    //quintic change lane trajectory
    private void UpdateLateralTrajectory(float dt)
    {
        if (lateralState != LateralState.LaneChange)
        {
            // Cruise: lock to lane center
            Vector3 pos = transform.position;

            //oscillate within lane
            float elapsed = Time.time - cruiseStartTime;

            float phase =
                (2f * Mathf.PI * elapsed)
                / cruiseOscillationPeriod;

            float offset =
                cruiseOscillationMagnitude * Mathf.Sin(phase);

            pos.x = currentCentreLine.position.x + offset;
            transform.position = pos;
        }
        else
        {
            float speedScale = 1.0f;
            if (velocity < laneChangeInitialVelocity)
            {
                speedScale = velocity > stopThreshold
                    ? velocity / laneChangeInitialVelocity
                    : 0f;
            }

            laneChangeTime += dt * speedScale;

            float u = laneChangeTime / manouvreDuration;

            if (u >= 1f)
            {
                u = 1f;
                cruiseStartTime = Time.time;
                lateralState = LateralState.Cruise;
            }

            u = Mathf.Pow(u, 1f / motionBias);

            // Quintic step
            float s =
                10f * u * u * u
                - 15f * u * u * u * u
                + 6f * u * u * u * u * u;

            float deltaX = laneChangeTargetX - laneChangeStartX;

            float x = laneChangeStartX + deltaX * s;

            Vector3 posFinal = transform.position;
            posFinal.x = x;
            transform.position = posFinal;
        }
    }


    //heading from lateral motion
    private void UpdateHeading()
    {
        // If we're basically stopped, freeze heading
        if (velocity <= stopThreshold)
        {
            return;
        }

        float deltaX = laneChangeTargetX - laneChangeStartX;

        float u = Mathf.Clamp01(laneChangeTime / manouvreDuration);

        float ds =
            30f * u * u
            - 60f * u * u * u
            + 30f * u * u * u * u;

        float dxdt = (deltaX * ds) / manouvreDuration;

        float plannedDzdt = laneChangeInitialVelocity;

        heading = Mathf.Atan2(dxdt, plannedDzdt);

        transform.rotation = Quaternion.Euler(
            0f,
            heading * Mathf.Rad2Deg,
            0f
        );
    }

    public void StartBraking()
    {
        if (longitudinalState != LongitudinalState.Stopped)
            longitudinalState = LongitudinalState.Braking;
    }

    public void ResumeDriving()
    {
        longitudinalState = LongitudinalState.Cruise;
    }
}
