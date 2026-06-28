using UnityEngine;

public class LaneDefinedMovementQuintic : MonoBehaviour
{
    // ----------------------------
    // LONGITUDINAL MOTION
    // ----------------------------

    [SerializeField] private float vehicleLength;

    [Header("Longitudinal")]
    [SerializeField] private float targetLongitudinalVelocity = 10f;
    [SerializeField] private float maxLongitudinalAcceleration = 3f;



    // ----------------------------
    // LATERAL MOTION
    // ----------------------------
    [Header("Lateral")]
    [SerializeField] private float manouvreDuration = 2.5f;
    [SerializeField] private float laneAlignmentTolerance = 0.05f;
    [SerializeField] private float motionBias = 1f;
    [SerializeField] private float cruiseOscillationPeriod = 5f;
    [SerializeField] private float cruiseOscillationMagnitude = 0.05f;
    [SerializeField] private Transform currentCentreLine;
    private float cruiseStartTime;

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

    private enum LateralState
    {
        Cruise,
        LaneChange
    }

    private LateralState lateralState;

    private float laneChangeTime;
    private float laneChangeStartX;
    private float laneChangeTargetX;



    // ----------------------------
    // INIT
    // ----------------------------
    private void Start()
    {
        velocity = targetLongitudinalVelocity;
        heading = transform.eulerAngles.y * Mathf.Deg2Rad;
        cruiseStartTime = Time.time;
    }

    private void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;

        UpdateLongitudinal(dt);
        UpdateLateralState();
        UpdateLateralTrajectory(dt);
        UpdateHeading();
    }

    // =========================================================
    // LONGITUDINAL (INDEPENDENT OF LATERAL)
    // =========================================================
    private void UpdateLongitudinal(float dt)
    {
        float error = targetLongitudinalVelocity - velocity;

        acceleration = Mathf.Clamp(
            error / dt,
            -maxLongitudinalAcceleration,
            maxLongitudinalAcceleration
        );

        velocity += acceleration * dt;

        float dz = velocity * dt;

        Vector3 pos = transform.position;
        pos += new Vector3(0f, 0f, dz);

        transform.position = pos;
    }

    // =========================================================
    // STATE TRANSITIONS
    // =========================================================
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
    }

    // =========================================================
    // LATERAL QUINTIC TRAJECTORY
    // =========================================================
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
            laneChangeTime += dt;

            float u = laneChangeTime / manouvreDuration;

            if (u >= 1f)
            {
                u = 1f;
                cruiseStartTime = Time.time;
                lateralState = LateralState.Cruise;
            }

            u = Mathf.Pow(u, 1f / motionBias);

            // Quintic smoothstep
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

    // =========================================================
    // HEADING FROM TRAJECTORY
    // =========================================================
    private void UpdateHeading()
    {
        float deltaX = laneChangeTargetX - laneChangeStartX;

        float u = Mathf.Clamp01(laneChangeTime / manouvreDuration);

        float ds =
            30f * u * u
            - 60f * u * u * u
            + 30f * u * u * u * u;

        float dxdt = (deltaX * ds) / manouvreDuration;

        float dzdt = Mathf.Max(velocity, 0.0001f);

        heading = Mathf.Atan2(dxdt, dzdt);

        transform.rotation = Quaternion.Euler(
            0f,
            heading * Mathf.Rad2Deg,
            0f
        );
    }
}
