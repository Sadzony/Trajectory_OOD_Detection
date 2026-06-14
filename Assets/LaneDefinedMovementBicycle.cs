using UnityEngine;

public class LaneDefinedMovementBicycle : MonoBehaviour
{
    [Header("Speed")]
    public float velocity = 10f;
    public float targetVelocity = 10f;
    public float maxAcceleration = 5f;

    [Header("Bicycle")]
    public float wheelBase = 2.5f;
    public float maxSteerAngle = 30f;

    [Header("Lane Change")]
    public float laneChangeDuration = 2f;

    private float accel;

    [SerializeField]
    private Transform currentLane;
    private Transform previousLane;

    private bool laneChanging = false;
    private float t = 0f;

    private float xStart;
    private float xEnd;

    private float desiredCurvature; // κ
    private float heading;

    void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;

        HandleVelocity(dt);
        ComputeTrajectory(dt);
        ApplyBicycle(dt);
    }

    // =========================================================
    // 1. LONGITUDINAL MODEL
    // =========================================================
    private void HandleVelocity(float dt)
    {
        float error = targetVelocity - velocity;

        accel = Mathf.Clamp(error / dt, -maxAcceleration, maxAcceleration);
        velocity += accel * dt;
    }

    // =========================================================
    // 2. TRAJECTORY → CURVATURE
    // =========================================================
    private void ComputeTrajectory(float dt)
    {
        if (currentLane == null) return;

        float laneX = currentLane.position.x;

        if (!laneChanging && previousLane != currentLane)
        {
            StartLaneChange(laneX);
        }

        if (!laneChanging)
        {
            // simple lane centering (pure P controller → curvature)
            float error = laneX - transform.position.x;
            desiredCurvature = error * 0.05f;
            return;
        }

        t += dt / laneChangeDuration;
        t = Mathf.Clamp01(t);

        float x0 = xStart;
        float x1 = xEnd;

        // =====================================================
        // Quintic-ish cosine trajectory (smooth, deterministic)
        // =====================================================
        float s = 0.5f - 0.5f * Mathf.Cos(Mathf.PI * t);

        float x = Mathf.Lerp(x0, x1, s);

        float sDot = (Mathf.PI * 0.5f * Mathf.Sin(Mathf.PI * t)) / laneChangeDuration;
        float xDot = (x1 - x0) * sDot;

        float sDDot = (Mathf.PI * Mathf.PI * Mathf.Cos(Mathf.PI * t)) /
                       (2f * laneChangeDuration * laneChangeDuration);

        float xDDot = (x1 - x0) * sDDot;

        // =====================================================
        // CURVATURE MODEL
        // κ ≈ x'' / v²  (small angle assumption)
        // =====================================================
        float v = Mathf.Max(velocity, 0.1f);

        desiredCurvature = xDDot / (v * v);

        bool done = t >= 1f;

        if (done)
        {
            t = 1;
            laneChanging = false;
        }
    }

    private void StartLaneChange(float laneX)
    {
        laneChanging = true;
        t = 0f;

        previousLane = currentLane;

        xStart = transform.position.x;
        xEnd = laneX;
    }

    // =========================================================
    // 3. CURVATURE → BICYCLE MODEL
    // =========================================================
    private void ApplyBicycle(float dt)
    {
        float steer = Mathf.Atan(desiredCurvature * wheelBase);

        steer = Mathf.Clamp(steer,
            -maxSteerAngle * Mathf.Deg2Rad,
             maxSteerAngle * Mathf.Deg2Rad);

        heading += (velocity / wheelBase) * Mathf.Tan(steer) * dt;

        Vector3 forward = new Vector3(
            Mathf.Sin(heading),
            0f,
            Mathf.Cos(heading)
        );

        transform.position += forward * velocity * dt;
        transform.rotation = Quaternion.Euler(0f, heading * Mathf.Rad2Deg, 0f);
    }

    // =========================================================
    public void SetTargetLane(Transform lane)
    {
        currentLane = lane;
    }
}
