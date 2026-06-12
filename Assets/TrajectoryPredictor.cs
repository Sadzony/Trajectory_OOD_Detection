using UnityEngine;
using System.Collections.Generic;
public struct ObservedState
{
    public float t;

    public Vector3 position;   // x, y, z
    public float heading;      // radians

    public float velocity;
    public float acceleration;
}
public class PredictedTrajectory
{
    public List<ObservedState> states = new();
}

public class TrajectoryPredictor : MonoBehaviour
{
    [Header("References")]
    [SerializeField] private LaneDefinedMovement vehicleController;
    [SerializeField] private Transform[] centreLanes;

    [Header("Prediction")]
    [SerializeField] private float sampleRate = 0.02f;
    [SerializeField] private float predictionDuration = 2.5f;

    [Header("Debug")]
    [SerializeField] private LineRenderer lineRenderer;

    private PredictedTrajectory currentTrajectory;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        BuildInitialTrajectory();
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void BuildInitialTrajectory()
    {
        currentTrajectory = new PredictedTrajectory();

        float dt = sampleRate;
        int steps = Mathf.CeilToInt(predictionDuration / dt);

        Vector3 pos = vehicleController.transform.position;

        float velocity = vehicleController.GetCurrentVelocity();
        float accel = vehicleController.GetCurrentAcceleration();

        Transform lane = centreLanes[0];

        float heading = vehicleController.GetHeading();

        float t = 0f;

        for (int i = 0; i < steps; i++)
        {
            // --- longitudinal motion ---
            float targetV = vehicleController.GetTargetVelocity();
            float maxA = vehicleController.GetMaxAcceleration();

            float error = targetV - velocity;

            accel = Mathf.Clamp(error / dt, -maxA, maxA);
            velocity += accel * dt;

            float dz = velocity * dt;

            // Following the lane at a constant lateral position
            float targetX = lane.position.x;

            pos.x = targetX;

            pos.z += dz;

            // --- heading ---
            float dxdt = (targetX - pos.x) / dt;
            float dzdt = Mathf.Max(velocity, 0.0001f);

            heading = Mathf.Atan2(dxdt, dzdt);

            // --- store sample ---
            currentTrajectory.states.Add(new ObservedState
            {
                t = t,
                position = pos,
                heading = heading,
                velocity = velocity,
                acceleration = accel
            });

            t += dt;
        }

        RenderTrajectory();
    }

    private void RenderTrajectory()
    {
        if (lineRenderer == null || currentTrajectory == null)
            return;

        lineRenderer.positionCount = currentTrajectory.states.Count;

        for (int i = 0; i < currentTrajectory.states.Count; i++)
        {
            lineRenderer.SetPosition(i, currentTrajectory.states[i].position);
        }
    }
}
