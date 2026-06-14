using UnityEngine;
using System.Collections.Generic;
using System.Linq;

public struct VehicleState
{
    public float t;

    public Vector3 position;   // x, y, z
    public float heading;      // radians

    public float velocity;
    public float acceleration;
}
public class Trajectory
{
    public float trajectoryStart = 0.0f;
    public List<VehicleState> states = new();
}

public class TrajectoryPredictor : MonoBehaviour
{
    [Header("References")]
    [SerializeField] private LaneDefinedMovementQuintic vehicleController;
    [SerializeField] private Transform[] centreLanes;

    [Header("Prediction")]
    [SerializeField] private float sampleRate = 0.02f;

    [Header("Debug")]
    [SerializeField] private LineRenderer lineRenderer;

    private Trajectory currentTrajectory;

    private float simulationTime;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        simulationTime = 0.0f;
        currentTrajectory = BuildLaneFollowTrajectory();
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        simulationTime += Time.fixedDeltaTime;
        if(vehicleController.transform.position.z > currentTrajectory.states[currentTrajectory.states.Count - 1].position.z)
        {
            currentTrajectory = BuildLaneFollowTrajectory(currentTrajectory);
        }
        RenderTrajectory();
    }

    public Trajectory BuildLaneFollowTrajectory()
    {
        Trajectory resultTrajectory = new Trajectory();

        float dt = sampleRate;

        Vector3 pos = vehicleController.transform.position;

        float velocity = vehicleController.GetCurrentVelocity();
        float accel = vehicleController.GetCurrentAcceleration();

        Transform lane = centreLanes[0];

        float heading = vehicleController.GetHeading();

        float t = simulationTime;
        resultTrajectory.trajectoryStart = t;
        float trajectoryTime = 0f;

        while (trajectoryTime < vehicleController.GetManouvreDuration())
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
            resultTrajectory.states.Add(new VehicleState
            {
                t = t,
                position = pos,
                heading = heading,
                velocity = velocity,
                acceleration = accel
            });

            t += dt;
            trajectoryTime += dt;
        }
        return resultTrajectory;
    }
    public Trajectory BuildLaneFollowTrajectory(Trajectory leadingTrajectory)
    {
        Trajectory resultTrajectory = new Trajectory();

        var lastLeadingState = leadingTrajectory.states[leadingTrajectory.states.Count - 1];
        leadingTrajectory.states.RemoveAt(leadingTrajectory.states.Count - 1);
        var lastTrajectoryStates = leadingTrajectory.states.Where(s => s.t >= leadingTrajectory.trajectoryStart);
        foreach (var state in lastTrajectoryStates)
        {
            resultTrajectory.states.Add(state);
        }

        float dt = sampleRate;

        Vector3 pos = vehicleController.transform.position;

        float velocity = vehicleController.GetCurrentVelocity();
        float accel = vehicleController.GetCurrentAcceleration();

        Transform lane = centreLanes[0];

        float heading = vehicleController.GetHeading();

        

        float t = lastLeadingState.t;
        resultTrajectory.trajectoryStart = t;
        float trajectoryTime = 0f;

        while (trajectoryTime < vehicleController.GetManouvreDuration())
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
            resultTrajectory.states.Add(new VehicleState
            {
                t = t,
                position = pos,
                heading = heading,
                velocity = velocity,
                acceleration = accel
            });

            t += dt;
            trajectoryTime += dt;
        }
        return resultTrajectory;
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
