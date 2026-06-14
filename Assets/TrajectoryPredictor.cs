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

    private VehicleState ObservedState = new();

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        simulationTime = 0.0f;
        ObservedState = ObserveVehicle();
        currentTrajectory = BuildLaneFollowTrajectory();
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        simulationTime += Time.fixedDeltaTime;
        ObservedState = ObserveVehicle();
        if (vehicleController.transform.position.z > currentTrajectory.states[currentTrajectory.states.Count - 1].position.z)
        {
            currentTrajectory = BuildLaneFollowTrajectory(currentTrajectory);
        }
        RenderTrajectory();
    }

    public VehicleState ObserveVehicle()
    {
        return new VehicleState
        {
            t = simulationTime,
            velocity = vehicleController.GetCurrentVelocity(),
            acceleration = vehicleController.GetCurrentAcceleration(),
            heading = vehicleController.GetHeading(),
            position = new Vector3(vehicleController.transform.position.x, vehicleController.transform.position.y, vehicleController.transform.position.z),
        };
    }

    public Trajectory BuildLaneFollowTrajectory()
    {
        Trajectory resultTrajectory = new Trajectory();

        float dt = sampleRate;

        Vector3 pos = ObservedState.position;

        float velocity = ObservedState.velocity;
        float accel = ObservedState.acceleration;

        Transform lane = centreLanes[0];

        float heading = vehicleController.GetHeading();

        float t = simulationTime;
        resultTrajectory.trajectoryStart = t;
        float trajectoryTime = 0f;

        //first point in the sequence, matching t = 0

        // --- heading ---
        float dxdt1 = (lane.position.x - pos.x) / dt;
        float dzdt1 = Mathf.Max(velocity, 0.0001f);
        heading = Mathf.Atan2(dxdt1, dzdt1);
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

        while (trajectoryTime < vehicleController.GetManouvreDuration())
        {
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
        var lastTrajectoryStates = leadingTrajectory.states.Where(s => s.t >= leadingTrajectory.trajectoryStart);
        foreach (var state in lastTrajectoryStates)
        {
            resultTrajectory.states.Add(state);
        }

        float dt = sampleRate;

        Vector3 pos = ObservedState.position;

        float velocity = ObservedState.velocity;
        float accel = ObservedState.acceleration;

        Transform lane = centreLanes[0];

        float heading = vehicleController.GetHeading();




        resultTrajectory.trajectoryStart = simulationTime;
        float t = resultTrajectory.trajectoryStart;
        float trajectoryTime = 0f;
        //first point in the sequence, matching t = start

        // --- heading ---
        float dxdt1 = (lane.position.x - pos.x) / dt;
        float dzdt1 = Mathf.Max(velocity, 0.0001f);
        heading = Mathf.Atan2(dxdt1, dzdt1);
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
        while (trajectoryTime < vehicleController.GetManouvreDuration())
        {
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
