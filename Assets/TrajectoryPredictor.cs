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

public enum PredictionMode
{
    ADE,
    FDE,
}
public class Trajectory
{
    public float trajectoryStart = 0.0f;
    public Transform LaneFrom;
    public Transform LaneTo;
    public List<VehicleState> states = new();
}

public class TrajectoryPredictor : MonoBehaviour
{
    [Header("References")]
    [SerializeField] private LaneDefinedMovementQuintic vehicleController;
    [SerializeField] private List<Transform> centreLanes;

    [Header("Prediction")]
    private float sampleRate = 0.02f;
    [SerializeField] PredictionMode predictionMode;

    [Header("Debug")]
    [SerializeField] private LineRenderer lineRenderer;

    public Trajectory currentTrajectory;

    private float simulationTime;


    public List<VehicleState> Observations = new List<VehicleState>();

    public List<Trajectory> TransitionTrajectories = new List<Trajectory>();

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        simulationTime = 0.0f;
        Observations = ObserveVehicle();
        var latestObservation = Observations[Observations.Count - 1];
        currentTrajectory = BuildLaneFollowTrajectory(centreLanes[0], latestObservation);
        // populate the first entries to transition trajectories
        TransitionTrajectories = FindTransitionTrajectories(latestObservation);
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        simulationTime += Time.fixedDeltaTime;
        Observations = ObserveVehicle();

        var latestObservation = Observations[Observations.Count - 1];

        //if trajectory has finished, set a new lane follow trajectory as the next one
        if (vehicleController.transform.position.z > currentTrajectory.states[currentTrajectory.states.Count - 1].position.z)
        {
            currentTrajectory = BuildLaneFollowTrajectory(currentTrajectory, latestObservation);
        }

        //1. Update the set of transition Trajectories based on observed state
        TransitionTrajectories = FindTransitionTrajectories(latestObservation);

        //2 Transition to a different, or stay on current trajectory, based on error measure
        currentTrajectory = SelectAlikeTrajectory(currentTrajectory, TransitionTrajectories, Observations);

        RenderTrajectory();
    }

    private Trajectory SelectAlikeTrajectory(Trajectory currentTrajectory, List<Trajectory> transitionTrajectories, List<VehicleState> Observations)
    {
        var potentialTrajectories = new List<Trajectory>(transitionTrajectories);
        potentialTrajectories.Insert(0, currentTrajectory);
        List<KeyValuePair<Trajectory, float>> trajectoryErrorMeasures = new List<KeyValuePair<Trajectory, float>>();
        Trajectory bestTrajectory = currentTrajectory;
        float bestError = float.PositiveInfinity;
        float tLatest = Observations[^1].t;
        int obsCount = Observations.Count;

        int FindClosestIndex(List<VehicleState> states, float t)
        {
            int bestIndex = states.Count - 1;
            float bestError = float.MaxValue;

            for (int i = 0; i < states.Count; i++)
            {
                float error = Mathf.Abs(states[i].t - t);

                if (error < bestError)
                {
                    bestError = error;
                    bestIndex = i;
                }
                else if (states[i].t > t)
                {
                    break;
                }
            }

            return bestIndex;
        }

        if (predictionMode == PredictionMode.ADE)
        {
            foreach (Trajectory trajectory in potentialTrajectories)
            {
                if (trajectory.states.Count == 0)
                    continue;

                int anchorIndex = FindClosestIndex(trajectory.states, tLatest);

                float totalError = 0f;
                int count = 0;

                for (int i = obsCount - 1; i >= 0; i--)
                {
                    int trajIndex = anchorIndex - (obsCount - 1 - i);

                    if (trajIndex < 0)
                        break;

                    totalError += Vector3.Distance(
                        Observations[i].position,
                        trajectory.states[trajIndex].position);

                    count++;
                }

                float ade = count > 0
                    ? totalError / count
                    : float.PositiveInfinity;

                if (ade < bestError)
                {
                    bestError = ade;
                    bestTrajectory = trajectory;
                }
            }
        }
        return bestTrajectory;
    }

    public List<Trajectory> FindTransitionTrajectories(VehicleState fromState)
    {
        float dt = sampleRate;

        //Filter out old trajectories
        float cutoffTime = fromState.t - vehicleController.GetManouvreDuration();
        TransitionTrajectories.RemoveAll(t => t.trajectoryStart < cutoffTime);

        //select the state in the current trajectory with the current simulation time - this is the starting point of the transition trajectories
        var tolerance = sampleRate * 0.5f;
        var transitionStartState = currentTrajectory.states.Find(s => Mathf.Abs(s.t - fromState.t) < tolerance);

        //Examine the current trajectory, using the FromLane and ToLane parameters to decide whether its a lane change or lane follow
        bool isLaneFollow = currentTrajectory.LaneFrom == currentTrajectory.LaneTo;

        // if its lane follow: append a lane follow with the new observed state, and a lane change to other lanes
        if (isLaneFollow)
        {
            var newLaneFollowTrajectory = new Trajectory();
            newLaneFollowTrajectory.LaneFrom = currentTrajectory.LaneFrom;
            newLaneFollowTrajectory.LaneTo = currentTrajectory.LaneTo;

            var lastLeadingState = transitionStartState;
            var lastTrajectoryStates = currentTrajectory.states.Where(s => s.t >= transitionStartState.t - vehicleController.GetManouvreDuration() && s.t < transitionStartState.t);
            foreach (var state in lastTrajectoryStates)
            {
                newLaneFollowTrajectory.states.Add(state);
            }
            Transform lane = newLaneFollowTrajectory.LaneTo;

            var t = fromState.t;
            newLaneFollowTrajectory.trajectoryStart = t;

            var followingLaneTrajectory = BuildLaneFollowTrajectory(lane, fromState);
            foreach(var state in followingLaneTrajectory.states)
            {
                newLaneFollowTrajectory.states.Add(state);
            }
            TransitionTrajectories.Add(newLaneFollowTrajectory);

            int currentLaneIndex = centreLanes.IndexOf(lane);

            for (int i = 0; i < centreLanes.Count; i++)
            {
                if (Mathf.Abs(i - currentLaneIndex) == 1)
                {
                    var laneCentre = centreLanes[i];

                    var laneChangeTrajectory = new Trajectory();
                    laneChangeTrajectory.LaneFrom = currentTrajectory.LaneFrom;
                    laneChangeTrajectory.LaneTo = laneCentre;

                    foreach (var state in lastTrajectoryStates)
                    {
                        laneChangeTrajectory.states.Add(state);
                    }

                    laneChangeTrajectory.trajectoryStart = fromState.t;

                    var changingLaneTrajectory =
                        BuildLaneChangeTrajectory(
                            laneChangeTrajectory.LaneFrom,
                            laneChangeTrajectory.LaneTo,
                            fromState,
                            vehicleController.GetManouvreDuration());

                    foreach (var state in changingLaneTrajectory.states)
                    {
                        laneChangeTrajectory.states.Add(state);
                    }

                    TransitionTrajectories.Add(laneChangeTrajectory);
                }
            }


        }
        //if its lane change: append a lane change to the same lane with the new observed state
        else if (!isLaneFollow)
        {
            // find the time we've spent within current trajectory
            var timeWithinCurrent = fromState.t - currentTrajectory.trajectoryStart;
            
            if(timeWithinCurrent >= 0)
            {
                var differenceToManouvreDuration = vehicleController.GetManouvreDuration() - timeWithinCurrent;

                //first we need will create an updated trajectory with the new observations
                var updatedLaneChangeTrajectory = new Trajectory();
                updatedLaneChangeTrajectory.LaneFrom = currentTrajectory.LaneFrom;
                updatedLaneChangeTrajectory.LaneTo = currentTrajectory.LaneTo;

                var lastTrajectoryStates = currentTrajectory.states.Where(s => s.t >= transitionStartState.t - vehicleController.GetManouvreDuration() && s.t < transitionStartState.t);
                foreach (var state in lastTrajectoryStates)
                {
                    updatedLaneChangeTrajectory.states.Add(state);
                }

                updatedLaneChangeTrajectory.trajectoryStart = currentTrajectory.trajectoryStart;

                var changingLaneTrajectory =
                    BuildLaneChangeTrajectory(
                        updatedLaneChangeTrajectory.LaneFrom,
                        updatedLaneChangeTrajectory.LaneTo,
                        fromState,
                        differenceToManouvreDuration);

                foreach (var state in changingLaneTrajectory.states)
                {
                    updatedLaneChangeTrajectory.states.Add(state);
                }

                TransitionTrajectories.Add(updatedLaneChangeTrajectory);
            }
        }
        return TransitionTrajectories;
    }

    public List<VehicleState> ObserveVehicle()
    {
        //generate current state
        var newState = new VehicleState
        {
            t = simulationTime,
            velocity = vehicleController.GetCurrentVelocity(),
            acceleration = vehicleController.GetCurrentAcceleration(),
            heading = vehicleController.GetHeading(),
            position = new Vector3(vehicleController.transform.position.x, vehicleController.transform.position.y, vehicleController.transform.position.z),
        };
        // filter the observation buffer: remove all entries where VehicleState.t < simulationTime - ManouvreDuration
        // Remove observations older than the manoeuvre duration
        float cutoffTime = simulationTime - vehicleController.GetManouvreDuration();

        Observations.RemoveAll(state => state.t < cutoffTime);

        // Append newest observation
        Observations.Add(newState);
        return Observations;
    }

    public Trajectory BuildLaneFollowTrajectory(Transform lane, VehicleState Observation)
    {
        Trajectory resultTrajectory = new Trajectory();

        resultTrajectory.LaneFrom = lane;
        resultTrajectory.LaneTo = lane;

        float dt = sampleRate;

        Vector3 pos = new Vector3(lane.position.x, Observation.position.y, Observation.position.z);

        float velocity = Observation.velocity;
        float accel = Observation.acceleration;



        float heading = Observation.heading;

        float t = Observation.t;
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
    public Trajectory BuildLaneFollowTrajectory(Trajectory leadingTrajectory, VehicleState Observation)
    {
        Trajectory resultTrajectory = new Trajectory();

        resultTrajectory.LaneFrom = leadingTrajectory.LaneTo;
        resultTrajectory.LaneTo = resultTrajectory.LaneFrom;

        Transform lane = resultTrajectory.LaneTo;

        var lastLeadingState = leadingTrajectory.states[leadingTrajectory.states.Count - 1];
        var lastTrajectoryStates = leadingTrajectory.states.Where(s => s.t >= leadingTrajectory.trajectoryStart);
        foreach (var state in lastTrajectoryStates)
        {
            resultTrajectory.states.Add(state);
        }

        float dt = sampleRate;

        Vector3 pos = new Vector3(lane.position.x, Observation.position.y, Observation.position.z);

        float velocity = Observation.velocity;
        float accel = Observation.acceleration;



        float heading = Observation.heading;




        resultTrajectory.trajectoryStart = Observation.t;
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

    private Trajectory BuildLaneChangeTrajectory(Transform LaneFrom, Transform LaneTo, VehicleState Observation, float duration)
    {
        Trajectory resultTrajectory = new Trajectory();

        resultTrajectory.LaneFrom = LaneFrom;
        resultTrajectory.LaneTo = LaneTo;

        float dt = sampleRate;

        Vector3 pos = Observation.position;

        float velocity = Observation.velocity;
        float accel = Observation.acceleration;

        float heading = Observation.heading;

        float t = Observation.t;
        resultTrajectory.trajectoryStart = t;

        float trajectoryTime = 0f;

        float startX = Observation.position.x;
        float targetX = LaneTo.position.x;
        float deltaX = targetX - startX;

        // First point (matches current observation)
        float u = 0f;

        float ds =
            30f * u * u
            - 60f * u * u * u
            + 30f * u * u * u * u;

        float dxdt = (deltaX * ds) / duration;
        float dzdt = Mathf.Max(velocity, 0.0001f);

        heading = Mathf.Atan2(dxdt, dzdt);

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

        while (trajectoryTime < duration)
        {
            // longitudinal motion
            velocity += accel * dt;

            float dz = velocity * dt;
            pos.z += dz;

            // lane-change progress
            u = Mathf.Clamp01(
                trajectoryTime / duration
            );

            // Quintic smoothstep
            float s =
                10f * u * u * u
                - 15f * u * u * u * u
                + 6f * u * u * u * u * u;

            pos.x = startX + deltaX * s;

            // heading from quintic derivative
            ds =
                30f * u * u
                - 60f * u * u * u
                + 30f * u * u * u * u;

            dxdt =
                (deltaX * ds)
                / duration;

            dzdt = Mathf.Max(velocity, 0.0001f);

            heading = Mathf.Atan2(dxdt, dzdt);

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
