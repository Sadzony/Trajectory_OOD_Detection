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
    public Trajectory() { }
    public Trajectory(Trajectory copy) { trajectoryStart = copy.trajectoryStart; Lane = copy.Lane; states = new List<VehicleState>(copy.states); followingTrajectory = copy.followingTrajectory; }
    public float trajectoryStart = 0.0f;
    public Transform Lane;
    public List<VehicleState> states = new();
    public Trajectory? followingTrajectory = null;
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

        currentTrajectory = UpdateTrajectory(currentTrajectory, latestObservation);

        //1. Update the set of transition Trajectories based on observed state
        TransitionTrajectories = FindTransitionTrajectories(latestObservation);

        //2 Transition to a different, or stay on current trajectory, based on error measure
        currentTrajectory = SelectAlikeTrajectory(currentTrajectory, TransitionTrajectories, Observations);

        RenderTrajectory();
    }

    private Trajectory UpdateTrajectory(Trajectory trajectory, VehicleState Observation)
    {
        var resultTrajectory = new Trajectory(trajectory);

        float dt = sampleRate;

        //generate a following trajectory
        if(trajectory.followingTrajectory == null)
        {
            resultTrajectory.followingTrajectory = BuildLaneFollowTrajectory(trajectory, trajectory.Lane, trajectory.states[^1]);
        }
        if(Observation.t >= trajectory.states[^1].t)
        {
            resultTrajectory = new Trajectory(resultTrajectory.followingTrajectory);
            resultTrajectory.followingTrajectory = BuildLaneFollowTrajectory(resultTrajectory, resultTrajectory.Lane, resultTrajectory.states[^1]);
        }

        return resultTrajectory;
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
                float halfLength = vehicleController.GetVehicleLength() / 2;

                for (int i = obsCount - 1; i >= 0; i--)
                {
                    int trajIndex = anchorIndex - (obsCount - 1 - i);

                    if (trajIndex < 0)
                        break;

                    VehicleState obs = Observations[i];
                    VehicleState pred = trajectory.states[trajIndex];

                    // --- forward vectors (heading in radians) ---
                    Vector3 obsForward = new Vector3(Mathf.Sin(obs.heading), 0f, Mathf.Cos(obs.heading));
                    Vector3 predForward = new Vector3(Mathf.Sin(pred.heading), 0f, Mathf.Cos(pred.heading));

                    // --- front/rear points (OBS) ---
                    Vector3 obsFront = obs.position + obsForward * halfLength;
                    Vector3 obsRear = obs.position - obsForward * halfLength;

                    // --- front/rear points (PRED) ---
                    Vector3 predFront = pred.position + predForward * halfLength;
                    Vector3 predRear = pred.position - predForward * halfLength;

                    // --- rigid body error ---
                    float frontError = Vector3.Distance(obsFront, predFront);
                    float rearError = Vector3.Distance(obsRear, predRear);

                    float error = (frontError + rearError) * 0.5f;

                    totalError += error;
                    count++;
                }

                float ade = count > 0
                    ? totalError / count
                    : float.PositiveInfinity;

                if (ade < bestError)
                {
                    if(bestError != float.PositiveInfinity)
                        Debug.Log("Found better: " + ade + " vs cuurrent: " + bestError);
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


        //1. create the same trajectory, updated with new observation
        var timeWithinCurrent = fromState.t - currentTrajectory.trajectoryStart;
        Transform lane = currentTrajectory.Lane;
        var lastLeadingState = transitionStartState;
        var lastTrajectoryStates = currentTrajectory.states.Where(s => s.t >= transitionStartState.t - vehicleController.GetManouvreDuration() && s.t <= transitionStartState.t);

        if (timeWithinCurrent > 0)
        {
            var differenceToManouvreDuration = vehicleController.GetManouvreDuration() - timeWithinCurrent;

            var observationUpdateTrajectory = new Trajectory();
            observationUpdateTrajectory.Lane = currentTrajectory.Lane;

            foreach (var state in lastTrajectoryStates)
            {
                observationUpdateTrajectory.states.Add(state);
            }

            observationUpdateTrajectory.trajectoryStart = currentTrajectory.trajectoryStart;

            var updatedTraj =
                BuildQuinticTrajectory(
                    observationUpdateTrajectory.Lane,
                    fromState,
                    differenceToManouvreDuration);

            var newStates = updatedTraj.states.Where(s => s.t > transitionStartState.t);
            foreach (var state in newStates)
            {
                observationUpdateTrajectory.states.Add(state);
            }

            TransitionTrajectories.Add(observationUpdateTrajectory);
        }

        //2. Create a state change trajectory, taking the full manouvre duration
        int currentLaneIndex = centreLanes.IndexOf(lane);
        for (int i = 0; i < centreLanes.Count; i++)
        {
            //act only on lanes which are 1 index away from this lane
            if (Mathf.Abs(i - currentLaneIndex) == 1)
            {
                var laneCentre = centreLanes[i];

                var laneChangeTrajectory = new Trajectory();
                laneChangeTrajectory.Lane = laneCentre;

                foreach (var state in lastTrajectoryStates)
                {
                    laneChangeTrajectory.states.Add(state);
                }

                laneChangeTrajectory.trajectoryStart = transitionStartState.t;

                var changingLaneTrajectory =
                    BuildQuinticTrajectory(laneChangeTrajectory,
                        laneCentre,
                        fromState,
                        vehicleController.GetManouvreDuration());
                var newStates = changingLaneTrajectory.states.Where(s => s.t > transitionStartState.t);
                foreach (var state in newStates)
                {
                    laneChangeTrajectory.states.Add(state);
                }

                TransitionTrajectories.Add(laneChangeTrajectory);
            }
        }

        return TransitionTrajectories;
    }

    public List<VehicleState> ObserveVehicle()
    {
        var newObservations = new List<VehicleState>(Observations);
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

        newObservations.RemoveAll(state => state.t < cutoffTime);

        // Append newest observation
        newObservations.Add(newState);
        return newObservations;
    }

    public Trajectory BuildLaneFollowTrajectory(Transform lane, VehicleState Observation)
    {
        Trajectory resultTrajectory = new Trajectory();

        resultTrajectory.Lane = lane;

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
    public Trajectory BuildLaneFollowTrajectory(Trajectory leadingTrajectory, Transform Lane, VehicleState Observation)
    {
        Trajectory resultTrajectory = new Trajectory();

        resultTrajectory.Lane = Lane;

        var lastLeadingState = leadingTrajectory.states[leadingTrajectory.states.Count - 1];
        var lastTrajectoryStates = leadingTrajectory.states.Where(s => s.t >= leadingTrajectory.trajectoryStart);
        foreach (var state in lastTrajectoryStates)
        {
            resultTrajectory.states.Add(state);
        }

        float dt = sampleRate;

        Vector3 pos = new Vector3(Lane.position.x, Observation.position.y, Observation.position.z);

        float velocity = Observation.velocity;
        float accel = Observation.acceleration;



        float heading = Observation.heading;




        resultTrajectory.trajectoryStart = Observation.t;
        float t = resultTrajectory.trajectoryStart;
        float trajectoryTime = 0f;
        //first point in the sequence, matching t = start

        // --- heading ---
        float dxdt1 = (Lane.position.x - pos.x) / dt;
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
            float targetX = Lane.position.x;

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

    public Trajectory BuildQuinticTrajectory(Transform Lane, VehicleState Observation, float duration)
    {
        Trajectory resultTrajectory = new Trajectory();

        resultTrajectory.Lane = Lane;

        float dt = sampleRate;

        Vector3 pos = Observation.position;

        float velocity = Observation.velocity;
        float accel = Observation.acceleration;

        float heading = Observation.heading;

        float t = Observation.t;
        resultTrajectory.trajectoryStart = t;

        float trajectoryTime = 0f;

        float startX = Observation.position.x;
        float targetX = Lane.position.x;
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

    public Trajectory BuildQuinticTrajectory(Trajectory leadingTrajectory, Transform Lane, VehicleState Observation, float duration)
    {
        Trajectory resultTrajectory = new Trajectory();

        resultTrajectory.Lane = Lane;

        var lastLeadingState = leadingTrajectory.states[leadingTrajectory.states.Count - 1];
        var lastTrajectoryStates = leadingTrajectory.states.Where(s => s.t >= leadingTrajectory.trajectoryStart);
        foreach (var state in lastTrajectoryStates)
        {
            resultTrajectory.states.Add(state);
        }

        float dt = sampleRate;

        Vector3 pos = lastLeadingState.position;

        float velocity = Observation.velocity;
        float accel = Observation.acceleration;

        float heading = Observation.heading;

        float t = lastLeadingState.t;
        resultTrajectory.trajectoryStart = lastLeadingState.t;

        float trajectoryTime = 0f;

        float startX = lastLeadingState.position.x;
        float targetX = Lane.position.x;
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

        int mainCount = currentTrajectory.states.Count;

        int followCount =
            currentTrajectory.followingTrajectory != null
                ? currentTrajectory.followingTrajectory.states.Count
                : 0;

        lineRenderer.positionCount = mainCount + followCount;

        // --- main trajectory ---
        for (int i = 0; i < mainCount; i++)
        {
            lineRenderer.SetPosition(i, currentTrajectory.states[i].position);
        }

        // --- following trajectory ---
        if (currentTrajectory.followingTrajectory != null)
        {
            var followStates = currentTrajectory.followingTrajectory.states;

            for (int i = 0; i < followCount; i++)
            {
                lineRenderer.SetPosition(mainCount + i, followStates[i].position);
            }
        }
    }
}
