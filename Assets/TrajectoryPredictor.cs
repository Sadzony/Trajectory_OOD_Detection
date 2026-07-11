using UnityEngine;
using System.Collections.Generic;
using System.Linq;

public class VehicleState
{
    public float t;

    public Vector3 position;   // x, y, z
    public float heading;      // radians

    public float velocity;
    public float acceleration;

    public double? recordedCusum = null;
}

public enum PredictionMode
{
    ADE,
    FDE,
}
public class Trajectory
{
    public Trajectory() { }
    public Trajectory(Trajectory copy) { trajectoryStart = copy.trajectoryStart; LaneFrom = copy.LaneFrom; LaneTo = copy.LaneTo; states = new List<VehicleState>(copy.states); followingTrajectory = copy.followingTrajectory; }
    public float trajectoryStart = 0.0f;
    public Transform LaneFrom;
    public Transform LaneTo;
    public List<VehicleState> states = new();
    public Trajectory? followingTrajectory = null;
    public bool isLaneSettle = false;
}

public class TrajectoryRecord
{
    public Trajectory trajectory { get; set; }
    public double error { get; set; }
    public List<VehicleState> observations { get; set; }

    public int trajectoryAnchorIndex;
}

public class TrajectoryPredictor : MonoBehaviour
{
    [Header("References")]
    [SerializeField] private LaneDefinedMovementQuintic vehicleController;
    [SerializeField] private List<Transform> centreLanes;

    [Header("Prediction")]
    private float sampleRate = 0.02f;
    [SerializeField] PredictionMode predictionMode;

    [Header("CUSUM")]
    [SerializeField] double cusumNoiseAlignment = 0.2;
    [SerializeField] double cumulativeErrorSum = 0.0;
    [SerializeField] double OODThreshold = 2.0;
    [SerializeField] bool simulateCUSUMOnStateChange = false;
    [Header("Debug")]
    [SerializeField] private LineRenderer lineRenderer;

    public Trajectory currentTrajectory;

    private float simulationTime;

    

    public List<VehicleState> Observations = new List<VehicleState>();

    public List<Trajectory> TransitionTrajectories = new List<Trajectory>();

    float biggestError = 0.0f;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        simulationTime = 0.0f;
        Observations = ObserveVehicle();
        var latestObservation = Observations[Observations.Count - 1];
        cumulativeErrorSum = 0.0f;
        latestObservation.recordedCusum = cumulativeErrorSum;
        currentTrajectory = BuildLaneFollowTrajectory(centreLanes[0], latestObservation);
        TransitionTrajectories = FindTransitionTrajectories(latestObservation);


        
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        simulationTime += Time.fixedDeltaTime;
        Observations = ObserveVehicle();

        var latestObservation = Observations[Observations.Count - 1];

        bool transitions = true;
        if (latestObservation.velocity < 0.1)
        {
            transitions = false;
        }

        //generates a follow up trajectory if observations exceed current trajectory
        currentTrajectory = UpdateTrajectory(currentTrajectory, latestObservation);

        //1. Update the set of transition Trajectories based on observed state
        if(transitions && !currentTrajectory.isLaneSettle)
            TransitionTrajectories = FindTransitionTrajectories(latestObservation);

        int currentTrajectoryAnchorIndex = FindClosestIndex(currentTrajectory.states, Observations[^1].t);
        int currentTrajectoryMinIndex = FindClosestIndexLowest(currentTrajectory.states, Mathf.Max(0, Observations[^1].t - vehicleController.GetManouvreDuration()));
        double currentTrajectoryError = FindErrorMeasure(currentTrajectory, currentTrajectoryAnchorIndex, currentTrajectoryMinIndex, Observations);

        cumulativeErrorSum += currentTrajectoryError;
        cumulativeErrorSum = System.Math.Max(0.0, cumulativeErrorSum - cusumNoiseAlignment);

        latestObservation.recordedCusum = cumulativeErrorSum;

        if (currentTrajectoryError > biggestError)
        {
            Debug.Log(currentTrajectoryError);
            biggestError = (float)currentTrajectoryError;
        }
        if (currentTrajectoryError > 0 && transitions && !currentTrajectory.isLaneSettle)
        {
            
            //2 Transition to a different, or stay on current trajectory, based on error measure
            currentTrajectory = SelectAlikeTrajectory(currentTrajectoryError, currentTrajectoryAnchorIndex, TransitionTrajectories);
        }

        RenderTrajectory(currentTrajectory);
    }

    private Trajectory UpdateTrajectory(Trajectory trajectory, VehicleState Observation)
    {
        var resultTrajectory = new Trajectory(trajectory);

        float dt = sampleRate;

        //generate a following trajectory
        if(trajectory.followingTrajectory == null)
        {
            resultTrajectory.followingTrajectory = BuildLaneFollowTrajectory(trajectory, trajectory.LaneTo, trajectory.states[^1]);
        }
        if(Observation.position.z > trajectory.states[^1].position.z)
        {
            if (trajectory.LaneFrom == trajectory.LaneTo)
            {
                resultTrajectory.followingTrajectory = BuildLaneFollowTrajectory(trajectory, trajectory.LaneTo, trajectory.states[^1]);
            }
            else
            {
                resultTrajectory.followingTrajectory = BuildLaneSettleTrajectory(trajectory, trajectory.LaneTo, Observation);
            }
            resultTrajectory = resultTrajectory.followingTrajectory;
            resultTrajectory.followingTrajectory = BuildLaneFollowTrajectory(resultTrajectory, resultTrajectory.LaneTo, resultTrajectory.states[^1]);
        }

        return resultTrajectory;
    }

    private double FindErrorMeasure(Trajectory trajectory, int trajectoryAnchorIndex, int minIndex, List<VehicleState> Observations)
    {
        VehicleState latest = Observations[^1];

        float dt = sampleRate;

        float tLatest = Observations[^1].t;
        int obsCount = Observations.Count;

        //error measure works off of the latest observation and works its way down the observation list
        

        if (predictionMode == PredictionMode.ADE)
        {
            if (trajectory.states.Count == 0)
                return double.PositiveInfinity;

            double totalError = 0f;
            int count = 0;
            float halfLength = vehicleController.GetVehicleLength() / 2;

            for (int i = obsCount - 1; i >= 0; i--)
            {
                int trajIndex = trajectoryAnchorIndex - (obsCount - 1 - i);

                if (trajIndex < minIndex)
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
                double frontError = Vector3.Distance(obsFront, predFront);
                double rearError = Vector3.Distance(obsRear, predRear);

                double error = (frontError + rearError) * 0.5;

                totalError += error;
                count++;
            }

            double ade = count > 0
                ? totalError / count
                : double.PositiveInfinity;

            return ade;
        }
        else
        {
            return double.PositiveInfinity;
        }
    }

    private Trajectory SelectAlikeTrajectory(double currentTrajectoryError, int currentTrajectoryAnchorIndex, List<Trajectory> transitionTrajectories)
    {
        VehicleState latest = Observations[^1];

        float dt = sampleRate;
        

        var potentialTrajectories = new List<Trajectory>(transitionTrajectories);
        Trajectory bestTrajectory = currentTrajectory;
        double bestError = currentTrajectoryError;
        float tLatest = Observations[^1].t;
        int obsCount = Observations.Count;

        bool runCusumSim = false;

        var topRecords = new List<TrajectoryRecord>();
        var currentTrajectoryRecord = new TrajectoryRecord
        {
            trajectory = currentTrajectory,
            error = bestError,
            observations = new List<VehicleState>(Observations),
            trajectoryAnchorIndex = currentTrajectoryAnchorIndex
        };

        if (predictionMode == PredictionMode.ADE)
        {
            foreach (Trajectory trajectory in potentialTrajectories)
            {
                if (trajectory.states.Count == 0)
                    continue;

                int anchorIndex = FindClosestIndex(trajectory.states, tLatest);
                int minIndex = FindClosestIndexLowest(trajectory.states, Mathf.Max(0, tLatest - vehicleController.GetManouvreDuration()));

                double totalError = 0f;
                int count = 0;
                float halfLength = vehicleController.GetVehicleLength() / 2;

                for (int i = obsCount - 1; i >= 0; i--)
                {
                    int trajIndex = anchorIndex - (obsCount - 1 - i);

                    if (trajIndex < minIndex)
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
                    double frontError = Vector3.Distance(obsFront, predFront);
                    double rearError = Vector3.Distance(obsRear, predRear);

                    double error = (frontError + rearError) * 0.5;

                    totalError += error;
                    count++;
                }

                double ade = count > 0
                    ? totalError / count
                    : float.PositiveInfinity;

                // Track top 3 matches
                topRecords.Add(new TrajectoryRecord
                {
                    trajectory = trajectory,
                    error = ade,
                    observations = new List<VehicleState>(Observations),
                    trajectoryAnchorIndex = anchorIndex,
                });

                topRecords.Sort((a, b) => a.error.CompareTo(b.error));

                if (topRecords.Count > 3)
                    topRecords.RemoveAt(3);

                
                if (ade < bestError)
                {
                    runCusumSim = true;
                } 
            }
        }
        if (runCusumSim)
        {
            if (simulateCUSUMOnStateChange)
            {
                //run the sim on the current record too.
                topRecords.Add(currentTrajectoryRecord);

                double bestFinalCusum = double.PositiveInfinity;
                TrajectoryRecord? bestRecord = currentTrajectoryRecord;

                foreach (var record in topRecords)
                {
                    // Find where this trajectory begins inside the observation history.
                    int startObservationIndex = record.observations.FindIndex(o =>
                        Mathf.Abs(o.t - record.trajectory.trajectoryStart) < sampleRate * 0.5f);

                    if (startObservationIndex < 0)
                        startObservationIndex = 0;

                    // Clone observations so we don't overwrite the real history yet.
                    List<VehicleState> simulatedObservations = record.observations
                        .Select(o => new VehicleState
                        {
                            t = o.t,
                            position = o.position,
                            heading = o.heading,
                            velocity = o.velocity,
                            acceleration = o.acceleration,
                            recordedCusum = o.recordedCusum
                        })
                        .ToList();

                    double cusum =
                        simulatedObservations[startObservationIndex].recordedCusum ?? 0.0;

                    int anchorIndex = record.trajectoryAnchorIndex;

                    for (int obsIndex = simulatedObservations.Count - 1;
                         obsIndex >= startObservationIndex;
                         obsIndex--)
                    {
                        VehicleState obs = simulatedObservations[obsIndex];

                        int minIndex = FindClosestIndexLowest(
                            record.trajectory.states,
                            Mathf.Max(0f, obs.t - vehicleController.GetManouvreDuration()));

                        double error = FindErrorMeasure(
                            record.trajectory,
                            anchorIndex,
                            minIndex,
                            simulatedObservations.GetRange(startObservationIndex,
                                obsIndex - startObservationIndex + 1));

                        cusum += error;
                        cusum = System.Math.Max(0.0, cusum - cusumNoiseAlignment);

                        obs.recordedCusum = cusum;

                        // Move trajectory anchor backwards one sample because
                        // we're replaying backwards through time.
                        anchorIndex--;
                        if (anchorIndex < 0)
                            break;
                    }

                    if (cusum < bestFinalCusum)
                    {
                        bestFinalCusum = cusum;

                        bestRecord = new TrajectoryRecord
                        {
                            trajectory = record.trajectory,
                            error = record.error,
                            trajectoryAnchorIndex = record.trajectoryAnchorIndex,
                            observations = simulatedObservations
                        };
                    }
                }

                if (bestRecord != null)
                {
                    Observations = bestRecord.observations;
                    cumulativeErrorSum = bestFinalCusum;
                    bestTrajectory = bestRecord.trajectory;
                }
            }
            else
            {
                var bestRecord = currentTrajectoryRecord;
                var bestRecordError = currentTrajectoryRecord.error;
                foreach (var record in topRecords)
                {
                    if(record.error < bestRecordError)
                    {
                        bestRecord = record;
                        bestRecordError = record.error;
                    }
                }
                bestTrajectory = bestRecord.trajectory;
            }
        }
        

        if (bestTrajectory.followingTrajectory == null)
        {
            bestTrajectory.followingTrajectory = BuildLaneFollowTrajectory(bestTrajectory, bestTrajectory.LaneTo, bestTrajectory.states[^1]);
        }
        return bestTrajectory;
    }

    public List<Trajectory> FindTransitionTrajectories(VehicleState fromState)
    {
        float dt = sampleRate;
        //Filter out old trajectories
        float cutoffTime = fromState.t - (vehicleController.GetManouvreDuration());
        TransitionTrajectories.RemoveAll(t => t.trajectoryStart < cutoffTime);

        var tolerance = sampleRate * 0.25f;




        List<VehicleState> lastTrajectoryStates;
        if (currentTrajectory.LaneFrom == currentTrajectory.LaneTo)
        {
            lastTrajectoryStates = currentTrajectory.states.Where(s =>
                                                                s.t >= fromState.t - vehicleController.GetManouvreDuration() && 
                                                                s.position.z < fromState.position.z && s.t < fromState.t)
                                                                .ToList();
        }
        else
        {
            lastTrajectoryStates = currentTrajectory.states.Where(s => s.t < currentTrajectory.trajectoryStart).ToList();
        }
        
        if(currentTrajectory.LaneFrom == currentTrajectory.LaneTo)
        {
            
            Transform currentLane = currentTrajectory.LaneTo;
            //create a lane change trajectory, for every neighbouring lane
            int currentLaneIndex = centreLanes.IndexOf(currentLane);

            //create an updated trajectory for the same lane
            var observationUpdateTrajectory = new Trajectory();

            observationUpdateTrajectory.LaneFrom = currentTrajectory.LaneFrom;
            observationUpdateTrajectory.LaneTo = currentTrajectory.LaneTo;
            foreach (var state in lastTrajectoryStates)
            {
                observationUpdateTrajectory.states.Add(state);
            }

            var updatedTraj = BuildLaneFollowTrajectory(observationUpdateTrajectory.LaneTo, fromState);
            foreach (var state in updatedTraj.states)
            {
                observationUpdateTrajectory.states.Add(state);
            }
            observationUpdateTrajectory.trajectoryStart = updatedTraj.trajectoryStart;
            TransitionTrajectories.Add(observationUpdateTrajectory); 


            for (int i = 0; i < centreLanes.Count; i++)
            {
                //act only on lanes which are 1 index away from this lane
                if (Mathf.Abs(i - currentLaneIndex) == 1)
                {
                    var laneCentre = centreLanes[i];

                    var laneChangeTrajectory = new Trajectory();
                    laneChangeTrajectory.LaneFrom = currentTrajectory.LaneTo;
                    laneChangeTrajectory.LaneTo = laneCentre;


                    foreach (var state in lastTrajectoryStates)
                    {
                        laneChangeTrajectory.states.Add(state);
                    }

                    laneChangeTrajectory.trajectoryStart = fromState.t;

                    var changingLaneTrajectory =
                        BuildLaneChangeTrajectory(laneChangeTrajectory.LaneFrom,
                            laneChangeTrajectory.LaneTo,
                            fromState);
                    foreach (var state in changingLaneTrajectory.states)
                    {
                        laneChangeTrajectory.states.Add(state);
                    }

                    TransitionTrajectories.Add(laneChangeTrajectory);
                }
            }
        }
        
        else
        {
            //create an updated LaneChange trajectory
            Trajectory? updatedTrajResult = UpdateLaneChangeTrajectory(
                    currentTrajectory,
                    currentTrajectory.LaneFrom,
                    currentTrajectory.LaneTo,
                    fromState);
            if (updatedTrajResult != null)
            {
                var updatedTraj = new Trajectory();
                updatedTraj.LaneFrom = updatedTrajResult.LaneFrom;
                updatedTraj.LaneTo = updatedTrajResult.LaneTo;
                foreach (var state in lastTrajectoryStates)
                {
                    updatedTraj.states.Add(state);
                }
                foreach (var state in updatedTrajResult.states)
                {
                    updatedTraj.states.Add(state);
                }

                updatedTraj.followingTrajectory = null;
                updatedTraj.trajectoryStart = updatedTrajResult.trajectoryStart;
                TransitionTrajectories.Add(updatedTraj);
            }
        }

        //append updated LaneFollow trajectories for every other lane, without history
        /*
        for (int i = 0; i < centreLanes.Count; i++)
        {
            if (currentLaneIndex != i)
            {
                var laneCentre = centreLanes[i];

                var followLaneTrajectory =
                    BuildLaneFollowTrajectory(laneCentre,
                        fromState);
                TransitionTrajectories.Add(followLaneTrajectory);
            }
        }*/

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
            position = new Vector3(vehicleController.transform.position.x, 0, vehicleController.transform.position.z),
        };
        // filter the observation buffer: remove all entries where VehicleState.t < simulationTime - ManouvreDuration
        // Remove observations older than the manoeuvre duration
        float cutoffTime = simulationTime - (2*vehicleController.GetManouvreDuration());

        newObservations.RemoveAll(state => state.t < cutoffTime);

        // Append newest observation
        newObservations.Add(newState);
        return newObservations;
    }

    public Trajectory BuildLaneFollowTrajectory(Transform Lane, VehicleState Observation)
    {
        Trajectory resultTrajectory = new Trajectory();

        resultTrajectory.LaneFrom = Lane;
        resultTrajectory.LaneTo = Lane;

        float dt = sampleRate;

        Vector3 pos = new Vector3(Lane.position.x, 0, Observation.position.z);

        float velocity;
        velocity = Mathf.Max(Observation.velocity, vehicleController.GetSpeedLimitMin());
        //velocity = Mathf.Min(velocity, vehicleController.GetSpeedLimitMax());

        float accel = Observation.acceleration;



        float heading = Observation.heading;

        float t = Observation.t;
        resultTrajectory.trajectoryStart = t;
        float trajectoryTime = 0f;

        //first point in the sequence, matching t = 0

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
            if (velocity + (accel * dt) < vehicleController.GetSpeedLimitMin())
                velocity = vehicleController.GetSpeedLimitMin();
            else velocity = velocity + (accel * dt);
            //velocity = Mathf.Min(velocity, vehicleController.GetSpeedLimitMax());

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
    public Trajectory BuildLaneFollowTrajectory(Trajectory leadingTrajectory, Transform Lane, VehicleState Observation)
    {
        Trajectory resultTrajectory = new Trajectory();

        resultTrajectory.LaneFrom = Lane;
        resultTrajectory.LaneTo = Lane;

        var lastLeadingState = leadingTrajectory.states[leadingTrajectory.states.Count - 1];
        var lastTrajectoryStates = leadingTrajectory.states.Where(s => s.t >= leadingTrajectory.trajectoryStart && s.t + (sampleRate*0.5f) < Observation.t).ToList();
        foreach (var state in lastTrajectoryStates)
        {
            resultTrajectory.states.Add(state);
        }

        float dt = sampleRate;

        Vector3 pos = new Vector3(Lane.position.x, 0, Observation.position.z);

        float velocity;
        velocity = Mathf.Max(Observation.velocity, vehicleController.GetSpeedLimitMin());
        //velocity = Mathf.Min(velocity, vehicleController.GetSpeedLimitMax());
        float accel = Observation.acceleration;



        float heading = Observation.heading;




        resultTrajectory.trajectoryStart = Observation.t;
        float t = resultTrajectory.trajectoryStart;
        float trajectoryTime = 0f;
        //first point in the sequence, matching t = start

        // --- heading ---
        float dxdt1 = (Lane.position.x - pos.x) / dt;
        float dzdt1 = Mathf.Max(velocity, 0.5f);
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
            if (velocity + (accel * dt) < vehicleController.GetSpeedLimitMin())
                velocity = vehicleController.GetSpeedLimitMin();
            else velocity = velocity + (accel * dt);
            //velocity = Mathf.Min(velocity, vehicleController.GetSpeedLimitMax());

            float dz = velocity * dt;

            // Following the lane at a constant lateral position
            float targetX = Lane.position.x;

            pos.x = targetX;
            pos.z += dz;

            // --- heading ---
            float dxdt = (targetX - pos.x) / dt;
            float dzdt = Mathf.Max(velocity, 0.5f);

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

    public Trajectory BuildLaneChangeTrajectory(Transform LaneFrom, Transform LaneTo, VehicleState Observation)
    {
        Trajectory resultTrajectory = new Trajectory();

        resultTrajectory.LaneFrom = LaneFrom;
        resultTrajectory.LaneTo = LaneTo;

        float dt = sampleRate;

        Vector3 pos = new Vector3(LaneFrom.position.x, 0, Observation.position.z);

        float velocity;
        velocity = Mathf.Max(Observation.velocity, vehicleController.GetSpeedLimitMin());
        //velocity = Mathf.Min(velocity, vehicleController.GetSpeedLimitMax());
        float accel = Observation.acceleration;

        float heading = Observation.heading;

        float t = Observation.t;
        resultTrajectory.trajectoryStart = t;

        float trajectoryTime = 0f;

        float startX = LaneFrom.position.x;
        float targetX = LaneTo.position.x;
        float deltaX = targetX - startX;

        // First point (matches current observation)
        float u = 0f;

        float ds =
            30f * u * u
            - 60f * u * u * u
            + 30f * u * u * u * u;

        var duration = vehicleController.GetManouvreDuration();

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
            if (velocity + (accel * dt) < vehicleController.GetSpeedLimitMin())
                velocity = vehicleController.GetSpeedLimitMin();
            else velocity = velocity + (accel * dt);
            //velocity = Mathf.Min(velocity, vehicleController.GetSpeedLimitMax());

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

    public Trajectory? UpdateLaneChangeTrajectory(Trajectory updateTrajectory, Transform LaneFrom, Transform LaneTo, VehicleState Observation)
    {
        Trajectory resultTrajectory = new Trajectory();

        resultTrajectory.LaneFrom = LaneFrom;
        resultTrajectory.LaneTo = LaneTo;

        var updateTrajectoryStates = updateTrajectory.states.Where(s => s.t < Observation.t && s.t >= updateTrajectory.trajectoryStart).ToList();

        foreach (var state in updateTrajectoryStates)
        {
            resultTrajectory.states.Add(state);
        }

        var tolerance = sampleRate * 0.5f;
        var updateStartStateResult = updateTrajectory.states.Find(s => Mathf.Abs(s.t - Observation.t) < tolerance);
        VehicleState updateStartState;
        if (updateStartStateResult == null)
            return null;
        else updateStartState = updateStartStateResult;

        var timeWithinCurrent = updateStartState.t - updateTrajectory.trajectoryStart;
        var differenceToManouvreDuration = vehicleController.GetManouvreDuration() - timeWithinCurrent;

        float dt = sampleRate;

        Vector3 pos = updateStartState.position;

        float velocity;
        velocity = Mathf.Max(Observation.velocity, vehicleController.GetSpeedLimitMin());
        //velocity = Mathf.Min(velocity, vehicleController.GetSpeedLimitMax());


        float accel = Observation.acceleration;

        float heading = Observation.heading;

        float t = updateStartState.t;
        resultTrajectory.trajectoryStart = updateTrajectory.trajectoryStart;

        float trajectoryTime = timeWithinCurrent;

        float startX = LaneFrom.position.x;
        float targetX = LaneTo.position.x;
        float deltaX = targetX - startX;

        var duration = vehicleController.GetManouvreDuration();

        // First point (matches current observation)
        float u = timeWithinCurrent / duration;

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
            if (velocity + (accel * dt) < vehicleController.GetSpeedLimitMin())
                velocity = vehicleController.GetSpeedLimitMin();
            else velocity = velocity + (accel * dt);
            //velocity = Mathf.Min(velocity, vehicleController.GetSpeedLimitMax());

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

    public Trajectory BuildLaneSettleTrajectory(Trajectory leadingTrajectory, Transform Lane, VehicleState Observation)
    {
        Trajectory resultTrajectory = new Trajectory();

        resultTrajectory.LaneFrom = Lane;
        resultTrajectory.LaneTo = Lane;
        resultTrajectory.isLaneSettle = true;

        // Copy history from the leading trajectory
        var lastTrajectoryStates = leadingTrajectory.states
            .Where(s => s.t >= leadingTrajectory.trajectoryStart &&
                        s.t + (sampleRate * 0.5f) < Observation.t)
            .ToList();

        foreach (var state in lastTrajectoryStates)
        {
            resultTrajectory.states.Add(state);
        }

        float dt = sampleRate;

        Vector3 pos = Observation.position;

        float velocity = Mathf.Max(
            Observation.velocity,
            vehicleController.GetSpeedLimitMin());

        float accel = Observation.acceleration;
        float heading = Observation.heading;

        float t = Observation.t;
        resultTrajectory.trajectoryStart = t;

        float trajectoryTime = 0f;
        float duration = 0.5f;

        float startX = Observation.position.x;
        float targetX = Lane.position.x;
        float deltaX = targetX - startX;

        // First point
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
            // Longitudinal motion
            if (velocity + accel * dt < vehicleController.GetSpeedLimitMin())
                velocity = vehicleController.GetSpeedLimitMin();
            else
                velocity += accel * dt;

            pos.z += velocity * dt;

            // Quintic interpolation
            u = Mathf.Clamp01(trajectoryTime / duration);

            float s =
                10f * u * u * u
                - 15f * u * u * u * u
                + 6f * u * u * u * u * u;

            pos.x = startX + deltaX * s;

            ds =
                30f * u * u
                - 60f * u * u * u
                + 30f * u * u * u * u;

            dxdt = (deltaX * ds) / duration;
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

        // Continue with normal lane following
        resultTrajectory.followingTrajectory =
            BuildLaneFollowTrajectory(resultTrajectory, Lane, resultTrajectory.states[^1]);

        return resultTrajectory;
    }

    private void RenderTrajectory(Trajectory renderTrajectory)
    {
        if (lineRenderer == null || renderTrajectory == null)
            return;

        int mainCount = renderTrajectory.states.Count;

        int followCount = 0;
        var followingTrajectoryStates = new List<VehicleState>();
        if (renderTrajectory.followingTrajectory != null)
        {
            //find the states after trajectory start
            followingTrajectoryStates = renderTrajectory.followingTrajectory.states.Where(s => s.t >= renderTrajectory.followingTrajectory.trajectoryStart).ToList();

            followCount = followingTrajectoryStates.Count;
        }

        lineRenderer.positionCount = mainCount + followCount;

        // --- main trajectory ---
        for (int i = 0; i < mainCount; i++)
        {
            lineRenderer.SetPosition(i, new Vector3(renderTrajectory.states[i].position.x , 1, renderTrajectory.states[i].position.z));
        }

        // --- following trajectory ---
        if (renderTrajectory.followingTrajectory != null)
        {
            var followStates = renderTrajectory.followingTrajectory.states;

            for (int i = 0; i < followCount; i++)
            {
                lineRenderer.SetPosition(mainCount + i, new Vector3(followingTrajectoryStates[i].position.x, 1, followingTrajectoryStates[i].position.z));
            }
        }
    }
    int FindClosestIndex(List<VehicleState> states, float t)
    {
        int bestIndex = states.Count - 1;
        float bestTimeDifference = Mathf.Abs(states[states.Count-1].t - t);

        for (int i = 0; i < states.Count; i++)
        {
            float timeDifference = Mathf.Abs(states[i].t - t);

            if (timeDifference < bestTimeDifference)
            {
                bestTimeDifference = timeDifference;
                bestIndex = i;
            }
            else if (states[i].t > t)
            {
                break;
            }
        }

        return bestIndex;
    }
    int FindClosestIndexLowest(List<VehicleState> states, float t)
    {
        int bestIndex = 0;
        float bestTimeDifference = Mathf.Abs(states[0].t - t);

        for (int i = 0; i < states.Count; i++)
        {
            float timeDifference = Mathf.Abs(states[i].t - t);

            if (timeDifference < bestTimeDifference)
            {
                bestTimeDifference = timeDifference;
                bestIndex = i;
            }
            else if (states[i].t > t)
            {
                break;
            }
        }

        return bestIndex;
    }

    private VehicleState FindClosestStateByTime(List<VehicleState> states, float targetTime)
    {
        int left = 0;
        int right = states.Count - 1;

        while (left <= right)
        {
            int mid = left + (right - left) / 2;

            float midTime = states[mid].t;

            if (Mathf.Approximately(midTime, targetTime))
            {
                return states[mid];
            }
            else if (midTime < targetTime)
            {
                left = mid + 1;
            }
            else
            {
                right = mid - 1;
            }
        }

        // left is the first state after targetTime
        // right is the last state before targetTime
        if (left >= states.Count)
            return states[^1];

        if (right < 0)
            return states[0];

        // Pick whichever timestamp is closest
        float leftDistance = Mathf.Abs(states[left].t - targetTime);
        float rightDistance = Mathf.Abs(states[right].t - targetTime);

        return leftDistance < rightDistance
            ? states[left]
            : states[right];
    }

}
