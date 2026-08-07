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
    LCSS,
    LCSSFinal,
}
public class Trajectory
{
    public Trajectory() { }
    public Trajectory(Trajectory copy) { trajectoryStart = copy.trajectoryStart; LaneFrom = copy.LaneFrom; LaneTo = copy.LaneTo; states = new List<VehicleState>(copy.states); followingTrajectory = copy.followingTrajectory; isLaneSettle = copy.isLaneSettle; }
    public float trajectoryStart = 0.0f;
    public Transform LaneFrom;
    public Transform LaneTo;
    public List<VehicleState> states = new();
    public Trajectory? followingTrajectory = null;
    public bool isLaneSettle = false;
}

public class TrajectoryPredictor : MonoBehaviour
{

    private class TrajectoryRecord
    {
        public Trajectory trajectory { get; set; }
        public double error { get; set; }
        public List<VehicleState> observations { get; set; }

        public int trajectoryAnchorIndex;
    }


    [SerializeField] FalseAlarmRateRecorder FARrecorder;
    [SerializeField] WADDRecorder WADDrecorder;
    public TMPro.TMP_InputField cusumValueField;
    public TMPro.TMP_InputField errorValueField;
    [Header("References")]
    [SerializeField] private LaneDefinedMovementQuintic vehicleController;
    [SerializeField] private List<Transform> centreLanes;

    [Header("Prediction")]
    private float sampleRate = 0.02f;
    [SerializeField] public PredictionMode predictionMode;

    [Header("CUSUM")]
    [SerializeField] public double cusumNoiseAlignmentEuclidean = 0.15;
    //[SerializeField] public double cusumNoiseAlignmentEuclideanWhenNoisy = 0.05;
    [SerializeField] public double cusumNoiseAlignmentLCSS = 0.015;
    //[SerializeField] public double cusumNoiseAlignmentLCSSWhenNoisy = 0.005;
    [SerializeField] double cumulativeErrorSum = 0.0;
    [SerializeField] public double OODThresholdEuclidean = 2.0;
    [SerializeField] public double OODThresholdLCSS = 0.2;
    [SerializeField] public float lcssAcceptanceMagnitude = 0.15f;
    [SerializeField] bool simulateCUSUMOnStateChange = false;
    [Header("Debug")]
    [SerializeField] private LineRenderer lineRenderer;
    [SerializeField] private Material inDistributionLine;
    [SerializeField] private Material oodLine;

    [Header("Observation Noise")]
    [SerializeField] public bool addObservationNoise = true;
    [SerializeField] private float changeNoiseDirectionPeriodMinimum = 0.0f;
    [SerializeField] private float changeNoiseDirectionPeriodMaximum = 1.5f;
    [SerializeField] private float maximumPositionNoiseMagnitude = 0.035f;
    [SerializeField] private float maximumVelocityNoiseMagnitude = 0.25f;
    [SerializeField] private float maximumAccelerationNoiseMagnitude = 0.05f;
    [SerializeField] private float maximumHeadingNoiseMagnitude = 1.0f;
    [Header("Observation Anomalies")]
    [SerializeField] public bool addObservationAnomalies = true;
    [SerializeField] private float anomalyMinimumPositionMagnitude = 0.05f;
    [SerializeField] private float anomalyMaximumPositionMagnitude = 0.06f;
    [SerializeField] private float anomalyMaximumVelocityMagnitude = 0.02f;
    [SerializeField] private float anomalyMaximumAccelerationMagnitude = 0.05f;
    [SerializeField] private float anomalyMaximumHeadingMagnitude = 3.5f;
    [SerializeField] private float anomalyOccurancePeriodMinimum = 4.0f;
    [SerializeField] private float anomalyOccurancePeriodMaximum = 7.5f;

    private float nextChangeNoiseDirectionTime;
    private float nextAnomalyTime;
    private Vector2 positionNoiseDirection;
    private int headingNoiseDirection;
    private int speedNoiseDirection;

    public Trajectory currentTrajectory;

    private float simulationTime;

    

    public List<VehicleState> Observations = new List<VehicleState>();

    public List<Trajectory> TransitionTrajectories = new List<Trajectory>();
    public List<Trajectory> OODTransitionTrajectories = new List<Trajectory>();

    float biggestError = 0.0f;

    public bool ood = false;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        lineRenderer.material = inDistributionLine;
        simulationTime = 0.0f;
        Observations = ObserveVehicle();
        var latestObservation = Observations[Observations.Count - 1];
        cumulativeErrorSum = 0.0f;
        latestObservation.recordedCusum = cumulativeErrorSum;
        currentTrajectory = BuildLaneFollowTrajectory(centreLanes[0], latestObservation);
        TransitionTrajectories = FindTransitionTrajectories(latestObservation);

        nextChangeNoiseDirectionTime = Random.Range(changeNoiseDirectionPeriodMinimum, changeNoiseDirectionPeriodMinimum);
        nextAnomalyTime = Random.Range(anomalyOccurancePeriodMinimum, anomalyOccurancePeriodMaximum);
        RegenerateNoiseDirections();

    }

    public double GetLatestError() => latestError;
    double latestError = 0.0;
    // Update is called once per frame
    void FixedUpdate()
    {
        simulationTime += Time.fixedDeltaTime;
        FARrecorder.UpdateSimulationTime(simulationTime);
        WADDrecorder.UpdateSimulationTime(simulationTime);

        Observations = ObserveVehicle();

        var latestObservation = Observations[Observations.Count - 1];

        if (!ood)
        {

            bool transitions = true;
            if (latestObservation.velocity < 0.1)
            {
                transitions = false;
            }

            //generates a follow up trajectory if observations exceed current trajectory
            currentTrajectory = UpdateTrajectory(currentTrajectory, latestObservation);

            //1. Update the set of transition Trajectories based on observed state
            if (transitions)
                TransitionTrajectories = FindTransitionTrajectories(latestObservation);

            int currentTrajectoryAnchorIndex = FindClosestIndex(currentTrajectory.states, Observations[^1].t);
            int currentTrajectoryMinIndex = FindClosestIndexLowest(currentTrajectory.states, Mathf.Max(0, Observations[^1].t - vehicleController.GetManouvreDuration()));
            double currentTrajectoryError = FindErrorMeasure(currentTrajectory, currentTrajectoryAnchorIndex, currentTrajectoryMinIndex, Observations);

            cumulativeErrorSum += currentTrajectoryError;
            if(predictionMode == PredictionMode.ADE || predictionMode == PredictionMode.FDE)
                cumulativeErrorSum = System.Math.Max(0.0, addObservationNoise ? cumulativeErrorSum - cusumNoiseAlignmentEuclidean : cumulativeErrorSum - cusumNoiseAlignmentEuclidean);
            else if (predictionMode == PredictionMode.LCSS || predictionMode == PredictionMode.LCSSFinal)
                cumulativeErrorSum = System.Math.Max(0.0, addObservationNoise ? cumulativeErrorSum - cusumNoiseAlignmentLCSS : cumulativeErrorSum - cusumNoiseAlignmentLCSS);


            latestObservation.recordedCusum = cumulativeErrorSum;

            if (currentTrajectoryError > biggestError)
            {
                Debug.Log(currentTrajectoryError);
                biggestError = (float)currentTrajectoryError;
            }
            if (currentTrajectoryError > 0 && transitions)
            {

                //2 Transition to a different, or stay on current trajectory, based on error measure
                currentTrajectory = SelectAlikeTrajectory(currentTrajectoryError, currentTrajectoryAnchorIndex, TransitionTrajectories);
            }
            currentTrajectoryError = FindErrorMeasure(currentTrajectory, currentTrajectoryAnchorIndex, currentTrajectoryMinIndex, Observations);
            errorValueField.text = currentTrajectoryError.ToString("F5");
            latestError = currentTrajectoryError;

            if (Mathf.Approximately((float)cumulativeErrorSum, 0.0f))
            {
                WADDrecorder.UpdateLastStableCUSUM();
            }

            //enage ood
            if (((predictionMode == PredictionMode.ADE || predictionMode == PredictionMode.FDE) && cumulativeErrorSum > OODThresholdEuclidean) ||
                ((predictionMode == PredictionMode.LCSS || predictionMode == PredictionMode.LCSSFinal) && cumulativeErrorSum > OODThresholdLCSS))
            {
                Debug.Log("OOD Engaged at value: " + cumulativeErrorSum + " and Error: " + currentTrajectoryError);
                FARrecorder.RecordAlarmTrigger();
                WADDrecorder.RecordAlarmTrigger();
                

                //generate first ctra traj
                currentTrajectory = GenerateCTRATrajectory();
                //clear transition trajectories
                TransitionTrajectories.Clear();
                //change color of line renderer
                lineRenderer.material = oodLine;
                //clear CUSUM
                foreach (var obs in Observations)
                    obs.recordedCusum = null;
                //fill out the first set of the trajectories to check if we get back in-distribution.
                OODTransitionTrajectories = FindOODTransitionTrajectories(latestObservation);


                cumulativeErrorSum = 0.0;

                ood = true;
            }
        }
        else
        {


            //check if we exit OOD:
            //1 compare error measure between observations and the trajectories in OODTransitionTrajectories
            //2 resimulate cusum on the lowest one,

            currentTrajectory = GenerateCTRATrajectory();

            OODTransitionTrajectories =
                FindOODTransitionTrajectories(latestObservation);


            // Step 1: choose lowest ADE trajectory
            Trajectory bestTrajectory =
                SelectBestOODTrajectory(
                    OODTransitionTrajectories);



            if (bestTrajectory != null)
            {
                // Step 2: only simulate CUSUM on this one
                var simulation =
                    SimulateOODCusum(
                        bestTrajectory);


                double simulatedCusum =
                    simulation.cusum;



                // Step 3: confirm recovery
                if (((predictionMode == PredictionMode.ADE || predictionMode == PredictionMode.FDE) && simulatedCusum < OODThresholdEuclidean) ||
                    ((predictionMode == PredictionMode.LCSS || predictionMode == PredictionMode.LCSSFinal) && simulatedCusum < OODThresholdLCSS))
                {
                    Observations =
                        simulation.observations;


                    cumulativeErrorSum =
                        simulatedCusum;

                    Debug.Log("Back In Distribution at cusum value: " + cumulativeErrorSum);


                    currentTrajectory =
                        bestTrajectory;


                    currentTrajectory =
                        UpdateTrajectory(
                            currentTrajectory,
                            latestObservation);


                    TransitionTrajectories =
                        FindTransitionTrajectories(
                            latestObservation);

                    OODTransitionTrajectories.Clear();

                    ood = false;

                    lineRenderer.material =
                        inDistributionLine;
                }


            }
        }

        RenderTrajectory(currentTrajectory);
        cusumValueField.text = cumulativeErrorSum.ToString("F5");
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
        float halfLength = vehicleController.GetVehicleLength() / 2;
        //error measure works off of the latest observation and works its way down the observation list


        if (predictionMode == PredictionMode.ADE)
        {
            if (trajectory.states.Count == 0)
                return double.PositiveInfinity;

            double totalError = 0f;
            int count = 0;
            

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
        else if(predictionMode == PredictionMode.FDE)
        {
            if (trajectory.states.Count == 0)
                return double.PositiveInfinity;

            VehicleState obs = Observations[^1];
            VehicleState pred = trajectory.states[trajectoryAnchorIndex];

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

            return error;
        }
        else if(predictionMode == PredictionMode.LCSS)
        {
            if (trajectory.states.Count == 0)
                return 1.0f;

            int frontMatches = 0;
            int rearMatches = 0;
            int count = 0;


            for (int i = obsCount - 1; i >= 0; i--)
            {
                int trajIndex =
                    trajectoryAnchorIndex - (obsCount - 1 - i);


                if (trajIndex < minIndex)
                    break;


                VehicleState obs =
                    Observations[i];

                VehicleState pred =
                    trajectory.states[trajIndex];


                // --- forward vectors (heading in radians) ---
                Vector3 obsForward =
                    new Vector3(
                        Mathf.Sin(obs.heading),
                        0f,
                        Mathf.Cos(obs.heading));

                Vector3 predForward =
                    new Vector3(
                        Mathf.Sin(pred.heading),
                        0f,
                        Mathf.Cos(pred.heading));


                // --- front/rear points (OBS) ---
                Vector3 obsFront =
                    obs.position + obsForward * halfLength;

                Vector3 obsRear =
                    obs.position - obsForward * halfLength;


                // --- front/rear points (PRED) ---
                Vector3 predFront =
                    pred.position + predForward * halfLength;

                Vector3 predRear =
                    pred.position - predForward * halfLength;


                // --- thresholded Euclidean similarity ---
                if (Vector3.Distance(obsFront, predFront) <= lcssAcceptanceMagnitude)
                {
                    frontMatches++;
                }

                if (Vector3.Distance(obsRear, predRear) <= lcssAcceptanceMagnitude)
                {
                    rearMatches++;
                }


                count++;
            }


            double frontSimilarity =
                count > 0
                ? (double)frontMatches / count
                : 0.0;


            double rearSimilarity =
                count > 0
                ? (double)rearMatches / count
                : 0.0;


            double similarity =
                (frontSimilarity + rearSimilarity) * 0.5;


            double lcssError =
                1.0 - similarity;


            return lcssError;
        }
        else if(predictionMode == PredictionMode.LCSSFinal)
        {
            VehicleState obs = Observations[^1];
            VehicleState pred = trajectory.states[trajectoryAnchorIndex];

            // --- forward vectors (heading in radians) ---
            Vector3 obsForward =
                new Vector3(
                    Mathf.Sin(obs.heading),
                    0f,
                    Mathf.Cos(obs.heading));

            Vector3 predForward =
                new Vector3(
                    Mathf.Sin(pred.heading),
                    0f,
                    Mathf.Cos(pred.heading));


            // --- front/rear points (OBS) ---
            Vector3 obsFront =
                obs.position + obsForward * halfLength;

            Vector3 obsRear =
                obs.position - obsForward * halfLength;


            // --- front/rear points (PRED) ---
            Vector3 predFront =
                pred.position + predForward * halfLength;

            Vector3 predRear =
                pred.position - predForward * halfLength;
            bool frontMatches = false;
            bool rearMatches = false;
            if (Vector3.Distance(obsFront, predFront) <= lcssAcceptanceMagnitude)
            {
                frontMatches = true;
            }

            if (Vector3.Distance(obsRear, predRear) <= lcssAcceptanceMagnitude)
            {
                rearMatches = true;
            }
            float frontSimilarity = frontMatches ? 1.0f : 0.0f;
            float rearSimilarity = rearMatches ? 1.0f : 0.0f;

            float similarity = (frontSimilarity + rearSimilarity) * 0.5f;
            double lcssError = 1.0f - similarity;
            return lcssError;
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
        else if (predictionMode == PredictionMode.FDE)
        {
            foreach (Trajectory trajectory in potentialTrajectories)
            {
                if (trajectory.states.Count == 0)
                    continue;

                int anchorIndex = FindClosestIndex(trajectory.states, tLatest); 
                int minIndex = FindClosestIndexLowest(trajectory.states, Mathf.Max(0, tLatest - vehicleController.GetManouvreDuration()));
                float halfLength = vehicleController.GetVehicleLength() / 2;

                int trajIndex = anchorIndex;

                if (trajIndex < minIndex)
                    break;

                VehicleState obs = Observations[^1];
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

                // Track top 3 matches
                topRecords.Add(new TrajectoryRecord
                {
                    trajectory = trajectory,
                    error = error,
                    observations = new List<VehicleState>(Observations),
                    trajectoryAnchorIndex = anchorIndex,
                });

                topRecords.Sort((a, b) => a.error.CompareTo(b.error));

                if (topRecords.Count > 3)
                    topRecords.RemoveAt(3);


                if (error < bestError)
                {
                    runCusumSim = true;
                }

            }
        }
        else if (predictionMode == PredictionMode.LCSS)
        {
            foreach (Trajectory trajectory in potentialTrajectories)
            {
                if (trajectory.states.Count == 0)
                    continue;

                int frontMatches = 0;
                int rearMatches = 0;
                int count = 0;

                int anchorIndex = FindClosestIndex(trajectory.states, tLatest);
                int minIndex = FindClosestIndexLowest(trajectory.states, Mathf.Max(0, tLatest - vehicleController.GetManouvreDuration()));
                float halfLength = vehicleController.GetVehicleLength() / 2;


                for (int i = obsCount - 1; i >= 0; i--)
                {
                    int trajIndex =
                        anchorIndex - (obsCount - 1 - i);


                    if (trajIndex < minIndex)
                        break;


                    VehicleState obs =
                        Observations[i];

                    VehicleState pred =
                        trajectory.states[trajIndex];


                    // --- forward vectors (heading in radians) ---
                    Vector3 obsForward =
                        new Vector3(
                            Mathf.Sin(obs.heading),
                            0f,
                            Mathf.Cos(obs.heading));

                    Vector3 predForward =
                        new Vector3(
                            Mathf.Sin(pred.heading),
                            0f,
                            Mathf.Cos(pred.heading));


                    // --- front/rear points (OBS) ---
                    Vector3 obsFront =
                        obs.position + obsForward * halfLength;

                    Vector3 obsRear =
                        obs.position - obsForward * halfLength;


                    // --- front/rear points (PRED) ---
                    Vector3 predFront =
                        pred.position + predForward * halfLength;

                    Vector3 predRear =
                        pred.position - predForward * halfLength;


                    // --- thresholded Euclidean similarity ---
                    if (Vector3.Distance(obsFront, predFront) <= lcssAcceptanceMagnitude)
                    {
                        frontMatches++;
                    }

                    if (Vector3.Distance(obsRear, predRear) <= lcssAcceptanceMagnitude)
                    {
                        rearMatches++;
                    }


                    count++;
                }


                double frontSimilarity =
                    count > 0
                    ? (double)frontMatches / count
                    : 0.0;


                double rearSimilarity =
                    count > 0
                    ? (double)rearMatches / count
                    : 0.0;


                double similarity =
                    (frontSimilarity + rearSimilarity) * 0.5;


                double lcssError =
                    1.0 - similarity;

                // Track top 3 matches
                topRecords.Add(new TrajectoryRecord
                {
                    trajectory = trajectory,
                    error = lcssError,
                    observations = new List<VehicleState>(Observations),
                    trajectoryAnchorIndex = anchorIndex,
                });

                topRecords.Sort((a, b) => a.error.CompareTo(b.error));

                if (topRecords.Count > 3)
                    topRecords.RemoveAt(3);


                if (lcssError < bestError)
                {
                    runCusumSim = true;
                }
            }
        }
        else if (predictionMode == PredictionMode.LCSSFinal)
        {
            foreach (Trajectory trajectory in potentialTrajectories)
            {
                if (trajectory.states.Count == 0)
                    continue;

                int anchorIndex = FindClosestIndex(trajectory.states, tLatest);
                int minIndex = FindClosestIndexLowest(trajectory.states, Mathf.Max(0, tLatest - vehicleController.GetManouvreDuration()));
                float halfLength = vehicleController.GetVehicleLength() / 2;
                int trajectoryAnchorIndex = anchorIndex;

                VehicleState obs = Observations[^1];
                VehicleState pred = trajectory.states[trajectoryAnchorIndex];

                // --- forward vectors (heading in radians) ---
                Vector3 obsForward =
                    new Vector3(
                        Mathf.Sin(obs.heading),
                        0f,
                        Mathf.Cos(obs.heading));

                Vector3 predForward =
                    new Vector3(
                        Mathf.Sin(pred.heading),
                        0f,
                        Mathf.Cos(pred.heading));


                // --- front/rear points (OBS) ---
                Vector3 obsFront =
                    obs.position + obsForward * halfLength;

                Vector3 obsRear =
                    obs.position - obsForward * halfLength;


                // --- front/rear points (PRED) ---
                Vector3 predFront =
                    pred.position + predForward * halfLength;

                Vector3 predRear =
                    pred.position - predForward * halfLength;
                bool frontMatches = false;
                bool rearMatches = false;
                if (Vector3.Distance(obsFront, predFront) <= lcssAcceptanceMagnitude)
                {
                    frontMatches = true;
                }

                if (Vector3.Distance(obsRear, predRear) <= lcssAcceptanceMagnitude)
                {
                    rearMatches = true;
                }
                float frontSimilarity = frontMatches ? 1.0f : 0.0f;
                float rearSimilarity = rearMatches ? 1.0f : 0.0f;

                float similarity = (frontSimilarity + rearSimilarity) * 0.5f;
                double lcssError = 1.0f - similarity;

                // Track top 3 matches
                topRecords.Add(new TrajectoryRecord
                {
                    trajectory = trajectory,
                    error = lcssError,
                    observations = new List<VehicleState>(Observations),
                    trajectoryAnchorIndex = anchorIndex,
                });

                topRecords.Sort((a, b) => a.error.CompareTo(b.error));

                if (topRecords.Count > 3)
                    topRecords.RemoveAt(3);


                runCusumSim = true;

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

                double cusum = 0.0;

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

                    cusum =
                        simulatedObservations[startObservationIndex].recordedCusum ?? 0.0;

                    for (int obsIndex = startObservationIndex;
                         obsIndex <= simulatedObservations.Count - 1;
                         obsIndex++)
                    {
                        VehicleState obs = simulatedObservations[obsIndex];

                        int maxIndex =
                            FindClosestIndex(
                            record.trajectory.states,
                            obs.t);

                        int minIndex = 0;

                        double error = FindErrorMeasure(
                            record.trajectory,
                            maxIndex,
                            minIndex,
                            simulatedObservations.GetRange(0,
                                obsIndex + 1));

                        cusum += error;
                        if (predictionMode == PredictionMode.ADE || predictionMode == PredictionMode.FDE)
                            cusum = System.Math.Max(0.0, addObservationNoise ? cusum - cusumNoiseAlignmentEuclidean : cusum - cusumNoiseAlignmentEuclidean);
                        else if (predictionMode == PredictionMode.LCSS || predictionMode == PredictionMode.LCSSFinal)
                            cusum = System.Math.Max(0.0, addObservationNoise ? cusum - cusumNoiseAlignmentLCSS : cusum - cusumNoiseAlignmentLCSS);

                        obs.recordedCusum = cusum;
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
        if (currentTrajectory.isLaneSettle)
        {
            lastTrajectoryStates = currentTrajectory.states.Where(s => s.t < currentTrajectory.trajectoryStart).ToList();
            var latestState = lastTrajectoryStates[^1];

            //create an updated LaneChange trajectory
            Trajectory? updatedTrajResult = UpdateLaneSettleTrajectory(
                    currentTrajectory,
                    currentTrajectory.LaneTo,
                    fromState);
            if (updatedTrajResult != null)
            {
                var updatedTraj = new Trajectory();
                updatedTraj.LaneFrom = updatedTrajResult.LaneFrom;
                updatedTraj.LaneTo = updatedTrajResult.LaneTo;
                updatedTraj.isLaneSettle = updatedTrajResult.isLaneSettle;
                foreach (var state in lastTrajectoryStates)
                {
                    updatedTraj.states.Add(state);
                }
                foreach (var state in updatedTrajResult.states)
                {
                    updatedTraj.states.Add(state);
                }

                updatedTraj.followingTrajectory = null;
                updatedTraj.trajectoryStart = latestState.t;
                TransitionTrajectories.Add(updatedTraj);
            }
        }
        else
        {
            if (currentTrajectory.LaneFrom == currentTrajectory.LaneTo)
            {
                lastTrajectoryStates = currentTrajectory.states.Where(s =>
                                                                    s.t >= currentTrajectory.trajectoryStart - vehicleController.GetManouvreDuration() &&
                                                                    s.position.z < fromState.position.z && s.t < fromState.t)
                                                                    .ToList();
            }
            else
            {
                lastTrajectoryStates = currentTrajectory.states.Where(s => s.t < currentTrajectory.trajectoryStart).ToList();
            }

            if (lastTrajectoryStates.Count == 0 || fromState.velocity < vehicleController.GetSpeedLimitMin())
                return TransitionTrajectories;
            var latestState = lastTrajectoryStates[^1];

            if (currentTrajectory.LaneFrom == currentTrajectory.LaneTo)
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
                observationUpdateTrajectory.trajectoryStart = latestState.t;
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

                        laneChangeTrajectory.trajectoryStart = latestState.t;

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
                    updatedTraj.trajectoryStart = latestState.t;
                    TransitionTrajectories.Add(updatedTraj);
                }

                //Continue Generating the keep lane trajectories for the original lane

                var keepLaneTrajectory = new Trajectory();

                var lastTrajectoryStatesDuration = currentTrajectory.states
                    .Where(s =>
                        s.t >= fromState.t - vehicleController.GetManouvreDuration() &&
                        s.position.z < fromState.position.z &&
                        s.t < currentTrajectory.trajectoryStart)
                    .ToList();

                if (lastTrajectoryStatesDuration.Count > 0)
                {
                    VehicleState lastState = lastTrajectoryStatesDuration[^1];

                    lastTrajectoryStatesDuration.AddRange(
                        ExtendLaneHistoryToTime(
                            currentTrajectory.LaneFrom,
                            lastState,
                            fromState.t));
                }

                keepLaneTrajectory.LaneFrom = currentTrajectory.LaneFrom;
                keepLaneTrajectory.LaneTo = currentTrajectory.LaneFrom;
                foreach (var state in lastTrajectoryStatesDuration)
                {
                    if (state.t < fromState.t)
                    {
                        keepLaneTrajectory.states.Add(state);
                    }
                }

                var keepLane = BuildLaneFollowTrajectory(keepLaneTrajectory.LaneTo, fromState);
                foreach (var state in keepLane.states)
                {
                    keepLaneTrajectory.states.Add(state);
                }
                keepLaneTrajectory.trajectoryStart = latestState.t;
                TransitionTrajectories.Add(keepLaneTrajectory);
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
            position = new Vector3(vehicleController.transform.position.x, 0, vehicleController.transform.position.z),
        };

        //add noise
        if (simulationTime >= nextChangeNoiseDirectionTime)
        {
            RegenerateNoiseDirections();
            nextChangeNoiseDirectionTime += Random.Range(changeNoiseDirectionPeriodMinimum, changeNoiseDirectionPeriodMinimum);
        }
        if (addObservationNoise)
        {
            newState.position = newState.position + new Vector3(
                Random.Range(0, maximumPositionNoiseMagnitude) * positionNoiseDirection.x,
                0,
                Random.Range(0, maximumPositionNoiseMagnitude) * positionNoiseDirection.y
                );
            newState.heading = newState.heading + (Random.Range(0, maximumHeadingNoiseMagnitude) * headingNoiseDirection * Mathf.Deg2Rad);
            newState.velocity = Mathf.Max(0, newState.velocity + (Random.Range(0, maximumVelocityNoiseMagnitude) * speedNoiseDirection));
            newState.acceleration = newState.acceleration + (Random.Range(0,maximumAccelerationNoiseMagnitude) * speedNoiseDirection);
        }
        if (simulationTime >= nextAnomalyTime)
        {
            nextAnomalyTime += Random.Range(anomalyOccurancePeriodMinimum, anomalyOccurancePeriodMaximum);
            if (addObservationAnomalies)
            {
                newState.position = newState.position + new Vector3(
                    Random.Range(anomalyMinimumPositionMagnitude, anomalyMaximumPositionMagnitude) * positionNoiseDirection.x,
                    0,
                    Random.Range(anomalyMinimumPositionMagnitude, anomalyMaximumPositionMagnitude) * positionNoiseDirection.y
                    );
                newState.heading = newState.heading + (Random.Range(0, anomalyMaximumHeadingMagnitude) * headingNoiseDirection * Mathf.Deg2Rad);
                newState.velocity = Mathf.Max(0, newState.velocity + (Random.Range(0, anomalyMaximumHeadingMagnitude) * speedNoiseDirection));
                newState.acceleration = newState.acceleration + (Random.Range(0, anomalyMaximumAccelerationMagnitude) * speedNoiseDirection);
            }
        }

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

    public Trajectory? UpdateLaneSettleTrajectory(Trajectory updateTrajectory, Transform Lane, VehicleState Observation)
    {
        Trajectory resultTrajectory = new Trajectory();

        resultTrajectory.LaneFrom = Lane;
        resultTrajectory.LaneTo = Lane;
        resultTrajectory.isLaneSettle = true;

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
        var differenceToManouvreDuration = vehicleController.GetLaneSettleDuration() - timeWithinCurrent;

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

        float startX = Observation.position.x;
        float targetX = Lane.position.x;
        float deltaX = targetX - startX;

        var duration = vehicleController.GetLaneSettleDuration(); ;

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
        float duration = vehicleController.GetLaneSettleDuration();

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

    Trajectory BuildOODTransitionLaneFollowTrajectory(Transform Lane, VehicleState Observation)
    {
        Trajectory resultTrajectory = new Trajectory();

        resultTrajectory.LaneFrom = Lane;
        resultTrajectory.LaneTo = Lane;


        float dt = sampleRate;
        float duration = vehicleController.GetManouvreDuration();


        float velocity =
            Mathf.Max(
                Observation.velocity,
                vehicleController.GetSpeedLimitMin());

        float accel = Observation.acceleration;


        /*
         * Generate backwards history
         */

        List<VehicleState> history = new List<VehicleState>();

        Vector3 pos = new Vector3(
            Lane.position.x,
            0,
            Observation.position.z);


        float t = Observation.t;


        history.Add(new VehicleState
        {
            t = t,
            position = pos,
            heading = Observation.heading,
            velocity = velocity,
            acceleration = accel
        });


        float trajectoryTime = 0f;

        float reverseVelocity = velocity;


        while (trajectoryTime < duration)
        {
            t -= dt;


            // reverse the forward implementation:
            // forward:
            // velocity += accel * dt
            // pos.z += velocity * dt

            reverseVelocity -= accel * dt;

            reverseVelocity =
                Mathf.Max(
                    reverseVelocity,
                    vehicleController.GetSpeedLimitMin());


            pos.z -= reverseVelocity * dt;


            history.Add(new VehicleState
            {
                t = t,
                position = pos,
                heading = Observation.heading,
                velocity = reverseVelocity,
                acceleration = accel
            });


            trajectoryTime += dt;
        }


        history.Reverse();


        foreach (var state in history)
        {
            resultTrajectory.states.Add(state);
        }



        /*
         * Generate forward trajectory
         */


        pos = new Vector3(
            Lane.position.x,
            0,
            Observation.position.z);


        velocity =
            Mathf.Max(
                Observation.velocity,
                vehicleController.GetSpeedLimitMin());


        accel = Observation.acceleration;


        t = Observation.t;

        resultTrajectory.trajectoryStart =
            t;


        trajectoryTime = 0f;



        // observation state
        float heading =
            Mathf.Atan2(
                (Lane.position.x - pos.x) / dt,
                Mathf.Max(velocity, 0.5f));


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

            if (velocity + (accel * dt) <
                vehicleController.GetSpeedLimitMin())
            {
                velocity =
                    vehicleController.GetSpeedLimitMin();
            }
            else
            {
                velocity += accel * dt;
            }


            float dz = velocity * dt;


            // EXACT lane-follow behaviour
            pos.x = Lane.position.x;
            pos.z += dz;



            float dxdt =
                (Lane.position.x - pos.x) / dt;


            float dzdt =
                Mathf.Max(velocity, 0.5f);


            heading =
                Mathf.Atan2(
                    dxdt,
                    dzdt);



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


        resultTrajectory.isLaneSettle = false;

        return resultTrajectory;
    }

    Trajectory GenerateCTRATrajectory()
    {
        Trajectory trajectory = new Trajectory();

        if (Observations.Count < 2)
            return trajectory;

        float historyDuration = 0.5f;
        float dt = sampleRate;
        float horizon = vehicleController.GetManouvreDuration();

        VehicleState latest = Observations[^1];

        // Collect recent observations
        List<VehicleState> history = Observations
            .Where(o => o.t >= latest.t - historyDuration)
            .ToList();

        if (history.Count < 2)
            history = Observations;

        // Estimate average velocity components
        float vx = 0f;
        float vz = 0f;
        int velocitySamples = 0;

        for (int i = 1; i < history.Count; i++)
        {
            float sampleDt = history[i].t - history[i - 1].t;

            if (sampleDt <= 0f)
                continue;

            Vector3 delta = history[i].position - history[i - 1].position;

            vx += delta.x / sampleDt;
            vz += delta.z / sampleDt;
            velocitySamples++;
        }

        if (velocitySamples == 0)
            return trajectory;

        vx /= velocitySamples;
        vz /= velocitySamples;

        // Estimate average acceleration components
        float ax = 0f;
        float az = 0f;
        int accelSamples = 0;

        for (int i = 2; i < history.Count; i++)
        {
            float dt1 = history[i - 1].t - history[i - 2].t;
            float dt2 = history[i].t - history[i - 1].t;

            if (dt1 <= 0f || dt2 <= 0f)
                continue;

            Vector3 vPrev = (history[i - 1].position - history[i - 2].position) / dt1;
            Vector3 vCurr = (history[i].position - history[i - 1].position) / dt2;

            ax += (vCurr.x - vPrev.x) / dt2;
            az += (vCurr.z - vPrev.z) / dt2;
            accelSamples++;
        }

        if (accelSamples > 0)
        {
            ax /= accelSamples;
            az /= accelSamples;
        }

        Vector3 pos = latest.position;
        float t = latest.t;

        trajectory.trajectoryStart = t;

        float heading = Mathf.Atan2(vx, vz);

        trajectory.states.Add(new VehicleState
        {
            t = t,
            position = pos,
            heading = heading,
            velocity = Mathf.Sqrt(vx * vx + vz * vz),
            acceleration = Mathf.Sqrt(ax * ax + az * az)
        });

        for (float sim = dt; sim <= horizon; sim += dt)
        {
            // Position update using constant acceleration kinematics
            pos.x += vx * dt + 0.5f * ax * dt * dt;
            pos.z += vz * dt + 0.5f * az * dt * dt;

            // Update velocity
            vx += ax * dt;
            vz += az * dt;

            heading = Mathf.Atan2(vx, vz);
            t += dt;

            trajectory.states.Add(new VehicleState
            {
                t = t,
                position = pos,
                heading = heading,
                velocity = Mathf.Sqrt(vx * vx + vz * vz),
                acceleration = Mathf.Sqrt(ax * ax + az * az)
            });
        }

        return trajectory;
    }
    List<Trajectory> FindOODTransitionTrajectories(VehicleState observation)
    {
        float dt = sampleRate;
        //Filter out old trajectories
        float cutoffTime = observation.t - (vehicleController.GetManouvreDuration());
        OODTransitionTrajectories.RemoveAll(t => t.trajectoryStart < cutoffTime);

        foreach (Transform lane in centreLanes)
        {
            Trajectory laneTrajectory =
                BuildOODTransitionLaneFollowTrajectory(
                    lane,
                    observation);


            OODTransitionTrajectories.Add(laneTrajectory);
        }

        return OODTransitionTrajectories;
    }

    private (double cusum, List<VehicleState> observations)
    SimulateOODCusum(Trajectory trajectory)
    {
        List<VehicleState> simulatedObservations =
            Observations
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



        double cusum = 0.0;


        int anchorIndex =
            FindClosestIndex(
                trajectory.states,
                simulatedObservations[^1].t);



        int startIndex =
            Mathf.Max(
                0,
                simulatedObservations.Count -
                Mathf.CeilToInt(
                    vehicleController.GetManouvreDuration()
                    / sampleRate));



        for (int i = startIndex; i < simulatedObservations.Count; i++)
        {
            VehicleState obs =
                simulatedObservations[i];


            int trajectoryIndex =
                FindClosestIndex(
                    trajectory.states,
                    obs.t);


            if (trajectoryIndex < 0)
                continue;



            int minIndex = 0;



            double error =
                FindErrorMeasure(
                    trajectory,
                    trajectoryIndex,
                    minIndex,
                    new List<VehicleState>
                    {
                    obs
                    });



            cusum += error;

            if (predictionMode == PredictionMode.ADE || predictionMode == PredictionMode.FDE)
                cusum = System.Math.Max(0.0, addObservationNoise ? cusum - cusumNoiseAlignmentEuclidean : cusum - cusumNoiseAlignmentEuclidean);
            else if (predictionMode == PredictionMode.LCSS || predictionMode == PredictionMode.LCSSFinal)
                cusum = System.Math.Max(0.0, addObservationNoise ? cusum - cusumNoiseAlignmentLCSS : cusum - cusumNoiseAlignmentLCSS);


            obs.recordedCusum = cusum;
        }


        return
        (
            cusum,
            simulatedObservations
        );
    }

    private Trajectory SelectBestOODTrajectory(List<Trajectory> trajectories)
    {
        Trajectory bestTrajectory = null;
        double bestError = double.PositiveInfinity;

        int obsCount = Observations.Count;
        float tLatest = Observations[^1].t;


        foreach (Trajectory trajectory in trajectories)
        {
            if (trajectory.states.Count == 0)
                continue;


            int anchorIndex =
                FindClosestIndex(
                    trajectory.states,
                    tLatest);


            int minIndex =
                FindClosestIndexLowest(
                    trajectory.states,
                    Mathf.Max(
                        0,
                        tLatest - vehicleController.GetManouvreDuration()));

            float halfLength = vehicleController.GetVehicleLength() / 2;


            if (predictionMode == PredictionMode.ADE)
            {
                double totalError = 0.0;
                int count = 0;


                for (int i = obsCount - 1; i >= 0; i--)
                {
                    int trajIndex =
                        anchorIndex -
                        (obsCount - 1 - i);


                    if (trajIndex < minIndex)
                        break;


                    VehicleState obs =
                        Observations[i];

                    VehicleState pred =
                        trajectory.states[trajIndex];

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

                    totalError +=
                        error;

                    count++;
                }


                double ade =
                    count > 0
                    ? totalError / count
                    : double.PositiveInfinity;



                if (ade < bestError)
                {
                    bestError = ade;
                    bestTrajectory = trajectory;
                }
            }
            else if(predictionMode == PredictionMode.FDE)
            {
                int trajIndex =
                        anchorIndex;


                if (trajIndex < minIndex)
                    break;


                VehicleState obs =
                    Observations[^1];

                VehicleState pred =
                    trajectory.states[trajIndex];

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

                if (error < bestError)
                {
                    bestError = error;
                    bestTrajectory = trajectory;
                }
            }
            else if(predictionMode == PredictionMode.LCSS)
            {
                if (trajectory.states.Count == 0)
                    continue;

                int frontMatches = 0;
                int rearMatches = 0;
                int count = 0;


                for (int i = obsCount - 1; i >= 0; i--)
                {
                    int trajIndex =
                        anchorIndex - (obsCount - 1 - i);


                    if (trajIndex < minIndex)
                        break;


                    VehicleState obs =
                        Observations[i];

                    VehicleState pred =
                        trajectory.states[trajIndex];


                    // --- forward vectors (heading in radians) ---
                    Vector3 obsForward =
                        new Vector3(
                            Mathf.Sin(obs.heading),
                            0f,
                            Mathf.Cos(obs.heading));

                    Vector3 predForward =
                        new Vector3(
                            Mathf.Sin(pred.heading),
                            0f,
                            Mathf.Cos(pred.heading));


                    // --- front/rear points (OBS) ---
                    Vector3 obsFront =
                        obs.position + obsForward * halfLength;

                    Vector3 obsRear =
                        obs.position - obsForward * halfLength;


                    // --- front/rear points (PRED) ---
                    Vector3 predFront =
                        pred.position + predForward * halfLength;

                    Vector3 predRear =
                        pred.position - predForward * halfLength;


                    // --- thresholded Euclidean similarity ---
                    if (Vector3.Distance(obsFront, predFront) <= lcssAcceptanceMagnitude)
                    {
                        frontMatches++;
                    }

                    if (Vector3.Distance(obsRear, predRear) <= lcssAcceptanceMagnitude)
                    {
                        rearMatches++;
                    }


                    count++;
                }


                double frontSimilarity =
                    count > 0
                    ? (double)frontMatches / count
                    : 0.0;


                double rearSimilarity =
                    count > 0
                    ? (double)rearMatches / count
                    : 0.0;


                double similarity =
                    (frontSimilarity + rearSimilarity) * 0.5;


                double lcssError =
                    1.0 - similarity;


                if (lcssError < bestError)
                {
                    bestError = lcssError;
                    bestTrajectory = trajectory;
                }
            }
            else if(predictionMode == PredictionMode.LCSSFinal)
            {
                VehicleState obs = Observations[^1];
                int trajIndex = anchorIndex;

                VehicleState pred = trajectory.states[trajIndex];

                // --- forward vectors (heading in radians) ---
                Vector3 obsForward =
                    new Vector3(
                        Mathf.Sin(obs.heading),
                        0f,
                        Mathf.Cos(obs.heading));

                Vector3 predForward =
                    new Vector3(
                        Mathf.Sin(pred.heading),
                        0f,
                        Mathf.Cos(pred.heading));


                // --- front/rear points (OBS) ---
                Vector3 obsFront =
                    obs.position + obsForward * halfLength;

                Vector3 obsRear =
                    obs.position - obsForward * halfLength;


                // --- front/rear points (PRED) ---
                Vector3 predFront =
                    pred.position + predForward * halfLength;

                Vector3 predRear =
                    pred.position - predForward * halfLength;
                bool frontMatches = false;
                bool rearMatches = false;
                if (Vector3.Distance(obsFront, predFront) <= lcssAcceptanceMagnitude)
                {
                    frontMatches = true;
                }

                if (Vector3.Distance(obsRear, predRear) <= lcssAcceptanceMagnitude)
                {
                    rearMatches = true;
                }
                float frontSimilarity = frontMatches ? 1.0f : 0.0f;
                float rearSimilarity = rearMatches ? 1.0f : 0.0f;

                float similarity = (frontSimilarity + rearSimilarity) * 0.5f;
                double lcssError = 1.0f - similarity;

                if (lcssError < bestError)
                {
                    bestError = lcssError;
                    bestTrajectory = trajectory;
                }
            }
        }


        return bestTrajectory;
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
    private void RegenerateNoiseDirections()
    {
        positionNoiseDirection = Random.insideUnitCircle.normalized;
        headingNoiseDirection = Random.value < 0.5f ? -1 : 1;
        speedNoiseDirection = Random.value < 0.5f ? -1 : 1;
    }

    private List<VehicleState> ExtendLaneHistoryToTime(
    Transform lane,
    VehicleState startState,
    float targetTime)
    {
        List<VehicleState> result = new List<VehicleState>();

        float dt = sampleRate;

        Vector3 pos = startState.position;
        float velocity = Mathf.Max(startState.velocity, vehicleController.GetSpeedLimitMin());
        float accel = startState.acceleration;
        float heading = startState.heading;
        float t = startState.t;

        while (t + dt < targetTime)
        {
            if (velocity + accel * dt < vehicleController.GetSpeedLimitMin())
                velocity = vehicleController.GetSpeedLimitMin();
            else
                velocity += accel * dt;

            float targetX = lane.position.x;

            pos.x = targetX;
            pos.z += velocity * dt;

            float dxdt = (targetX - pos.x) / dt;
            float dzdt = Mathf.Max(velocity, 0.0001f);
            heading = Mathf.Atan2(dxdt, dzdt);

            t += dt;

            result.Add(new VehicleState
            {
                t = t,
                position = pos,
                heading = heading,
                velocity = velocity,
                acceleration = accel
            });
        }

        return result;
    }
}
