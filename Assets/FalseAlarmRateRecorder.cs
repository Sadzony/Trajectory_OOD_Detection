using UnityEngine;
using System.Collections.Generic;
using System.IO;
using UnityEngine.UI;

public class FalseAlarmRateRecorder : MonoBehaviour
{
    [SerializeField] Toggle farToggle;

    private bool recordAlarms = false;

    private double cusumPeak = 0.0;

    public bool RecordAlarms
    {
        get => recordAlarms;
        set
        {
            if (recordAlarms == value)
                return;

            recordAlarms = value;

            farToggle.isOn = value;

            OnRecordAlarmsChanged();
        }
    }

    private void OnRecordAlarmsChanged()
    {
        if(!recordAlarms)
        {
            EndCsvSession();
        }
        // Clear old session data
        allTrajectoryRecords.Clear();
        FalseAlarms.Clear();
        cusumPeak = 0.0;

        if (recordAlarms)
        {
            StartNewCsvSession();
        }

    }

    private void StartNewCsvSession()
    {
        currentSessionFolder = Path.Combine(
            GetSessionFolder(),
            $"Session_{System.DateTime.Now:ddMMyy_HHmmss}");

        Directory.CreateDirectory(currentSessionFolder);


        // Summary file
        string summaryPath = Path.Combine(
            currentSessionFolder,
            "Summary.csv");

        summaryCsvFile = new StreamWriter(summaryPath, false);

        summaryCsvFile.WriteLine(
            "CUSUM Threshold,CUSUM Noise Alignment,Error Measure Method,Total Trajectories,Total Alarms,False Alarm Rate,CUSUM Peak,Observation Noise Present,Observation Anomalies Present,Motion Bias Present,Throttle Noise Present,Lane Oscillation Present");

        summaryCsvFile.Flush();



        // Trajectories file
        string trajectoriesPath = Path.Combine(
            currentSessionFolder,
            "Trajectories.csv");

        trajectoriesCsvFile = new StreamWriter(trajectoriesPath, false);

        trajectoriesCsvFile.WriteLine(
            "Trajectory Type,Trajectory Start Time");

        trajectoriesCsvFile.Flush();



        // Alarms file
        string alarmsPath = Path.Combine(
            currentSessionFolder,
            "Alarms.csv");

        alarmsCsvFile = new StreamWriter(alarmsPath, false);

        alarmsCsvFile.WriteLine(
            "Trajectory Type,Predicted Trajectory Type,Target Speed,Vehicle Speed,Trajectory Start Time,Alarm Time,Time Within Trajectory,Latest Error");

        alarmsCsvFile.Flush();


        Debug.Log($"Started alarm recording session: {currentSessionFolder}");
    }


    private void EndCsvSession()
    {
        UpdateSummaryFile();
        summaryCsvFile?.Dispose();
        trajectoriesCsvFile?.Dispose();
        alarmsCsvFile?.Dispose();

        summaryCsvFile = null;
        trajectoriesCsvFile = null;
        alarmsCsvFile = null;

        currentSessionFolder = null;

        Debug.Log("Alarm recording stopped.");
    }

    private string GetSessionFolder()
    {
        string folder;

        if (Application.isEditor)
        {
            folder = Path.Combine(
                Directory.GetParent(Application.dataPath).FullName,
                "FalseAlarmRateSessions");
        }
        else
        {
            folder = Path.Combine(
                Directory.GetParent(Application.dataPath).FullName,
                "FalseAlarmRateSessions");
        }

        Directory.CreateDirectory(folder);

        return folder;
    }

    private string currentSessionFolder;

    private StreamWriter summaryCsvFile;
    private StreamWriter trajectoriesCsvFile;
    private StreamWriter alarmsCsvFile;

    private bool observationNoisePresent;
    private bool observationAnomaliesPresent;
    private bool motionBiasPresent;
    private bool throttleNoisePresent;
    private bool laneOscillationPresent;

    public LaneDefinedMovementQuintic car;
    public TrajectoryPredictor predictor;



    public ZigZagLaneBehaviour zigZagLaneBehaviour;
    public ZigZagRoadBehaviour zigZagRoadBehaviour;
    public TrailOffBehaviour trailOffBehaviour;
    public LossOfControlBehaviour lossOfControlBehaviour;

    private enum CarBehaviours
    {
        None,
        Standard,
        Brake,
        TrailOff,
        ZigZagLane,
        ZigZagRoad,
        Cut,
        LoseControl
    }

    float simulationTime = 0.0f;
    CarBehaviours currentBehaviour;

    CarBehaviours lastBehaviour = CarBehaviours.None;

    private void Start()
    {
        if (predictor.addObservationNoise)
            observationNoisePresent = true;
        else observationNoisePresent = false;

        if (predictor.addObservationAnomalies)
            observationAnomaliesPresent = true;
        else observationAnomaliesPresent = false;

        if (car.addMotionBias)
            motionBiasPresent = true;
        else motionBiasPresent = false;

        if (car.throttleNoiseOn)
            throttleNoisePresent = true;
        else throttleNoisePresent = false;

        if (car.oscillate)
            laneOscillationPresent = true;
        else laneOscillationPresent = false;

        //1. Determine the car's current behaviour
        if (car.enabled == true && car.longitudinalState != LaneDefinedMovementQuintic.LongitudinalState.Braking && car.longitudinalState != LaneDefinedMovementQuintic.LongitudinalState.Stopped && !car.cuttingLanes)
            currentBehaviour = CarBehaviours.Standard;
        else if (car.enabled == true && (car.longitudinalState == LaneDefinedMovementQuintic.LongitudinalState.Braking || car.longitudinalState == LaneDefinedMovementQuintic.LongitudinalState.Stopped))
            currentBehaviour = CarBehaviours.Brake;
        else if (car.enabled && car.cuttingLanes)
            currentBehaviour = CarBehaviours.Cut;
        else if (car.enabled == false && trailOffBehaviour.enabled == true)
            currentBehaviour = CarBehaviours.TrailOff;
        else if (car.enabled == false && zigZagLaneBehaviour.enabled == true)
            currentBehaviour = CarBehaviours.ZigZagLane;
        else if (car.enabled == false && zigZagRoadBehaviour.enabled == true)
            currentBehaviour = CarBehaviours.ZigZagRoad;
        else if (car.enabled == false && lossOfControlBehaviour.enabled == true)
            currentBehaviour = CarBehaviours.LoseControl;
        else
            currentBehaviour = CarBehaviours.None;
        lastBehaviour = currentBehaviour;
    }

    void FixedUpdate()
    {

        if (predictor.addObservationNoise)
            observationNoisePresent = true;
        else observationNoisePresent = false;

        if (predictor.addObservationAnomalies)
            observationAnomaliesPresent = true;
        else observationAnomaliesPresent = false;

        if (car.addMotionBias)
            motionBiasPresent = true;
        else motionBiasPresent = false;

        if (car.throttleNoiseOn)
            throttleNoisePresent = true;
        else throttleNoisePresent = false;

        if (car.oscillate)
            laneOscillationPresent = true;
        else laneOscillationPresent = false;

        //1. Determine the car's current behaviour
        if (car.enabled == true && car.longitudinalState != LaneDefinedMovementQuintic.LongitudinalState.Braking && car.longitudinalState != LaneDefinedMovementQuintic.LongitudinalState.Stopped && !car.cuttingLanes)
            currentBehaviour = CarBehaviours.Standard;
        else if (car.enabled == true && (car.longitudinalState == LaneDefinedMovementQuintic.LongitudinalState.Braking || car.longitudinalState == LaneDefinedMovementQuintic.LongitudinalState.Stopped))
            currentBehaviour = CarBehaviours.Brake;
        else if (car.enabled && car.cuttingLanes)
            currentBehaviour = CarBehaviours.Cut;
        else if (car.enabled == false && trailOffBehaviour.enabled == true)
            currentBehaviour = CarBehaviours.TrailOff;
        else if (car.enabled == false && zigZagLaneBehaviour.enabled == true)
            currentBehaviour = CarBehaviours.ZigZagLane;
        else if (car.enabled == false && zigZagRoadBehaviour.enabled == true)
            currentBehaviour = CarBehaviours.ZigZagRoad;
        else if (car.enabled == false && lossOfControlBehaviour.enabled == true)
            currentBehaviour = CarBehaviours.LoseControl;
        else
            currentBehaviour = CarBehaviours.None;

        lastBehaviour = currentBehaviour;
    }

    public void UpdateSimulationTime(float simTime)
    {
        simulationTime = simTime;
    }

    public void CheckCusumPeak(double currentCusum)
    {
        if (currentCusum > cusumPeak)
            cusumPeak = currentCusum;
    }


    private class FalseAlarmRecord
    {
        public TrajectoryRecord relevantTrajectory;
        public TrajectoryType predictedTrajectoryType;
        public float timeOfAlarm;
        public float durationInManouvre;
        public double currentCUSUMThreshold;
        public double currentCUSUMNoiseAlignment;
        public PredictionMode errorMeasureMethod;

        public bool observationNoisePresent;
        public bool observationAnomaliesPresent;
        public bool motionBiasPresent;
        public bool throttleNoisePresent;
        public bool laneOscillationPresent;
    }

    private List<FalseAlarmRecord> FalseAlarms = new List<FalseAlarmRecord>();
    public void RecordAlarmTrigger()
    {
        if (predictor.addObservationNoise)
            observationNoisePresent = true;
        else observationNoisePresent = false;

        if (predictor.addObservationAnomalies)
            observationAnomaliesPresent = true;
        else observationAnomaliesPresent = false;

        if (car.addMotionBias)
            motionBiasPresent = true;
        else motionBiasPresent = false;

        if (car.throttleNoiseOn)
            throttleNoisePresent = true;
        else throttleNoisePresent = false;

        if (car.oscillate)
            laneOscillationPresent = true;
        else laneOscillationPresent = false;

        // Determine the car's current behaviour
        if (car.enabled == true && car.longitudinalState != LaneDefinedMovementQuintic.LongitudinalState.Braking && car.longitudinalState != LaneDefinedMovementQuintic.LongitudinalState.Stopped && !car.cuttingLanes)
            currentBehaviour = CarBehaviours.Standard;
        else if (car.enabled == true && (car.longitudinalState == LaneDefinedMovementQuintic.LongitudinalState.Braking || car.longitudinalState == LaneDefinedMovementQuintic.LongitudinalState.Stopped))
            currentBehaviour = CarBehaviours.Brake;
        else if (car.enabled && car.cuttingLanes)
            currentBehaviour = CarBehaviours.Cut;
        else if (car.enabled == false && trailOffBehaviour.enabled == true)
            currentBehaviour = CarBehaviours.TrailOff;
        else if (car.enabled == false && zigZagLaneBehaviour.enabled == true)
            currentBehaviour = CarBehaviours.ZigZagLane;
        else if (car.enabled == false && zigZagRoadBehaviour.enabled == true)
            currentBehaviour = CarBehaviours.ZigZagRoad;
        else if (car.enabled == false && lossOfControlBehaviour.enabled == true)
            currentBehaviour = CarBehaviours.LoseControl;
        else
            currentBehaviour = CarBehaviours.None;

        if (currentBehaviour == CarBehaviours.Standard && recordAlarms && alarmsCsvFile != null && summaryCsvFile != null &&
        currentSessionFolder != null && trajectoriesCsvFile != null)
        {
            var newFARrecord = new FalseAlarmRecord();
            newFARrecord.durationInManouvre = simulationTime - currentTrajectory.trajectoryStartTime;
            newFARrecord.timeOfAlarm = simulationTime;
            newFARrecord.relevantTrajectory = currentTrajectory;

            newFARrecord.errorMeasureMethod = predictor.predictionMode;
            newFARrecord.currentCUSUMThreshold = (predictor.predictionMode == PredictionMode.ADE || predictor.predictionMode == PredictionMode.FDE) ? predictor.OODThresholdEuclidean : predictor.OODThresholdLCSS;
            newFARrecord.currentCUSUMNoiseAlignment = (predictor.predictionMode == PredictionMode.ADE || predictor.predictionMode == PredictionMode.FDE) ? predictor.cusumNoiseAlignmentEuclidean : predictor.cusumNoiseAlignmentLCSS;

            if (predictor.currentTrajectory.isLaneSettle)
                newFARrecord.predictedTrajectoryType = TrajectoryType.LaneSettle;
            else if (predictor.currentTrajectory.LaneFrom == predictor.currentTrajectory.LaneTo)
                newFARrecord.predictedTrajectoryType = TrajectoryType.LaneFollow;
            else if (predictor.currentTrajectory.LaneFrom != predictor.currentTrajectory.LaneTo)
                newFARrecord.predictedTrajectoryType = TrajectoryType.LaneChange;
            else newFARrecord.predictedTrajectoryType = TrajectoryType.Undefined;

            newFARrecord.observationNoisePresent = observationNoisePresent;
            newFARrecord.observationAnomaliesPresent = observationAnomaliesPresent;
            newFARrecord.motionBiasPresent = motionBiasPresent;
            newFARrecord.throttleNoisePresent = throttleNoisePresent;
            newFARrecord.laneOscillationPresent = laneOscillationPresent;

            FalseAlarms.Add(newFARrecord);

            //save the False alarm
            alarmsCsvFile.WriteLine(
            $"{currentTrajectory.type.ToString()}," +
            $"{newFARrecord.predictedTrajectoryType}," +
            $"{car.GetTargetVelocity()}," +
            $"{car.GetCurrentVelocity()}," +
            $"{currentTrajectory.trajectoryStartTime:F5}," +
            $"{newFARrecord.timeOfAlarm:F5}," +
            $"{newFARrecord.durationInManouvre:F5}," +
            $"{predictor.GetLatestError():F5}"
            );


            //check if the current record actually exists (the only case this happens is when we just exited OOD and triggered it again before the manouvre finished)
            if (!allTrajectoryRecords.Contains(currentTrajectory))
            {
                allTrajectoryRecords.Add(currentTrajectory);
                                trajectoriesCsvFile.WriteLine(
                    $"{currentTrajectory.type.ToString()}," +
                    $"{currentTrajectory.trajectoryStartTime.ToString("F5")}");

                trajectoriesCsvFile.Flush();
                //keep a maximum of 500 records
                if (allTrajectoryRecords.Count > 499)
                {
                    RecordAlarms = false;
                }
            }

            //update the Summary section for this session
            int totalTrajectories = allTrajectoryRecords.Count;
            int totalAlarms = FalseAlarms.Count;

            float falseAlarmRate = totalTrajectories > 0
                ? (float)totalAlarms / totalTrajectories
                : 0f;
            // Update Summary.csv
            UpdateSummaryFile();
        }
    }

    private void UpdateSummaryFile()
    {
        if (summaryCsvFile == null || currentSessionFolder == null)
            return;


        string summaryPath = Path.Combine(
            currentSessionFolder,
            "Summary.csv");


        int totalTrajectories = allTrajectoryRecords.Count;
        int totalAlarms = FalseAlarms.Count;


        float falseAlarmRate = totalTrajectories > 0
            ? (float)totalAlarms / totalTrajectories
            : 0f;


        // Close current writer before overwriting
        summaryCsvFile.Dispose();


        summaryCsvFile = new StreamWriter(summaryPath, false);


        summaryCsvFile.WriteLine(
            "CUSUM Threshold,CUSUM Noise Alignment,Error Measure Method,Total Trajectories,Total Alarms,False Alarm Rate,CUSUM Peak,Observation Noise Present,Observation Anomalies Present,Motion Bias Present,Throttle Noise Present,Lane Oscillation Present");


        summaryCsvFile.WriteLine(
            $"{((predictor.predictionMode == PredictionMode.ADE || predictor.predictionMode == PredictionMode.FDE) ? predictor.OODThresholdEuclidean : predictor.OODThresholdLCSS):F5}," +
            (
                (predictor.predictionMode == PredictionMode.ADE || predictor.predictionMode == PredictionMode.FDE)
                    ? $"{predictor.cusumNoiseAlignmentEuclidean:F5}"
                    : $"{predictor.cusumNoiseAlignmentLCSS:F5} | {predictor.lcssAcceptanceMagnitude:F5}"
            ) + "," +
            $"{predictor.predictionMode.ToString()}," +
            $"{totalTrajectories}," +
            $"{totalAlarms}," +
            $"{falseAlarmRate:F5}," +
            $"{cusumPeak:F5}," +
            $"{(observationNoisePresent ? "True" : "False")}," +
            $"{(observationAnomaliesPresent ? "True" : "False")}," +
            $"{(motionBiasPresent ? "True" : "False")}," +
            $"{(throttleNoisePresent ? "True" : "False")}," +
            $"{(laneOscillationPresent ? "True" : "False")}");

        summaryCsvFile.Flush();
    }

    public enum TrajectoryType
    {
        Undefined,
        LaneFollow,
        LaneChange,
        LaneSettle
    }


    private class TrajectoryRecord 
    {
        public float trajectoryStartTime;
        public TrajectoryType type;
    }

    private TrajectoryRecord currentTrajectory;
    private List<TrajectoryRecord> allTrajectoryRecords = new List<TrajectoryRecord>();

    public void RecordStartedTrajectory(TrajectoryType type)
    {
        if (predictor.addObservationNoise)
            observationNoisePresent = true;
        else observationNoisePresent = false;

        if (predictor.addObservationAnomalies)
            observationAnomaliesPresent = true;
        else observationAnomaliesPresent = false;

        if (car.addMotionBias)
            motionBiasPresent = true;
        else motionBiasPresent = false;

        if (car.throttleNoiseOn)
            throttleNoisePresent = true;
        else throttleNoisePresent = false;

        if (car.oscillate)
            laneOscillationPresent = true;
        else laneOscillationPresent = false;

        // Determine the car's current behaviour
        if (car.enabled == true && car.longitudinalState != LaneDefinedMovementQuintic.LongitudinalState.Braking && car.longitudinalState != LaneDefinedMovementQuintic.LongitudinalState.Stopped && !car.cuttingLanes)
            currentBehaviour = CarBehaviours.Standard;
        else if (car.enabled == true && (car.longitudinalState == LaneDefinedMovementQuintic.LongitudinalState.Braking || car.longitudinalState == LaneDefinedMovementQuintic.LongitudinalState.Stopped))
            currentBehaviour = CarBehaviours.Brake;
        else if (car.enabled && car.cuttingLanes)
            currentBehaviour = CarBehaviours.Cut;
        else if (car.enabled == false && trailOffBehaviour.enabled == true)
            currentBehaviour = CarBehaviours.TrailOff;
        else if (car.enabled == false && zigZagLaneBehaviour.enabled == true)
            currentBehaviour = CarBehaviours.ZigZagLane;
        else if (car.enabled == false && zigZagRoadBehaviour.enabled == true)
            currentBehaviour = CarBehaviours.ZigZagRoad;
        else if (car.enabled == false && lossOfControlBehaviour.enabled == true)
            currentBehaviour = CarBehaviours.LoseControl;
        else
            currentBehaviour = CarBehaviours.None;
        lastBehaviour = currentBehaviour;

        if (currentBehaviour == CarBehaviours.Standard && !predictor.ood)
        {
            TrajectoryRecord newRecord = new TrajectoryRecord();

            //time the trajectory started..
            newRecord.trajectoryStartTime = simulationTime;

            //type of trajectory..
            newRecord.type = type;

            //only add it to all records if we're in-distribution
            if (!predictor.ood)
            {
                allTrajectoryRecords.Add(newRecord);
                if (trajectoriesCsvFile != null)
                {
                    trajectoriesCsvFile.WriteLine(
                        $"{newRecord.type.ToString()}," +
                        $"{newRecord.trajectoryStartTime.ToString("F5")}");


                    trajectoriesCsvFile.Flush();
                }
            }

            //keep a maximum of 500 records
            if (allTrajectoryRecords.Count > 499)
            {
                RecordAlarms = false;
            }

            currentTrajectory = newRecord;
        }
    }

}
