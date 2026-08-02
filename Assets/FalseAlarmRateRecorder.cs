using UnityEngine;
using System.Collections.Generic;
using System.IO;

public class FalseAlarmRateRecorder : MonoBehaviour
{
    private bool recordAlarms = true;

    public bool RecordAlarms
    {
        get => recordAlarms;
        set
        {
            if (recordAlarms == value)
                return;

            recordAlarms = value;
            OnRecordAlarmsChanged();
        }
    }

    private void OnRecordAlarmsChanged()
    {
        // Clear old session data
        allTrajectoryRecords.Clear();

        if (recordAlarms)
        {
            StartNewCsvSession();
        }
        else
        {
            EndCsvSession();
        }
    }

    private void StartNewCsvSession()
    {
        currentCsvPath = Path.Combine(
            Application.persistentDataPath,
            $"AlarmSession_{System.DateTime.Now:yyyyMMdd_HHmmss}.csv");

        currentCsvFile = new StreamWriter(currentCsvPath, false);

        // Header row
        currentCsvFile.WriteLine(
            "SimulationTime,TrajectoryStartTime,TrajectoryType,Alarm");

        currentCsvFile.Flush();

        Debug.Log($"Started alarm recording: {currentCsvPath}");
    }


    private void EndCsvSession()
    {
        if (currentCsvFile != null)
        {
            currentCsvFile.Flush();
            currentCsvFile.Close();
            currentCsvFile.Dispose();

            currentCsvFile = null;
        }

        currentCsvPath = null;

        Debug.Log("Alarm recording stopped.");
    }

    private StreamWriter currentCsvFile;
    private string currentCsvPath;

    public bool observationNoisePresent;
    public bool observationAnomaliesPresent;
    public bool motionBiasPresent;
    public bool throttleNoisePresent;
    public bool laneOscillationPresent;

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

    float simulationTime;
    CarBehaviours currentBehaviour;

    CarBehaviours lastBehaviour = CarBehaviours.None;

    private void Start()
    {
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
        simulationTime += Time.fixedDeltaTime;

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

    public void RecordAlarmTrigger()
    {
        if (currentBehaviour == CarBehaviours.Standard && recordAlarms)
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

            //save the False alarm



            //check if the current record actually exists (the only case this happens is when we just exited OOD and triggered it again before the manouvre finished)
            if (!allTrajectoryRecords.Contains(currentTrajectory))
            {
                allTrajectoryRecords.Add(currentTrajectory);
                //keep a maximum of 1000 records
                if (allTrajectoryRecords.Count > 1000)
                {
                    allTrajectoryRecords.RemoveAt(0);
                    RecordAlarms = false;
                }
            }


        }
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

        if (currentBehaviour == CarBehaviours.Standard && !predictor.ood)
        {
            TrajectoryRecord newRecord = new TrajectoryRecord();

            //time the trajectory started..
            newRecord.trajectoryStartTime = simulationTime;

            //type of trajectory..
            newRecord.type = type;

            //only add it to all records if we're in-distribution
            if (!predictor.ood)
                allTrajectoryRecords.Add(newRecord);

            //keep a maximum of 1000 records
            if (allTrajectoryRecords.Count > 1000)
            {
                allTrajectoryRecords.RemoveAt(0);
                RecordAlarms = false;
            }

            currentTrajectory = newRecord;
        }
    }

}
