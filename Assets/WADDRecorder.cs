using UnityEngine;
using System.Collections.Generic;
using System.IO;
using UnityEngine.UI;
using System.Linq;

public class WADDRecorder : MonoBehaviour
{
    [SerializeField] Toggle waddToggle;
    public float lastStableCUSUMTime = 0.0f;

    private bool recordWadd = false;

    public bool RecordWadd
    {
        get => recordWadd;
        set
        {
            if (recordWadd == value)
                return;

            recordWadd = value;

            waddToggle.isOn = value;

            OnRecordWaddChanged();
        }
    }

    float simulationTime;

    CarBehaviours currentBehaviour;
    CarBehaviours lastBehaviour = CarBehaviours.None;

    private string currentSessionFolder;

    private StreamWriter summaryCsvFile;
    private StreamWriter oodManouvresCsvFile;

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

    private float brakeWadd = 0.0f;
    private float trailOffWadd = 0.0f;
    private float zigZagLaneWadd = 0.0f;
    private float zigZagRoadWadd = 0.0f;
    private float cuttingWadd = 0.0f;
    private float lossOfControlWadd = 0.0f;
    private float overallWadd = 0.0f;

    private float brakeAverageDifference = 0.0f;
    private float trailOffAverageDifference = 0.0f;
    private float zigZagLaneAverageDifference = 0.0f;
    private float zigZagRoadAverageDifference = 0.0f;
    private float cuttingAverageDifference = 0.0f;
    private float lossOfControlAverageDifference = 0.0f;
    private float overallAverageDifference = 0.0f;
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


    private void OnRecordWaddChanged()
    {
        // Clear old session data
        OODRecords.Clear();
        brakeTotal = 0;
        cuttingTotal = 0;
        trailOffTotal = 0;
        zigZagLaneTotal = 0;
        zigZagRoadTotal = 0;
        lossOfControlTotal = 0;
        totalOODManouvres = 0;

        lastStableCUSUMTime = 0.0f;
        lastOODSimTime = 0.0f;

        brakeWadd = 0.0f;
        trailOffWadd = 0.0f;
        zigZagLaneWadd = 0.0f;
        zigZagRoadWadd = 0.0f;
        cuttingWadd = 0.0f;
        lossOfControlWadd = 0.0f;
        overallWadd = 0.0f;


        if (recordWadd)
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
            "CUSUM Threshold,CUSUM Noise Alignment,Error Measure Method,Total Manouvres,WADD (Overall),Average Difference (Overall),Total Brake Manouvres,WADD Brake,Average Difference (Brake),Total Cutting Lane Manouvres,WADD Cutting Lane,Average Difference (Cutting),Total Trail-Off Manouvres,WADD Trail-Off,Average Difference (Trail-Off),Total Zig-Zag Lane Manouvres,WADD Zig-Zag Lane,Average Difference (Zig-Zag Lane),Total Zig-Zag Road Manouvres,WADD Zig-Zag Road,Average Difference (Zig-Zag Road),Total Loss-Of-Control Manouvres,WADD Loss-Of-Control,Average Difference (Loss-Of-Control),Observation Noise Present,Observation Anomalies Present,Motion Bias Present,Throttle Noise Present,Lane Oscillation Present");

        summaryCsvFile.Flush();



        // Trajectories file
        string oodManouvres = Path.Combine(
            currentSessionFolder,
            "OODManouvres.csv");

        oodManouvresCsvFile = new StreamWriter(oodManouvres, false);

        oodManouvresCsvFile.WriteLine(
            "OOD Type,Manouvre Start Time,Alarm Time,Time Within Manouvre,CUSUM Instability Start Time,Instability Difference to Manouvre Beginning,Time Until Alarm Raised,");

        oodManouvresCsvFile.Flush();


        Debug.Log($"Started wadd recording session: {currentSessionFolder}");
    }


    private void EndCsvSession()
    {
        summaryCsvFile?.Dispose();
        oodManouvresCsvFile?.Dispose();

        summaryCsvFile = null;
        oodManouvresCsvFile = null;

        currentSessionFolder = null;

        Debug.Log("Wadd recording stopped.");
    }

    private string GetSessionFolder()
    {
        string folder;

        if (Application.isEditor)
        {
            folder = Path.Combine(
                Directory.GetParent(Application.dataPath).FullName,
                "WADDSessions");
        }
        else
        {
            folder = Path.Combine(
                Directory.GetParent(Application.dataPath).FullName,
                "WADDSessions");
        }

        Directory.CreateDirectory(folder);

        return folder;
    }


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

    private void FixedUpdate()
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

        if (lastBehaviour == CarBehaviours.Standard && currentBehaviour != CarBehaviours.Standard)
            RecordOODManouvre(currentBehaviour);

        lastBehaviour = currentBehaviour;
    }


    private class OODRecord
    {
        public CarBehaviours oodType;
        public float timeOfManouvre;
        public float timeOfAlarm;
        public float durationInManouvre;
        public double currentCUSUMThreshold;
        public double currentCUSUMNoiseAlignment;
        public PredictionMode errorMeasureMethod;

        public float instabilityStartTime;
        public float timeUntilAlarmRaised;

        public float differenceFromManouvreTime;

        public bool observationNoisePresent;
        public bool observationAnomaliesPresent;
        public bool motionBiasPresent;
        public bool throttleNoisePresent;
        public bool laneOscillationPresent;
    }

    private List<OODRecord> OODRecords = new List<OODRecord>();
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

        if (lastBehaviour == CarBehaviours.Standard && currentBehaviour != CarBehaviours.Standard)
            RecordOODManouvre(currentBehaviour);

        lastBehaviour = currentBehaviour;

        if(currentBehaviour != CarBehaviours.Standard && recordWadd && summaryCsvFile != null && oodManouvresCsvFile != null && currentSessionFolder != null && currentBehaviour != CarBehaviours.None)
        {
            var newOODRecord = new OODRecord();

            newOODRecord.oodType = currentBehaviour;

            newOODRecord.timeOfManouvre = lastOODSimTime;
            newOODRecord.timeOfAlarm = simulationTime;
            newOODRecord.durationInManouvre = simulationTime - lastOODSimTime;

            newOODRecord.currentCUSUMThreshold =
                (predictor.predictionMode == PredictionMode.ADE ||
                 predictor.predictionMode == PredictionMode.FDE)
                ? predictor.OODThresholdEuclidean
                : predictor.OODThresholdLCSS;

            newOODRecord.currentCUSUMNoiseAlignment =
                (predictor.predictionMode == PredictionMode.ADE ||
                 predictor.predictionMode == PredictionMode.FDE)
                ? predictor.cusumNoiseAlignmentEuclidean
                : predictor.cusumNoiseAlignmentLCSS;

            newOODRecord.errorMeasureMethod = predictor.predictionMode;

            newOODRecord.instabilityStartTime = lastStableCUSUMTime;
            newOODRecord.timeUntilAlarmRaised =
                simulationTime - lastStableCUSUMTime;

            newOODRecord.differenceFromManouvreTime =
                lastStableCUSUMTime - lastOODSimTime;


            newOODRecord.observationNoisePresent = observationNoisePresent;
            newOODRecord.observationAnomaliesPresent = observationAnomaliesPresent;
            newOODRecord.motionBiasPresent = motionBiasPresent;
            newOODRecord.throttleNoisePresent = throttleNoisePresent;
            newOODRecord.laneOscillationPresent = laneOscillationPresent;

            OODRecords.Add(newOODRecord);

            // Save OOD record
            oodManouvresCsvFile.WriteLine(
                $"{newOODRecord.oodType}," +
                $"{newOODRecord.timeOfManouvre:F5}," +
                $"{newOODRecord.timeOfAlarm:F5}," +
                $"{newOODRecord.durationInManouvre:F5}," +
                $"{newOODRecord.instabilityStartTime:F5}," +
                $"{newOODRecord.differenceFromManouvreTime:F5}," +
                $"{newOODRecord.timeUntilAlarmRaised:F5}");

            oodManouvresCsvFile.Flush();

            // Update running WADDs
            switch (newOODRecord.oodType)
            {
                case CarBehaviours.Brake:
                    brakeWadd = (float)OODRecords
                        .Where(r => r.oodType == CarBehaviours.Brake)
                        .Average(r => r.timeUntilAlarmRaised);

                    brakeAverageDifference = (float)OODRecords
                        .Where(r => r.oodType == CarBehaviours.Brake)
                        .Average(r => r.differenceFromManouvreTime);
                    break;

                case CarBehaviours.TrailOff:
                    trailOffWadd = (float)OODRecords
                        .Where(r => r.oodType == CarBehaviours.TrailOff)
                        .Average(r => r.timeUntilAlarmRaised);

                    trailOffAverageDifference = (float)OODRecords
                        .Where(r => r.oodType == CarBehaviours.TrailOff)
                        .Average(r => r.differenceFromManouvreTime);
                    break;

                case CarBehaviours.ZigZagLane:
                    zigZagLaneWadd = (float)OODRecords
                        .Where(r => r.oodType == CarBehaviours.ZigZagLane)
                        .Average(r => r.timeUntilAlarmRaised);

                    zigZagLaneAverageDifference = (float)OODRecords
                        .Where(r => r.oodType == CarBehaviours.ZigZagLane)
                        .Average(r => r.differenceFromManouvreTime);
                    break;

                case CarBehaviours.ZigZagRoad:
                    zigZagRoadWadd = (float)OODRecords
                        .Where(r => r.oodType == CarBehaviours.ZigZagRoad)
                        .Average(r => r.timeUntilAlarmRaised);

                    zigZagRoadAverageDifference = (float)OODRecords
                        .Where(r => r.oodType == CarBehaviours.ZigZagRoad)
                        .Average(r => r.differenceFromManouvreTime);
                    break;

                case CarBehaviours.Cut:
                    cuttingWadd = (float)OODRecords
                        .Where(r => r.oodType == CarBehaviours.Cut)
                        .Average(r => r.timeUntilAlarmRaised);

                    cuttingAverageDifference = (float)OODRecords
                        .Where(r => r.oodType == CarBehaviours.Cut)
                        .Average(r => r.differenceFromManouvreTime);
                    break;

                case CarBehaviours.LoseControl:
                    lossOfControlWadd = (float)OODRecords
                        .Where(r => r.oodType == CarBehaviours.LoseControl)
                        .Average(r => r.timeUntilAlarmRaised);

                    lossOfControlAverageDifference = (float)OODRecords
                        .Where(r => r.oodType == CarBehaviours.LoseControl)
                        .Average(r => r.differenceFromManouvreTime);
                    break;
            }

            overallWadd = (float)OODRecords
                .Average(r => r.timeUntilAlarmRaised);

            overallAverageDifference = (float)OODRecords
                .Average(r => r.differenceFromManouvreTime);

            // Update Summary.csv
            UpdateSummaryFile();


            //Make sure we stop recording after reaching 100
            if (totalOODManouvres >= 100)
                RecordWadd = false;
        }
    }

    private void UpdateSummaryFile()
    {
        if (summaryCsvFile == null || currentSessionFolder == null)
            return;

        string summaryPath = Path.Combine(
            currentSessionFolder,
            "Summary.csv");

        // Close current writer before overwriting
        summaryCsvFile.Dispose();

        summaryCsvFile = new StreamWriter(summaryPath, false);

        // Header
        summaryCsvFile.WriteLine(
            "CUSUM Threshold,CUSUM Noise Alignment,Error Measure Method,Total Manouvres,WADD (Overall),Average Difference (Overall),Total Brake Manouvres,WADD Brake,Average Difference (Brake),Total Cutting Lane Manouvres,WADD Cutting Lane,Average Difference (Cutting),Total Trail-Off Manouvres,WADD Trail-Off,Average Difference (Trail-Off),Total Zig-Zag Lane Manouvres,WADD Zig-Zag Lane,Average Difference (Zig-Zag Lane),Total Zig-Zag Road Manouvres,WADD Zig-Zag Road,Average Difference (Zig-Zag Road),Total Loss-Of-Control Manouvres,WADD Loss-Of-Control,Average Difference (Loss-Of-Control),Observation Noise Present,Observation Anomalies Present,Motion Bias Present,Throttle Noise Present,Lane Oscillation Present");

        // Summary values
        summaryCsvFile.WriteLine(
            $"{((predictor.predictionMode == PredictionMode.ADE || predictor.predictionMode == PredictionMode.FDE) ? predictor.OODThresholdEuclidean : predictor.OODThresholdLCSS):F5}," +
            $"{((predictor.predictionMode == PredictionMode.ADE || predictor.predictionMode == PredictionMode.FDE) ? predictor.cusumNoiseAlignmentEuclidean : predictor.cusumNoiseAlignmentLCSS):F5}," +
            $"{predictor.predictionMode}," +

            $"{totalOODManouvres}," +
            $"{overallWadd:F5}," +
            $"{overallAverageDifference:F5}," +

            $"{brakeTotal}," +
            $"{brakeWadd:F5}," +
            $"{brakeAverageDifference:F5}," +

            $"{cuttingTotal}," +
            $"{cuttingWadd:F5}," +
            $"{cuttingAverageDifference:F5}," +

            $"{trailOffTotal}," +
            $"{trailOffWadd:F5}," +
            $"{trailOffAverageDifference:F5}," +

            $"{zigZagLaneTotal}," +
            $"{zigZagLaneWadd:F5}," +
            $"{zigZagLaneAverageDifference:F5}," +

            $"{zigZagRoadTotal}," +
            $"{zigZagRoadWadd:F5}," +
            $"{zigZagRoadAverageDifference:F5}," +

            $"{lossOfControlTotal}," +
            $"{lossOfControlWadd:F5}," +
            $"{lossOfControlAverageDifference:F5}," +

            $"{(observationNoisePresent ? "True" : "False")}," +
            $"{(observationAnomaliesPresent ? "True" : "False")}," +
            $"{(motionBiasPresent ? "True" : "False")}," +
            $"{(throttleNoisePresent ? "True" : "False")}," +
            $"{(laneOscillationPresent ? "True" : "False")}");

        summaryCsvFile.Flush();
    }


    private int brakeTotal = 0;
    private int cuttingTotal = 0;
    private int trailOffTotal = 0;
    private int zigZagLaneTotal = 0;
    private int zigZagRoadTotal = 0;
    private int lossOfControlTotal = 0;
    private int totalOODManouvres = 0;

    private float lastOODSimTime = 0.0f;
    
    void RecordOODManouvre(CarBehaviours typeOfManouvre)
    {
        if (recordWadd && typeOfManouvre != CarBehaviours.Standard && typeOfManouvre != CarBehaviours.None)
        {
            lastOODSimTime = simulationTime;

            totalOODManouvres++;

            switch (typeOfManouvre)
            {
                case (CarBehaviours.Brake):
                    brakeTotal++;
                    break;
                case (CarBehaviours.Cut):
                    cuttingTotal++;
                    break;
                case (CarBehaviours.TrailOff):
                    trailOffTotal++;
                    break;
                case (CarBehaviours.ZigZagLane):
                    zigZagLaneTotal++;
                    break;
                case (CarBehaviours.ZigZagRoad):
                    zigZagRoadTotal++;
                    break;
                case (CarBehaviours.LoseControl):
                    lossOfControlTotal++;
                    break;
            }
        }
    }
    public void UpdateSimulationTime(float simTime)
    {
        simulationTime = simTime;
    }

    public void UpdateLastStableCUSUM()
    {
        lastStableCUSUMTime = simulationTime;
    }

}
