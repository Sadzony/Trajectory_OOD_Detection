using System.Collections.Generic;
using UnityEngine;

public class LaneDefinedMovementQuintic : MonoBehaviour
{
    [SerializeField] private FalseAlarmRateRecorder FARrecorder;

    [SerializeField] private float vehicleLength;

    [Header("Behaviour")]
    [SerializeField] private float minCruiseTimeChangeLane = 0.0f;
    [SerializeField] private float maxCruiseTimeChangeLane = 6.0f;
    private float timeSpentCruising = 0.0f;
    private float nextLaneChangeTime;

    [Header("Longitudinal")]
    [SerializeField] private float targetLongitudinalVelocity = 20f;




   private float speedLimitMinimum = 10f;
   private float speedLimitMaximum = 100f;


    [Header("Lateral")]
    [SerializeField] private float manouvreDuration = 2.5f;
    [SerializeField] float laneSettleDuration;
    [SerializeField] private float laneAlignmentTolerance = 0.05f;
    [SerializeField] public Transform currentCentreLine;
    [SerializeField] public List<Transform> centreLines;
    [SerializeField] public List<Transform> separationLines;

    [Header("Lane Changing Noise")]
    [SerializeField] public bool addMotionBias = false;
    [SerializeField] public float motionBias = 1f;
    [SerializeField] private float motionBiasMin = 0.75f;
    [SerializeField] private float motionBiasMax = 1.25f;

    [Header("Cruising Noise")]
    [SerializeField] public bool oscillate = false;
    [SerializeField] private float cruiseOscillationPeriod = 5f;
    [SerializeField] float cruiseOscillationMagnitude;


    private float cruiseStartTime;


    private float currentCruiseOscillationMagnitude;
    
private float targetCruiseOscillationMagnitude;


    [Header("Braking")]
    [SerializeField] private float brakingDeceleration = 8f;
    [SerializeField] private float stopThreshold = 0.05f;
    private float laneChangeInitialVelocity;

    [Header("Throttle Control")]
    [SerializeField] public bool throttleNoiseOn = false;
    [SerializeField] private float maxLongitudinalAcceleration = 3f;
    [SerializeField] private float coastingDeceleration = 1f;
    [SerializeField] private float throttleOffBelowMin = 2f;
    [SerializeField] private float throttleOffBelowMax = 6f;
    [SerializeField] private float throttleOffAboveMin = 1f;
    [SerializeField] private float throttleOffAboveMax = 4f;

    private bool throttlePressed = false;
    private float throttleOnVelocity;
    private float throttleOffVelocity;
    private float previousTargetLongitudinalVelocity;

    [Header("Current Values")]
    [SerializeField] public float heading;

    [SerializeField] public float velocity;
    [SerializeField] public float acceleration;

    public bool cuttingLanes = false;
    Transform preCuttingLane;

    //accessor functions
    public float GetManouvreDuration() => manouvreDuration;
    public float GetTargetVelocity() => targetLongitudinalVelocity;

    public float GetMaxAcceleration() => maxLongitudinalAcceleration;
    public float GetCurrentVelocity() => velocity;
    public float GetCurrentAcceleration() => acceleration;
    public float GetHeading() => heading;

    public float GetVehicleLength() => vehicleLength;

    public float GetSpeedLimitMin() => speedLimitMinimum;
    public float GetSpeedLimitMax() => speedLimitMaximum;

    public float GetBrakingDeceleration() => brakingDeceleration;

    public float GetLaneSettleDuration() => laneSettleDuration;

    public float GetTimeSpentCruising() => timeSpentCruising;

    public void SetTargetVelocity(float velocityMs)
    {
        targetLongitudinalVelocity = velocityMs;
    }

    public enum LongitudinalState
    {
        Cruise,
        Braking,
        Stopped,
    }
    public LongitudinalState longitudinalState = LongitudinalState.Cruise;

    public enum LateralState
    {
        Cruise,
        LaneChange,
    }

    public LateralState lateralState;

    private float laneChangeTime;
    private float laneChangeStartX;
    private float laneChangeTargetX;

    private bool laneToCutNeedsFinding = false;
    private bool cutLaneEntered = true;

    private void OnEnable()
    {
        //select the closest centre line
        if (centreLines == null || centreLines.Count == 0)
        {
            currentCentreLine = null;
            return;
        }

        Transform closest = null;
        float closestDist = float.MaxValue;

        Vector3 position = transform.position;

        foreach (Transform centreLine in centreLines)
        {
            if (centreLine == null)
                continue;

            float dist = (new Vector3(centreLine.position.x, position.y, position.z) - position).magnitude;
            // If this line is almost the same distance as the current closest,
            // randomly decide whether to keep the existing one or use this one.
            if (dist < closestDist - 0.1f)
            {
                // Definitely closer.
                closestDist = dist;
                closest = centreLine;
            }

            else if (Mathf.Abs(dist - closestDist) < 0.1f)
            {
                if (Random.value < 0.5f)
                {
                    closestDist = dist;
                    closest = centreLine;
                }
            }
        }

        // Reset timer.
        timeSpentCruising = 0f;
        nextLaneChangeTime = Random.Range(
            minCruiseTimeChangeLane,
            maxCruiseTimeChangeLane);

        currentCentreLine = closest;

    }
    private void OnDisable()
    {
        longitudinalState = LongitudinalState.Cruise;
        lateralState = LateralState.Cruise;
        // Reset timer.
        timeSpentCruising = 0f;
        nextLaneChangeTime = Random.Range(
            minCruiseTimeChangeLane,
            maxCruiseTimeChangeLane);
    }
    private void Start()
    {
        velocity = targetLongitudinalVelocity;
        previousTargetLongitudinalVelocity = targetLongitudinalVelocity;

        speedLimitMinimum = KilometresPerHourToMetresPerSecond(10.0f) - KilometresPerHourToMetresPerSecond(throttleOffBelowMax);
        speedLimitMaximum = targetLongitudinalVelocity + KilometresPerHourToMetresPerSecond(throttleOffAboveMax);

        heading = transform.eulerAngles.y * Mathf.Deg2Rad;
        cruiseStartTime = Time.time;
        currentCruiseOscillationMagnitude = cruiseOscillationMagnitude;
        targetCruiseOscillationMagnitude = cruiseOscillationMagnitude;

        longitudinalState = LongitudinalState.Cruise;

        FARrecorder.RecordStartedTrajectory(FalseAlarmRateRecorder.TrajectoryType.LaneFollow);

        nextLaneChangeTime = Random.Range(
            minCruiseTimeChangeLane,
            maxCruiseTimeChangeLane);

        ChooseNextThrottleTargets();
        UpdateLongitudinal(0);
    }

    private void FixedUpdate()
    {
        speedLimitMinimum = KilometresPerHourToMetresPerSecond(10.0f) - KilometresPerHourToMetresPerSecond(throttleOffBelowMax); ;
        speedLimitMaximum = targetLongitudinalVelocity + KilometresPerHourToMetresPerSecond(throttleOffAboveMax);

        float dt = Time.fixedDeltaTime;

        UpdateLongitudinal(dt);
        UpdateLateralState();
        UpdateLateralTrajectory(dt);
        UpdateHeading();

        if(longitudinalState == LongitudinalState.Cruise && lateralState == LateralState.Cruise)
        {
            PerformRandomLaneChanges(dt);
        }
    }


    //longitudinal motion is independent from lateral trajectories - simple velocity controller
    private void UpdateLongitudinal(float dt)
    {
        //check if our throttle targets are correct regarding target velocity
        if (!Mathf.Approximately(targetLongitudinalVelocity, previousTargetLongitudinalVelocity))
        {
            ChooseNextThrottleTargets();
            previousTargetLongitudinalVelocity = targetLongitudinalVelocity;
        }

        switch (longitudinalState)
        {
            case LongitudinalState.Cruise:
                {
                    if (throttleNoiseOn)
                    {
                        var brakingThreshold = throttleOffVelocity + KilometresPerHourToMetresPerSecond(5);
                        if (velocity > brakingThreshold)
                        {
                            // Too fast: apply braking deceleration
                            acceleration = -brakingDeceleration;
                        }
                        else
                        {

                            // Kind of simulated Throttle control
                            if (velocity <= throttleOnVelocity)
                            {
                                throttlePressed = true;
                            }
                            else if (velocity >= throttleOffVelocity)
                            {
                                throttlePressed = false;
                                ChooseNextThrottleTargets();
                            }

                            acceleration = throttlePressed
                                ? maxLongitudinalAcceleration
                                : -coastingDeceleration;
                        }
                        float error = targetLongitudinalVelocity - velocity;

                        if (targetLongitudinalVelocity < 3 && Mathf.Abs(error) < 3)
                            acceleration *= 0.2f;

                    }
                    else
                    {
                        float error = targetLongitudinalVelocity - velocity;

                        
                        var brakingThreshold = targetLongitudinalVelocity + KilometresPerHourToMetresPerSecond(5);
                        //if (velocity > brakingThreshold)
                        //{
                            // Too fast: apply braking deceleration
                        //    acceleration = Mathf.Clamp(
                        //    float.IsNaN(error / dt) ? 0f : error / dt,
                        //    -brakingDeceleration,
                        //    brakingDeceleration);
                        //}
                        //else
                        //{
                            acceleration = Mathf.Clamp(
                                float.IsNaN(error / dt) ? 0f : error / dt,
                                -brakingDeceleration,
                                maxLongitudinalAcceleration);
                        //}
                    }
                    break;
                }

            case LongitudinalState.Braking:
                {
                    acceleration = -brakingDeceleration;
                    break;
                }

            case LongitudinalState.Stopped:
                {
                    acceleration = 0f;
                    velocity = 0f;
                    break;
                }
        }

        velocity += acceleration * dt;

        if (longitudinalState == LongitudinalState.Braking &&
            velocity <= stopThreshold)
        {
            velocity = 0f;
            acceleration = 0f;
            longitudinalState = LongitudinalState.Stopped;
        }

        velocity = Mathf.Max(0f, velocity);

        Vector3 pos = transform.position;
        pos += new Vector3(0f, 0f, velocity * dt);
        transform.position = pos;
    }


    //finds out if we should engage a lane change
    private void UpdateLateralState()
    {
        float lateralError = currentCentreLine.position.x - transform.position.x;

        if (lateralState == LateralState.Cruise)
        {
            if (Mathf.Abs(lateralError) > laneAlignmentTolerance)
            {
                StartLaneChange();
            }
        }
    }

    private void StartLaneChange()
    {
        lateralState = LateralState.LaneChange;

        laneChangeTime = 0f;
        laneChangeStartX = transform.position.x;
        laneChangeTargetX = currentCentreLine.position.x;

        laneChangeInitialVelocity = velocity;
        FARrecorder.RecordStartedTrajectory(FalseAlarmRateRecorder.TrajectoryType.LaneChange);
    }


    //quintic change lane trajectory
    private void UpdateLateralTrajectory(float dt)
    {
        if (lateralState != LateralState.LaneChange)
        {
            // Cruise: lock to lane center
            Vector3 pos = transform.position;

            //oscillate within lane
            if (oscillate)
            {
                currentCruiseOscillationMagnitude = Mathf.MoveTowards(
                                                currentCruiseOscillationMagnitude,
                                                targetCruiseOscillationMagnitude,
                                                (0.1f) * dt);


                float elapsed = Time.time - cruiseStartTime;

                float phase =
                    (2f * Mathf.PI * elapsed)
                    / cruiseOscillationPeriod;

                float offset =
                    currentCruiseOscillationMagnitude * Mathf.Sin(phase);

                pos.x = currentCentreLine.position.x + offset;
            }
            transform.position = pos;
        }
        else
        {
            float speedScale = 1.0f;
            if (velocity < laneChangeInitialVelocity && (longitudinalState == LongitudinalState.Braking || longitudinalState == LongitudinalState.Stopped))
            {
                speedScale = velocity > stopThreshold
                    ? velocity / laneChangeInitialVelocity
                    : 0f;
            }

            laneChangeTime += dt * speedScale;

            float u = laneChangeTime / manouvreDuration;

            if (u >= 1f)
            {
                if(!cuttingLanes)
                {
                    FARrecorder.RecordStartedTrajectory(FalseAlarmRateRecorder.TrajectoryType.LaneFollow);
                }
                if (!cutLaneEntered)
                    cutLaneEntered = true;
                else if (cuttingLanes)
                    cuttingLanes = false;
                u = 1f;
                cruiseStartTime = Time.time;
                lateralState = LateralState.Cruise;
            }

            u = Mathf.Pow(u, 1f / motionBias);

            // Quintic step
            float s =
                10f * u * u * u
                - 15f * u * u * u * u
                + 6f * u * u * u * u * u;

            float deltaX = laneChangeTargetX - laneChangeStartX;

            float x = laneChangeStartX + deltaX * s;

            Vector3 posFinal = transform.position;
            posFinal.x = x;
            transform.position = posFinal;
        }
    }


    //heading from lateral motion
    private void UpdateHeading()
    {
        // If we're basically stopped, freeze heading
        if (velocity <= stopThreshold)
        {
            return;
        }

        float deltaX = laneChangeTargetX - laneChangeStartX;

        float u = Mathf.Clamp01(laneChangeTime / manouvreDuration);

        float ds =
            30f * u * u
            - 60f * u * u * u
            + 30f * u * u * u * u;

        float dxdt = (deltaX * ds) / manouvreDuration;

        float plannedDzdt = laneChangeInitialVelocity;

        heading = Mathf.Atan2(dxdt, plannedDzdt);

        transform.rotation = Quaternion.Euler(
            0f,
            heading * Mathf.Rad2Deg,
            0f
        );
    }

    public void StartBraking()
    {
        if (longitudinalState != LongitudinalState.Stopped)
        {
            targetCruiseOscillationMagnitude = 0.0f;
            longitudinalState = LongitudinalState.Braking;
        }
    }

    public void ResumeDriving()
    {
        timeSpentCruising = 0.0f;
        nextLaneChangeTime = Random.Range(
            minCruiseTimeChangeLane,
            maxCruiseTimeChangeLane);
        targetCruiseOscillationMagnitude = cruiseOscillationMagnitude;
        longitudinalState = LongitudinalState.Cruise;
    }

    private void PerformRandomLaneChanges(float dt)
    {
        // Only count time while cruising laterally.
        if (lateralState != LateralState.Cruise)
        {
            timeSpentCruising = 0f;
            return;
        }

        timeSpentCruising += dt;

        if (timeSpentCruising < nextLaneChangeTime || timeSpentCruising < laneSettleDuration)
            return;

        // Choose a neighbouring lane.
        int currentIndex = centreLines.IndexOf(currentCentreLine);

        List<int> possibleIndices = new List<int>();

        if (currentIndex > 0)
            possibleIndices.Add(currentIndex - 1);

        if (currentIndex < centreLines.Count - 1)
            possibleIndices.Add(currentIndex + 1);

        if (possibleIndices.Count == 0)
            return;

        int chosenIndex = possibleIndices[Random.Range(0, possibleIndices.Count)];

        if (laneToCutNeedsFinding)
        {
            chosenIndex = centreLines.IndexOf(preCuttingLane);
            laneToCutNeedsFinding = false;
        }

        currentCentreLine = centreLines[chosenIndex];

        // Randomise lane-change profile.
        if (Random.value < 0.5f)
        {
            // 50% chance of a slower profile
            motionBias = Random.Range(motionBiasMin, 1f);
        }
        else
        {
            // 50% chance of a faster profile
            motionBias = Random.Range(1f, motionBiasMax);
        }
        if (!addMotionBias)
            motionBias = 1.0f;

        // Reset timer.
        timeSpentCruising = 0f;
        nextLaneChangeTime = Random.Range(
            minCruiseTimeChangeLane,
            maxCruiseTimeChangeLane);
    }

    private void ChooseNextThrottleTargets()
    {
        throttleOnVelocity =
            targetLongitudinalVelocity -
            Random.Range(KilometresPerHourToMetresPerSecond(throttleOffBelowMin), KilometresPerHourToMetresPerSecond(throttleOffBelowMax));

        throttleOffVelocity =
            targetLongitudinalVelocity +
            Random.Range(KilometresPerHourToMetresPerSecond(throttleOffAboveMin), KilometresPerHourToMetresPerSecond(throttleOffAboveMax));
    }


    public float MetresPerSecondToKilometresPerHour(float metresPerSecond)
    {
        return metresPerSecond * 3.6f;
    }

    public float KilometresPerHourToMetresPerSecond(float kilometresPerHour)
    {
        return kilometresPerHour / 3.6f;
    }
    public void TriggerCutThroughTraffic()
    {
        //select closest separation line
        //select the closest centre line
        if (separationLines == null || separationLines.Count == 0)
        {
            currentCentreLine = null;
            return;
        }

        Transform closest = null;
        float closestDist = float.MaxValue;

        Vector3 position = transform.position;

        foreach (Transform line in separationLines)
        {
            if (line == null)
                continue;

            float dist = (new Vector3(line.position.x, position.y, position.z) - position).magnitude;

            // If this line is almost the same distance as the current closest,
            // randomly decide whether to keep the existing one or use this one.
            if (dist < closestDist - 0.1f)
            {
                // Definitely closer.
                closestDist = dist;
                closest = line;
            }

            else if (Mathf.Abs(dist - closestDist) < 0.1f)
            {
                if (Random.value < 0.5f)
                {
                    closestDist = dist;
                    closest = line;
                }
            }
            
        }
        cuttingLanes = true;
        cutLaneEntered = false;
        laneToCutNeedsFinding = true;
        preCuttingLane = currentCentreLine;

        // Reset timer.
        timeSpentCruising = 0.0f;
        nextLaneChangeTime = Random.Range(
            minCruiseTimeChangeLane,
            maxCruiseTimeChangeLane);

        currentCentreLine = closest;
    }
}
