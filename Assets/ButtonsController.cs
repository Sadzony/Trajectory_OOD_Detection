using UnityEngine;
using UnityEngine.UI;
using System.Collections.Generic;
using TMPro;
public class ButtonsController : MonoBehaviour
{
    public LaneDefinedMovementQuintic car;
    public TrailOffBehaviour traillOffBehaviour;
    public ZigZagLaneBehaviour zigZagLaneBehaviour;
    public ZigZagRoadBehaviour zigZagRoadBehaviour;
    public LossOfControlBehaviour lossOfControlBehaviour;
    public Button DefaultBehaviourButton;
    public Button IntermittentOODButton;
    public Button BrakingButton;
    public Button TrailOffButton;
    public Button ZigZagLaneButton;
    public Button ZigZagRoadButton;
    public Button CutButton;
    public Button LossOfControlButton;

    public TMP_InputField speedInputField;
    public Toggle intermittedSpeedToggle;

    private bool intermittentOODEnabled = false;

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

    private void Start()
    {
        intermittedSpeedToggle.onValueChanged.AddListener(OnIntermittentSpeedToggleChanged);
    }
    public void ResumeDefaultBehaviour()
    {
        traillOffBehaviour.enabled = false;
        zigZagLaneBehaviour.enabled = false;
        zigZagRoadBehaviour.enabled = false;
        lossOfControlBehaviour.enabled = false;
        car.enabled = true;
        if (!intermittentOODEnabled)
        {
            DefaultBehaviourButton.interactable = false;
            BrakingButton.interactable = true;
        }
        car.ResumeDriving();
    }
    public void Brake()
    {
        car.enabled = true;
        traillOffBehaviour.enabled = false;
        zigZagLaneBehaviour.enabled = false;
        zigZagRoadBehaviour.enabled = false;
        lossOfControlBehaviour.enabled = false;
        if (!intermittentOODEnabled)
        {
            BrakingButton.interactable = false;
            DefaultBehaviourButton.interactable = true;
        }
        car.StartBraking();
    }

    public void TriggerCutLane()
    {
        car.enabled = true;
        zigZagLaneBehaviour.enabled = false;
        zigZagRoadBehaviour.enabled = false;
        traillOffBehaviour.enabled = false;
        lossOfControlBehaviour.enabled = false;

        car.TriggerCutThroughTraffic();
    }

    public void TriggerTrailOff()
    {
        car.enabled = false;
        zigZagLaneBehaviour.enabled = false;
        zigZagRoadBehaviour.enabled = false;
        lossOfControlBehaviour.enabled = false;
        traillOffBehaviour.enabled = true;
        if (!intermittentOODEnabled)
        {
            DefaultBehaviourButton.interactable = true;
        }
    }
    public void TriggerZigZagLane()
    {
        car.enabled = false;
        traillOffBehaviour.enabled = false;
        zigZagRoadBehaviour.enabled = false;
        zigZagLaneBehaviour.enabled = true;
        if (!intermittentOODEnabled)
        {
            DefaultBehaviourButton.interactable = true;
        }
    }
    public void TriggerZigZagRoad()
    {
        car.enabled = false;
        traillOffBehaviour.enabled = false;
        zigZagLaneBehaviour.enabled = false;
        lossOfControlBehaviour.enabled = false;
        zigZagRoadBehaviour.enabled = true;
        if (!intermittentOODEnabled)
        {
            DefaultBehaviourButton.interactable = true;
        }
    }

    public void TriggerLossOfControl()
    {
        car.enabled = false;
        traillOffBehaviour.enabled = false;
        zigZagLaneBehaviour.enabled = false;
        zigZagRoadBehaviour.enabled = false;
        if (!intermittentOODEnabled)
        {
            DefaultBehaviourButton.interactable = true;
        }
        lossOfControlBehaviour.enabled = true;
    }

    private bool intermittentSpeed = false;
    [SerializeField] private float speedChangeMin;
    [SerializeField] private float speedChangeMax;
    private float timeUntilNextSpeedChange;
    private float elapsedTimeSinceLastSpeedChange;
    public void OnIntermittentSpeedToggleChanged(bool isOn)
    {
        if (isOn)
        {
            elapsedTimeSinceLastSpeedChange = 0f;
            timeUntilNextSpeedChange = Random.Range(speedChangeMin, speedChangeMax);
            speedInputField.interactable = false;
            intermittentSpeed = true;
        }
        else
        {
            elapsedTimeSinceLastSpeedChange = 0f;
            speedInputField.interactable = true;
            intermittentSpeed = false;
        }
    }

    private void Update()
    {
        float dt = Time.deltaTime;
        HandleIntermittentSpeedChanges(dt);
        if (intermittentOODEnabled)
        {
            HandleIntermittentOOD(dt);
        }
    }


    CarBehaviours nextOODBehaviour;
    bool nextBehaviourSelected = false;
    [SerializeField] float timeToOODMin;
    [SerializeField] float timeToOODMax;
    float nextTimeToOOD;
    float elapsedTimeBeforeOOD = 0.0f;
    float timeWithinOOD = 0.0f;

    CarBehaviours lastBehaviour = CarBehaviours.None;
    private void HandleIntermittentOOD(float dt)
    {
        //1. Determine the car's current behaviour

        CarBehaviours currentBehaviour;
        if(lastBehaviour == CarBehaviours.Cut && !car.cuttingLanes)
        {
            ResumeDefaultBehaviour();
            ResetOODTimer();
        }
        if (car.enabled == true && car.longitudinalState != LaneDefinedMovementQuintic.LongitudinalState.Braking && car.longitudinalState != LaneDefinedMovementQuintic.LongitudinalState.Stopped && !car.cuttingLanes)
            currentBehaviour = CarBehaviours.Standard;
        else if (car.enabled == true && (car.longitudinalState == LaneDefinedMovementQuintic.LongitudinalState.Braking || car.longitudinalState == LaneDefinedMovementQuintic.LongitudinalState.Stopped))
            currentBehaviour = CarBehaviours.Brake;
        else if (car.enabled && car.cuttingLanes)
            currentBehaviour = CarBehaviours.Cut;
        else if (car.enabled == false && traillOffBehaviour.enabled == true)
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

        if (currentBehaviour == CarBehaviours.Standard)
        {
            if (!nextBehaviourSelected)
            {
                // Pick next OOD behaviour
                nextOODBehaviour = GetNextOODBehaviour();

                nextBehaviourSelected = true;

                elapsedTimeBeforeOOD = 0f;

                nextTimeToOOD = Random.Range(
                    timeToOODMin,
                    timeToOODMax
                );
            }


            elapsedTimeBeforeOOD += dt;


            if (elapsedTimeBeforeOOD >= nextTimeToOOD)
            {
                bool readyToTrigger = true;


                // All non-braking behaviours require stable lane cruising
                if (nextOODBehaviour != CarBehaviours.Brake)
                {
                    if (car.lateralState != LaneDefinedMovementQuintic.LateralState.Cruise ||
                        car.GetTimeSpentCruising() < car.GetLaneSettleDuration())
                    {
                        readyToTrigger = false;
                    }
                }


                if (readyToTrigger)
                {
                    TriggerOODBehaviour(nextOODBehaviour);

                    elapsedTimeBeforeOOD = 0f;
                    timeWithinOOD = 0f;
                }
            }
        }
        else if(currentBehaviour == CarBehaviours.Brake || currentBehaviour == CarBehaviours.TrailOff || currentBehaviour == CarBehaviours.LoseControl)
        {
            // Only start counting once vehicle has stopped
            if (car.GetCurrentVelocity() <= 0.05f)
            {
                timeWithinOOD += dt;
            }
            else
            {
                timeWithinOOD = 0f;
            }


            if (timeWithinOOD >= 4f)
            {
                ResumeDefaultBehaviour();

                ResetOODTimer();
            }
        }
        else if(currentBehaviour == CarBehaviours.Cut)
        {
            //wait until car.cuttingLanes = false
            //reset timer
            if (!car.cuttingLanes)
            {
                ResumeDefaultBehaviour();

                ResetOODTimer();
            }
        }
        else if(currentBehaviour == CarBehaviours.ZigZagLane || currentBehaviour == CarBehaviours.ZigZagRoad)
        {
            //wait until 10 or 20 seconds passed, then return to normal and reset timer
            timeWithinOOD += dt;


            if ((currentBehaviour == CarBehaviours.ZigZagLane && timeWithinOOD >= 10f) ||
                (currentBehaviour == CarBehaviours.ZigZagRoad && timeWithinOOD >= 20f))
            {
                ResumeDefaultBehaviour();

                ResetOODTimer();
            }
        }
    }

    private void TriggerOODBehaviour(CarBehaviours behaviour)
    {
        switch (behaviour)
        {
            case CarBehaviours.Brake:
                Brake();
                break;

            case CarBehaviours.TrailOff:
                TriggerTrailOff();
                break;

            case CarBehaviours.ZigZagLane:
                TriggerZigZagLane();
                break;

            case CarBehaviours.ZigZagRoad:
                TriggerZigZagRoad();
                break;

            case CarBehaviours.Cut:
                TriggerCutLane();
                break;

            case CarBehaviours.LoseControl:
                TriggerLossOfControl();
                break;
        }
    }


    private void ResetOODTimer()
    {
        elapsedTimeBeforeOOD = 0f;
        timeWithinOOD = 0f;
        nextBehaviourSelected = false;
    }

    private void HandleIntermittentSpeedChanges(float dt)
    {
        if (!intermittentSpeed)
            return;

        elapsedTimeSinceLastSpeedChange += dt;

        if (elapsedTimeSinceLastSpeedChange >= timeUntilNextSpeedChange)
        {
            float currentSpeedKmh = car.GetTargetVelocity() * 3.6f;
            float newSpeedKmh;

            if (currentSpeedKmh < 50f)          // Currently 40
            {
                newSpeedKmh = 60f;
            }
            else if (currentSpeedKmh < 80f)     // Currently 60
            {
                newSpeedKmh = Random.value < 0.5f ? 40f : 100f;
            }
            else                                // Currently 100
            {
                newSpeedKmh = 60f;
            }

            speedInputField.text = ((int)newSpeedKmh).ToString();
            car.SetTargetVelocity(newSpeedKmh / 3.6f);

            elapsedTimeSinceLastSpeedChange = 0f;
            timeUntilNextSpeedChange = Random.Range(speedChangeMin, speedChangeMax);
        }
    }

    public void TriggerIntermittentOOD()
    {
        if(!intermittentOODEnabled)
        {
            DefaultBehaviourButton.interactable = false;
            BrakingButton.interactable = false;
            TrailOffButton.interactable = false;
            ZigZagLaneButton.interactable = false;
            ZigZagRoadButton.interactable = false;
            CutButton.interactable = false;
            LossOfControlButton.interactable = false;
            ResetOODTimer();

            IntermittentOODButton.transform.GetChild(0).GetComponent<TMP_Text>().text = "Deactivate Intermittent OOD";
            intermittentOODEnabled = true;
        }
        else if (intermittentOODEnabled)
        {
            DefaultBehaviourButton.interactable = true;
            BrakingButton.interactable = true;
            TrailOffButton.interactable = true;
            ZigZagLaneButton.interactable = true;
            ZigZagRoadButton.interactable = true;
            CutButton.interactable = true;
            LossOfControlButton.interactable = true;
            ResetOODTimer();


            IntermittentOODButton.transform.GetChild(0).GetComponent<TMP_Text>().text = "Activate Intermittent OOD";
            intermittentOODEnabled = false;
            ResumeDefaultBehaviour();
        }
    }

    private readonly List<CarBehaviours> behaviourBag = new();
    private int bagIndex = 0;

    private void RefillBehaviourBag()
    {
        behaviourBag.Clear();

        //let's test more brakes
        behaviourBag.Add(CarBehaviours.Brake);
        behaviourBag.Add(CarBehaviours.Brake);

        
        behaviourBag.Add(CarBehaviours.TrailOff);
        behaviourBag.Add(CarBehaviours.ZigZagLane);
        behaviourBag.Add(CarBehaviours.ZigZagRoad);
        behaviourBag.Add(CarBehaviours.Cut);
        behaviourBag.Add(CarBehaviours.LoseControl);

        // Fisher-Yates shuffle
        for (int i = behaviourBag.Count - 1; i > 0; i--)
        {
            int j = Random.Range(0, i + 1);
            (behaviourBag[i], behaviourBag[j]) = (behaviourBag[j], behaviourBag[i]);
        }

        bagIndex = 0;
    }

    private CarBehaviours GetNextOODBehaviour()
    {
        if (bagIndex >= behaviourBag.Count)
            RefillBehaviourBag();

        return behaviourBag[bagIndex++];
    }
}
