using UnityEngine;
using UnityEngine.UI;
public class ButtonsController : MonoBehaviour
{
    public LaneDefinedMovementQuintic car;
    public TrailOffBehaviour traillOffBehaviour;
    public ZigZagLaneBehaviour zigZagLaneBehaviour;
    public ZigZagRoadBehaviour zigZagRoadBehaviour;
    public Button DefaultBehaviourButton;
    public Button IntermittentOODButton;
    public Button BrakingButton;
    public Button TrailOffButton;
    public Button ZigZagLaneButton;
    public Button ZigZagRoadButton;
    public Button CutButton;
    public Button LossOfControlButton;

    public void ResumeDefaultBehaviour()
    {
        traillOffBehaviour.enabled = false;
        zigZagLaneBehaviour.enabled = false;
        zigZagRoadBehaviour.enabled = false;
        car.enabled = true;
        DefaultBehaviourButton.interactable = false;
        BrakingButton.interactable = true;
        car.ResumeDriving();
    }
    public void Brake()
    {
        car.enabled = true;
        traillOffBehaviour.enabled = false;
        zigZagLaneBehaviour.enabled = false;
        zigZagRoadBehaviour.enabled = false;
        BrakingButton.interactable = false;
        DefaultBehaviourButton.interactable = true;
        car.StartBraking();
    }

    public void TriggerCutLane()
    {
        car.enabled = true;
        zigZagLaneBehaviour.enabled = false;
        zigZagRoadBehaviour.enabled = false;
        traillOffBehaviour.enabled = false;

        car.TriggerCutThroughTraffic();
    }

    public void TriggerTrailOff()
    {
        car.enabled = false;
        zigZagLaneBehaviour.enabled = false;
        zigZagRoadBehaviour.enabled = false;
        traillOffBehaviour.enabled = true;
        DefaultBehaviourButton.interactable = true;
    }
    public void TriggerZigZagLane()
    {
        car.enabled = false;
        traillOffBehaviour.enabled = false;
        zigZagRoadBehaviour.enabled = false;
        zigZagLaneBehaviour.enabled = true;
        DefaultBehaviourButton.interactable = true;
    }
    public void TriggerZigZagRoad()
    {
        car.enabled = false;
        traillOffBehaviour.enabled = false;
        zigZagLaneBehaviour.enabled = false;
        zigZagRoadBehaviour.enabled = true;
        DefaultBehaviourButton.interactable = true;
    }
}
