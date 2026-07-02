using UnityEngine;
using UnityEngine.UI;
public class ButtonsController : MonoBehaviour
{
    public LaneDefinedMovementQuintic car;
    public Button DefaultBehaviourButton;
    public Button BrakingButton;

    public void ResumeDefaultBehaviour()
    {
        DefaultBehaviourButton.interactable = false;
        BrakingButton.interactable = true;
        car.ResumeDriving();
    }
    public void Brake()
    {
        BrakingButton.interactable = false;
        DefaultBehaviourButton.interactable = true;
        car.StartBraking();
    }
}
