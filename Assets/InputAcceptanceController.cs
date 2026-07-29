using UnityEngine;
using TMPro;
public class InputAcceptanceController : InputFieldController
{
    private TMP_InputField inputField;

    [SerializeField] TrajectoryPredictor predictor;

    private bool updatingField = false;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        if (inputField == null)
            inputField = GetComponent<TMP_InputField>();
    }

    public override void TriggerValueChange(string value, PredictionMode mode)
    {
        currentMode = mode;
        if (updatingField)
            return;

        float.TryParse(value, out float doubleValue);
        if (mode == PredictionMode.ADE || mode == PredictionMode.FDE)
        {
            //predictor.lcssAcceptanceMagnitude = doubleValue;
        }
        else if (mode == PredictionMode.LCSS || mode == PredictionMode.LCSSFinal)
        {
            predictor.lcssAcceptanceMagnitude = doubleValue;
        }
    }
}
