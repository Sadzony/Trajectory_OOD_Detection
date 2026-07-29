using UnityEngine;
using TMPro;
public class InputAlignmentController : InputFieldController
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

        double.TryParse(value, out double doubleValue);
        if (mode == PredictionMode.ADE || mode == PredictionMode.FDE)
        {
            predictor.cusumNoiseAlignmentEuclidean = doubleValue;
        }
        else if (mode == PredictionMode.LCSS || mode == PredictionMode.LCSSFinal)
        {
            predictor.cusumNoiseAlignmentLCSS = doubleValue;
        }
    }
}
