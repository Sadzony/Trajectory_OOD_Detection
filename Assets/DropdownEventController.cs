using TMPro;
using UnityEngine;

public class DropdownEventController : MonoBehaviour
{
    [SerializeField] private TMP_Dropdown dropdown;
    [SerializeField] private ErrorMeasureValueStorage[] inputFields;

    [SerializeField] private TrajectoryPredictor predictor;

    public PredictionMode currentMode;


    private void Start()
    {
        dropdown.onValueChanged.AddListener(ChangeMode);
    }


    private void ChangeMode(int index)
    {
        currentMode = (PredictionMode)index;
        

        foreach (ErrorMeasureValueStorage field in inputFields)
        {
            field.SetMode(currentMode);
            if(field.transform.parent.gameObject.name == "AcceptableDist" && (currentMode == PredictionMode.LCSS || currentMode == PredictionMode.LCSSFinal))
            {
                field.GetComponent<TMP_InputField>().interactable = true;
            }
            else if(field.transform.parent.gameObject.name == "AcceptableDist" && (currentMode == PredictionMode.ADE || currentMode == PredictionMode.FDE))
            {
                field.GetComponent<TMP_InputField>().interactable = false;
            }
        }
        predictor.predictionMode = currentMode;

    }

}
