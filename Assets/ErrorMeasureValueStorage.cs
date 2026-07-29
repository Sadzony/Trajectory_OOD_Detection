using TMPro;
using UnityEngine;
using System.Text.RegularExpressions;

public class ErrorMeasureValueStorage : MonoBehaviour
{
    [Header("Input Field")]
    private TMP_InputField inputField;
    [SerializeField] private InputFieldController fieldController;


    [Header("Stored Values")]
    public string ADEValue;
    public string FDEValue;
    public string LCSSValue;
    public string LCSSFinalValue;

    private PredictionMode currentMode;

    private bool updatingField = false;
    private static readonly Regex numberRegex = new Regex(@"^\d*\.?\d*$");


    private void Awake()
    {
        currentMode = PredictionMode.ADE;

        if (inputField == null)
            inputField = GetComponent<TMP_InputField>();

        LoadCurrentValue();
        inputField.onValueChanged.AddListener(OnValueChanged);
    }


    private void OnDestroy()
    {
        inputField.onValueChanged.RemoveListener(OnValueChanged);
    }


    private void LoadCurrentValue()
    {
        switch (currentMode)
        {
            case PredictionMode.ADE:
                inputField.SetTextWithoutNotify(ADEValue);
                break;

            case PredictionMode.FDE:
                inputField.SetTextWithoutNotify(FDEValue);
                break;

            case PredictionMode.LCSS:
                inputField.SetTextWithoutNotify(LCSSValue);
                break;

            case PredictionMode.LCSSFinal:
                inputField.SetTextWithoutNotify(LCSSFinalValue);
                break;
        }
        fieldController.TriggerValueChange(inputField.text, currentMode);
    }

    public void SetMode(PredictionMode mode)
    {
        // Save the old value before switching
        SaveCurrentValue(currentMode);

        currentMode = mode;

        // Load the new value
        LoadCurrentValue();
    }

    private void OnValueChanged(string value)
    {
        if (updatingField)
            return;


        if (!numberRegex.IsMatch(value) && value != "") 
        {
            updatingField = true;

            // Remove the invalid character(s)
            inputField.text = Regex.Replace(value, @"[^0-9.]", "");

            updatingField = false;
        }
        else if (numberRegex.IsMatch(value) && float.TryParse(value, out float number) && value != "")
        {
            fieldController.TriggerValueChange(value, currentMode);
            SaveCurrentValue(currentMode);
        }
    }



    public void SaveCurrentValue(PredictionMode mode)
    {
        var value = inputField.text;
        if (numberRegex.IsMatch(value) && float.TryParse(value, out float number) && value != "")
        {
            switch (mode)
            {
                case PredictionMode.ADE:
                    ADEValue = inputField.text;
                    break;

                case PredictionMode.FDE:
                    FDEValue = inputField.text;
                    break;

                case PredictionMode.LCSS:
                    LCSSValue = inputField.text;
                    break;

                case PredictionMode.LCSSFinal:
                    LCSSFinalValue = inputField.text;
                    break;
            }
        }
    }
}
