using TMPro;
using UnityEngine;

public class SpeedInputController : MonoBehaviour
{
    [SerializeField] private TMP_InputField inputField;
    [SerializeField] private LaneDefinedMovementQuintic carBehaviour;

    private bool updatingField = false;

    private void Start()
    {
        // Convert m/s -> km/h and display as integer
        //int speedKmh = Mathf.RoundToInt(carBehaviour.GetTargetVelocity() * 3.6f);

        //inputField.text = speedKmh.ToString();
        OnSpeedChanged(inputField.text);
        inputField.onValueChanged.AddListener(OnSpeedChanged);
    }

    private void OnDestroy()
    {
        inputField.onValueChanged.RemoveListener(OnSpeedChanged);
    }
    private void Awake()
    {
        inputField.contentType = TMP_InputField.ContentType.IntegerNumber;
    }

    public void OnSpeedChanged(string value)
    {
        if (updatingField)
            return;

        if (int.TryParse(value, out int speedKmh))
        {
            speedKmh = Mathf.Max(speedKmh, 10);

            updatingField = true;
            inputField.text = speedKmh.ToString();
            updatingField = false;

            float speedMs = speedKmh / 3.6f;

            carBehaviour.SetTargetVelocity(speedMs);
        }
    }
}
