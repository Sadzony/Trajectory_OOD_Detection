using UnityEngine;
using UnityEngine.UI;

public class ToggleFalseAlarmRateRecording : MonoBehaviour
{
    [SerializeField] private Toggle toggle;
    [SerializeField] private FalseAlarmRateRecorder FARrecorder;

    private void Awake()
    {
        if (toggle == null)
            toggle = GetComponent<Toggle>();

        toggle.onValueChanged.AddListener(OnToggleChanged);
    }

    private void OnDestroy()
    {
        toggle.onValueChanged.RemoveListener(OnToggleChanged);
    }

    private void OnToggleChanged(bool isOn)
    {
        FARrecorder.RecordAlarms = isOn;
    }
}
