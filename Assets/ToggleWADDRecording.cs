using UnityEngine;
using UnityEngine.UI;

public class ToggleWADDRecording : MonoBehaviour
{
    [SerializeField] private Toggle toggle;
    [SerializeField] private WADDRecorder WADDrecorder;

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
        WADDrecorder.RecordWadd = isOn;
    }
}
