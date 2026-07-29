using UnityEngine;
using UnityEngine.UI;
public class ToggleAnomalyController : MonoBehaviour
{
    [SerializeField] private Toggle toggle;
    [SerializeField] private TrajectoryPredictor predictor;

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
        predictor.addObservationAnomalies = isOn;
    }
}
