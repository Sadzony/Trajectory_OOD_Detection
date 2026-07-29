using UnityEngine;
using UnityEngine.UI;
public class ToggleThrottleController : MonoBehaviour
{
    [SerializeField] private Toggle toggle;
    [SerializeField] private LaneDefinedMovementQuintic vehicle;

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
        vehicle.throttleNoiseOn = isOn;
    }
}
