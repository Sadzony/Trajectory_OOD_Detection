using TMPro;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.InputSystem;

public class Tooltip : MonoBehaviour
{
    public static Tooltip Instance;

    [Header("References")]
    [SerializeField] private RectTransform tooltipRect;
    [SerializeField] private TMP_Text tooltipText;

    [Header("Settings")]
    [SerializeField] private float maxTextWidth = 300f;
    [SerializeField] private Vector2 mouseOffset = new Vector2(15, -15);
    [SerializeField] private float padding = 10f;

    private RectTransform textRect;

    private void Awake()
    {
        Instance = this;

        textRect = tooltipText.rectTransform;

        Hide();
    }


    private void Update()
    {
        if (gameObject.activeSelf)
            UpdatePosition();
    }


    public void Show(string message)
    {
        gameObject.SetActive(true);

        tooltipText.text = message;

        Resize();
        UpdatePosition();
    }


    public void Hide()
    {
        gameObject.SetActive(false);
    }


    private void Resize()
    {
        // First measure as a single line
        tooltipText.textWrappingMode = TextWrappingModes.NoWrap;
        tooltipText.rectTransform.sizeDelta = new Vector2(1000, 0);

        tooltipText.ForceMeshUpdate();

        float width = tooltipText.preferredWidth;


        // Need wrapping
        if (width > maxTextWidth)
        {
            width = maxTextWidth;

            tooltipText.textWrappingMode = TextWrappingModes.Normal;
            tooltipText.rectTransform.sizeDelta =
                new Vector2(width, 0);

            tooltipText.ForceMeshUpdate();
        }


        float height = tooltipText.preferredHeight;


        // Set text box size
        textRect.sizeDelta = new Vector2(width, height);


        // Set tooltip background size
        tooltipRect.sizeDelta =
            new Vector2(
                width + padding * 2,
                height + padding * 2
            );
    }


    private void UpdatePosition()
    {
        Vector2 mousePos = Mouse.current.position.ReadValue();

        Vector2 size = tooltipRect.sizeDelta;


        float x = mousePos.x + mouseOffset.x;
        float y = mousePos.y + mouseOffset.y;


        // Right edge overflow
        if (x + size.x > Screen.width)
        {
            x = mousePos.x - size.x - mouseOffset.x;
        }


        // Bottom overflow
        if (y - size.y < 0)
        {
            y = mousePos.y + size.y;
        }


        tooltipRect.position = new Vector2(x, y);
    }
}