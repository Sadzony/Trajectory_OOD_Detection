using TMPro;
using UnityEngine;

public class DropdownTooltip : MonoBehaviour
{
    [SerializeField] private TMP_Dropdown dropdown;

    [TextArea]
    public string[] tooltips;

    private GameObject currentDropdownList;
    private bool setup;


    private void Update()
    {
        // Find the runtime-created Dropdown List
        GameObject dropdownList = GameObject.Find("Dropdown List");

        if (dropdownList != null && dropdownList != currentDropdownList)
        {
            currentDropdownList = dropdownList;
            SetupTooltips(dropdownList);
        }
    }


    private void SetupTooltips(GameObject dropdownList)
    {
        Transform content = dropdownList
            .transform
            .Find("Viewport/Content");

        if (content == null)
        {
            Debug.LogWarning("Dropdown Content not found");
            return;
        }


        // Skip child 0 because it is the template item
        for (int i = 1; i < content.childCount; i++)
        {
            int optionIndex = i - 1;

            GameObject option = content.GetChild(i).gameObject;


            TooltipTrigger trigger =
                option.GetComponent<TooltipTrigger>();

            if (trigger == null)
                trigger = option.AddComponent<TooltipTrigger>();


            if (optionIndex < tooltips.Length)
            {
                trigger.tooltipMessage = tooltips[optionIndex];
            }
        }
    }
}