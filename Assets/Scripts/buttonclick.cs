using UnityEngine;
using UnityEngine.UI;
using Valve.VR;

public class ControllerUIButtonClick : MonoBehaviour
{
    public Button uiButton; // Assign your UI button in the inspector
    public SteamVR_Action_Boolean uiClickAction; // Assign your UI_Click action in the inspector

    private void Update()
    {
        // Check if the controller button is pressed
        if (uiClickAction.GetStateDown(SteamVR_Input_Sources.Any))
        {
            // Simulate a click on the UI button
            if (uiButton != null)
            {
                uiButton.onClick.Invoke();
            }
        }
    }
}
