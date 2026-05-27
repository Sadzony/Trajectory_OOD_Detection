using UnityEngine;

public class RoadScrollScript : MonoBehaviour
{
    [SerializeField] private Transform road1;
    [SerializeField] private Transform road2;

    // Length of one road section
    [SerializeField] private float roadLength = 40f;

    // Initial camera Z offset
    [SerializeField] private float cameraStartZ = -2f;

    private Camera mainCam;

    private void Start()
    {
        mainCam = Camera.main;
        cameraStartZ = mainCam.transform.position.z;
    }

    private void Update()
    {
        float triggerPoint = roadLength + cameraStartZ;

        // If camera passed the trigger point
        if (mainCam.transform.position.z > triggerPoint)
        {
            // Determine which road is behind
            Transform backRoad;
            Transform frontRoad;

            if (road1.position.z < road2.position.z)
            {
                backRoad = road1;
                frontRoad = road2;
            }
            else
            {
                backRoad = road2;
                frontRoad = road1;
            }

            // Move the back road in front
            backRoad.position = new Vector3(
                backRoad.position.x,
                backRoad.position.y,
                frontRoad.position.z + roadLength
            );

            // Move trigger forward so it can repeat forever
            cameraStartZ += roadLength;
        }
    }
}
