using UnityEngine;

public class SmoothZFollow : MonoBehaviour
{
    [SerializeField] private Transform target;
    [SerializeField] private float smoothSpeed = 5f;

    private float fixedX;
    private float fixedY;

    private float offsetZ;

    private void Start()
    {
        fixedX = transform.position.x;
        fixedY = transform.position.y;

        // Capture initial offset
        offsetZ = transform.position.z - target.position.z;
    }

    private void LateUpdate()
    {
        if (target == null) return;

        float targetZ = target.position.z + offsetZ;

        Vector3 desiredPosition = new Vector3(fixedX, fixedY, targetZ);

        transform.position = Vector3.Lerp(
            transform.position,
            desiredPosition,
            smoothSpeed * Time.deltaTime
        );
    }
}
