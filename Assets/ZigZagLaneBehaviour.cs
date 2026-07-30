using UnityEngine;
using System.Collections.Generic;

public class ZigZagLaneBehaviour : MonoBehaviour
{
    [SerializeField] public LaneDefinedMovementQuintic vehicleController;
    [SerializeField] public Transform currentCentreLine;
    [SerializeField] public List<Transform> centreLines;

    public Vector3 leftZigZagPoint;
    public Vector3 rightZigZagPoint;
    public Vector3 nextZigZagTarget;

    [SerializeField] public float velocity;
    [SerializeField] public float acceleration;
    [SerializeField] public float heading;
    [SerializeField] public float timeToZigZag;
    private float currentTimeToZigZag;
    private float zigZagTime = 0.0f;
    private float zigZagStartX;
    private float zigZagTargetX;
    private float laneChangeInitialVelocity;

    private void OnEnable()
    {
        //select the closest centre line
        if (centreLines == null || centreLines.Count == 0)
        {
            currentCentreLine = null;
            return;
        }

        Transform closest = null;
        float closestDist = float.MaxValue;

        Vector3 position = transform.position;

        foreach (Transform centreLine in centreLines)
        {
            if (centreLine == null)
                continue;

            float dist = (new Vector3(centreLine.position.x, position.y, position.z) - position).magnitude;
            // If this line is almost the same distance as the current closest,
            // randomly decide whether to keep the existing one or use this one.
            if (dist < closestDist - 0.1f)
            {
                // Definitely closer.
                closestDist = dist;
                closest = centreLine;
            }

            else if (Mathf.Abs(dist - closestDist) < 0.1f)
            {
                if (Random.value < 0.5f)
                {
                    closestDist = dist;
                    closest = centreLine;
                }
            }
        }

        currentCentreLine = closest;

        if (currentCentreLine != null)
        {
            // Build the two zig-zag targets.
            leftZigZagPoint = currentCentreLine.position + Vector3.left * 1.35f;
            rightZigZagPoint = currentCentreLine.position + Vector3.right * 1.35f;

            int index = centreLines.IndexOf(currentCentreLine);

            switch (index)
            {
                case 0:
                    nextZigZagTarget = rightZigZagPoint;
                    break;

                case 1:
                    nextZigZagTarget = Random.value < 0.5f
                        ? leftZigZagPoint
                        : rightZigZagPoint;
                    break;

                case 2:
                    nextZigZagTarget = leftZigZagPoint;
                    break;

                default:
                    nextZigZagTarget = currentCentreLine.position;
                    break;
            }
        }

        
        heading = vehicleController.GetHeading();
        velocity = vehicleController.GetCurrentVelocity();
        acceleration = vehicleController.GetMaxAcceleration();
        currentTimeToZigZag = timeToZigZag / 2;
        zigZagTargetX = nextZigZagTarget.x;
        zigZagStartX = transform.position.x;
        zigZagTime = 0.0f;
        laneChangeInitialVelocity = velocity;
    }

    private void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;
        HandleLongitudinal(dt);
        HandleLateral(dt);
        UpdateHeading();

        vehicleController.heading = heading;
        vehicleController.velocity = velocity;
        vehicleController.acceleration = acceleration;
    }

    private void HandleLongitudinal(float dt)
    {
        float error = vehicleController.GetTargetVelocity() - velocity;

        acceleration = Mathf.Clamp(
            float.IsNaN(error / dt) ? 0f : error / dt,
            -vehicleController.GetBrakingDeceleration(),
            vehicleController.GetMaxAcceleration());

        velocity += acceleration * dt;

        velocity = Mathf.Max(0f, velocity);

        Vector3 pos = transform.position;
        pos += new Vector3(0f, 0f, velocity * dt);
        transform.position = pos;
    }

    private void HandleLateral(float dt)
    {
        zigZagTime += dt;

        float u = zigZagTime / currentTimeToZigZag;

        if (u >= 1f)
        {
            u = 1f;

            // Arrived at target, start the next zig-zag
            transform.position = new Vector3(
                zigZagTargetX,
                transform.position.y,
                transform.position.z
            );

            StartNextZigZag();

            u = 0f;
        }

        // Quintic interpolation
        float s =
            10f * u * u * u
            - 15f * u * u * u * u
            + 6f * u * u * u * u * u;

        float x = Mathf.Lerp(
            zigZagStartX,
            zigZagTargetX,
            s
        );

        Vector3 pos = transform.position;
        pos.x = x;
        transform.position = pos;
    }

    private void StartNextZigZag()
    {
        laneChangeInitialVelocity = velocity;

        // Determine which zig-zag point is currently closer.
        float leftDistance = Mathf.Abs(transform.position.x - leftZigZagPoint.x);
        float rightDistance = Mathf.Abs(transform.position.x - rightZigZagPoint.x);

        // Pick the opposite target.
        nextZigZagTarget = leftDistance < rightDistance
            ? rightZigZagPoint
            : leftZigZagPoint;


        currentTimeToZigZag = timeToZigZag;
        zigZagTime = 0.0f;
        zigZagStartX = transform.position.x;
        zigZagTargetX = nextZigZagTarget.x;
    }

    private void UpdateHeading()
    {
        // If we're basically stopped, freeze heading
        if (velocity <= 0.05f)
        {
            return;
        }

        float deltaX = zigZagTargetX - zigZagStartX;

        float u = Mathf.Clamp01(zigZagTime / currentTimeToZigZag);

        float ds =
            30f * u * u
            - 60f * u * u * u
            + 30f * u * u * u * u;

        float dxdt = (deltaX * ds) / currentTimeToZigZag;

        float plannedDzdt = laneChangeInitialVelocity;

        heading = Mathf.Atan2(dxdt, plannedDzdt);

        transform.rotation = Quaternion.Euler(
            0f,
            heading * Mathf.Rad2Deg,
            0f
        );
    }
}
