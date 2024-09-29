using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TangentBugAlgorithm : MonoBehaviour
{
    public float sensorRange;
    public float speed;
    public GameObject Goal;

    private Tuple<float, Vector2>[] sensorData;

    private bool followingBoundary;
    private float prevDistToGoal, currDistToGoal, minDistToGoal, distAtLeavePoint;
    private Vector2 initialHitPoint, lastMoveDirection;
    private int boundaryLoopCounter;

    // Start is called before the first frame update
    void Start()
    {
        sensorData = new Tuple<float, Vector2>[36]; // 36 sensors

        followingBoundary = false;
        prevDistToGoal = Mathf.Infinity;
        minDistToGoal = Mathf.Infinity;
    }

    // Update is called once per frame
    void Update()
    {
        UpdateSensors();

        // position reached to the goal
        if (Vector2.Distance(transform.position, Goal.transform.position) < 0.01f)
        {
            Debug.Log("Goal reached!");
            UnityEditor.EditorApplication.isPlaying = false;
        }

        if (followingBoundary)
        {
            BoundaryFollowing();
        }
        else
        {
            MotionToGoal();
        }
    }

    void MotionToGoal()
    {
        Vector2 moveDirection = CalculateMoveDirection();
        moveDirection.Normalize();
        transform.position = Vector2.MoveTowards(transform.position, (Vector2)transform.position + moveDirection, speed);
        lastMoveDirection = moveDirection;

        // Check if we are stuck in a local minimum
        if (prevDistToGoal < currDistToGoal)
        {
            followingBoundary = true;
            initialHitPoint = transform.position;
            boundaryLoopCounter = 0;
        }
        prevDistToGoal = currDistToGoal;
    }

    void BoundaryFollowing()
    {
        boundaryLoopCounter += 1;
        // When the robot has been following the boundary for too long and returns to the initial hit point
        if (boundaryLoopCounter > 10 && Vector2.Distance(transform.position, initialHitPoint) < 0.01f)
        {
            Debug.Log("No solution");
            UnityEditor.EditorApplication.isPlaying = false;
        }


        distAtLeavePoint = Vector2.Distance(transform.position, Goal.transform.position);
        // Find the tangent direction at the obstacle edge
        Vector2 obstacleEdgeDirection = FindObstacleEdgeDirection();
        obstacleEdgeDirection.Normalize();

        // action to motion-to-goal
        if (distAtLeavePoint < minDistToGoal)
        {
            followingBoundary = false; // Exit boundary-following
        }
        else
        {
            // Move along the tangent direction
            Vector2 tangentDirection = new Vector2(-obstacleEdgeDirection.y, obstacleEdgeDirection.x);
            transform.position = Vector2.MoveTowards(transform.position, (Vector2)transform.position + tangentDirection, speed);
            lastMoveDirection = tangentDirection;
        }
    }

    void UpdateSensors()
    {
        for (int idx = 0; idx < 36; idx++)
        {
            int angle = idx * 10;
            Vector2 direction = new Vector2(Mathf.Cos(Mathf.Deg2Rad * angle), Mathf.Sin(Mathf.Deg2Rad * angle));

            RaycastHit2D[] hits = Physics2D.RaycastAll(transform.position, direction, sensorRange);

            // Find the closest obstacle or set the sensor range if no obstacle is detected
            if (hits.Length > 1)
            {
                sensorData[idx] = new Tuple<float, Vector2>(hits[1].distance, hits[1].point);
            }
            else
            {
                sensorData[idx] = new Tuple<float, Vector2>(Mathf.Infinity, (Vector2)transform.position + sensorRange * direction);
            }

            // Visualize sensor rays in yellow
            Debug.DrawRay(transform.position, direction * sensorRange, Color.yellow);
        }
    }

    Vector2 CalculateMoveDirection()
    {
        Vector2 dirToGoal  = Goal.transform.position - transform.position;
        float minTotalDist  = Mathf.Infinity;

        // Check if there's a direct path to the goal
        RaycastHit2D[] hits = Physics2D.RaycastAll(transform.position, dirToGoal, sensorRange);
        if (hits.Length == 1 || hits[1].collider.gameObject == Goal)
        {
            currDistToGoal = dirToGoal.sqrMagnitude;
            return dirToGoal;
        }

        // Obstacle encountered: compute tangent points and find best path
        for (int idx = 0; idx < sensorData.Length; idx++)
        {
            int prev_idx = (idx - 1 + sensorData.Length) % sensorData.Length;
            int next_idx = (idx + 1) % sensorData.Length;

            // Detect edge of obstacle
            if (sensorData[idx].Item1 != Mathf.Infinity &&
                (sensorData[prev_idx].Item1 == Mathf.Infinity || sensorData[next_idx].Item1 == Mathf.Infinity))
            {
                float totalDistance = Vector2.Distance(transform.position, sensorData[idx].Item2) +
                                      Vector2.Distance(sensorData[idx].Item2, Goal.transform.position);
                if (totalDistance < minTotalDist )
                {
                    minTotalDist  = totalDistance;
                    dirToGoal = sensorData[idx].Item2 - (Vector2)transform.position;
                }
            }
        }

        currDistToGoal = minTotalDist ;
        return dirToGoal;
    }

    /*
        - Find the closest obstacle point
        - In sensorData, distance is stored in Item1 and the point is stored in Item2
        - If the sensor cannot detect any obstacle, the distance is set to Mathf.Infinity
    */
    Vector2 FindObstacleEdgeDirection()
    {
        float closestObstacleDistance = Mathf.Infinity;
        Vector2 closestObstaclePoint = Vector2.zero;
        for (int idx = 0; idx < sensorData.Length; idx++)
        {
            if (sensorData[idx].Item1 != Mathf.Infinity)
            {
                if (sensorData[idx].Item1 < closestObstacleDistance)
                {
                    closestObstacleDistance = sensorData[idx].Item1;
                    closestObstaclePoint = sensorData[idx].Item2;
                }
                float distanceToTarget = Vector2.Distance(sensorData[idx].Item2, Goal.transform.position);
                if (distanceToTarget < minDistToGoal)
                {
                    minDistToGoal = distanceToTarget;
                }
            }
        }
        return closestObstaclePoint - (Vector2)transform.position;
    }
}
