using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Bug1 : MonoBehaviour
{
    public GameObject Goal;
    public float r, w, speed;
    private Vector2 q_0, q_g, q_H, q_L, turning_point;
    private int motion;
    private float dist = Mathf.Infinity;
    private int cnt = 0;

    
    // Start is called before the first frame update
    void Start()
    {
        q_0 = transform.position;
        q_L = q_0;

        motion = 0; // 0: Motion-to-goal, 1: Boundary following, 2: Go to q_L
        q_g = Goal.transform.position;
    }
    // Update is called once per frame
    void Update()
    {
        switch(motion)
        {
            case 0: // Motion-to-goal
                transform.position = Vector2.MoveTowards(transform.position, q_g, speed);
                break;
            case 1: // Boundary following
                transform.RotateAround(turning_point, new Vector3(0, 0, -1), w);
                Update_q_L();
                break;
            case 2: // Go to q_L
                transform.RotateAround(turning_point, new Vector3(0, 0, -1), w);

                // q_L에 도착하면 motion을 0으로 변경
                if (Vector2.Distance(transform.position, q_L) < 0.1f)
                {
                    motion = 0;
                    Reset_q_L();
                    cnt = 0;
                }
                break;
        }
    }

    void Update_q_L()
    {
        float current_dist = Vector2.Distance(transform.position, q_g);
        if (current_dist < dist)
        {
            dist = current_dist;
            q_L = transform.position;
        }
    }

    void Reset_q_L()
    {
        q_L = q_0;
        dist = Mathf.Infinity;
    }

    void OnTriggerEnter2D(Collider2D collision)
    {
        Debug.Log("Find the Goal!!!");
        motion = 3;
    }
    
    void OnCollisionEnter2D(Collision2D other)
    {
        if (motion == 0)
        {
            motion = 1;
            q_H = transform.position;
            dist = Vector2.Distance(q_H, q_g);
        }
        Vector2 direction = (other.contacts[0].point - (Vector2)transform.position).normalized;
        
        Vector2 rotatedDirection = Quaternion.Euler(0, 0, 90) * direction;
        float rotate_x = rotatedDirection.x * r + transform.position.x;
        float rotate_y = rotatedDirection.y * r + transform.position.y;


        turning_point = new Vector2(rotate_x, rotate_y);

        cnt ++;
        if (cnt > 3 && motion == 1)
        {
            if (Vector2.Distance(transform.position, q_H) < 0.3f)
            {
                motion = 2;
            }
        }
    }

}
