using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Bug2 : MonoBehaviour
{
    public GameObject Goal;
    public float r, w, speed;
    private Vector2 q_0, q_g, q_H, q_L, turning_point;
    private LineRenderer m_line;
    private int motion;
    private float dist;

    
    // Start is called before the first frame update
    void Start()
    {
        q_0 = transform.position;
        q_g = Goal.transform.position;

        m_line = GetComponent<LineRenderer>();
        m_line.SetPosition(0, q_0);
        m_line.SetPosition(1, q_g);

        motion = 0; // 0: Motion-to-goal, 1: Boundary following

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
                if (on_mline() && Vector2.Distance(transform.position, q_g) < Vector2.Distance(q_H, q_g))
                {
                    motion = 0;
                }
                break;
        }
    }

    void OnTriggerEnter2D(Collider2D collision)
    {
        Debug.Log("Find the Goal!!!");
        motion = 2;
    }

    bool on_mline()
    {
        float m = (q_g.y - q_0.y) / (q_g.x - q_0.x);
        float c = q_0.y - m * q_0.x;

        float yOnLine = m * transform.position.x + c;
        float epsilon = 0.3f; // 오차 허용 범위

        return Mathf.Abs(transform.position.y - yOnLine) < epsilon;
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
    }

}
