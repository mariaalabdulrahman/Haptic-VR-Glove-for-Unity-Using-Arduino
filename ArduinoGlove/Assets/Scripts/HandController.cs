using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HandController : MonoBehaviour
{
    [SerializeField]
    private int xSpeed, ySpeed;

    public void Enact(float horizontalMovement, float verticalMovement)
    {
        if (horizontalMovement > 10 || horizontalMovement < -10)
        {
            if (horizontalMovement > 0 && transform.localPosition.z < 0.1f)
            {
                //transform.localRotation = Quaternion.Euler(-80 * Time.deltaTime, transform.localEulerAngles.y, transform.localEulerAngles.z);
                transform.Translate(Vector3.down * xSpeed * Time.deltaTime);
            }
            else if (horizontalMovement < 0 && transform.localPosition.z > -0.100)
            {
                //transform.localRotation = Quaternion.Euler(-130 * Time.deltaTime, transform.localEulerAngles.y, transform.localEulerAngles.z);
                transform.Translate(Vector3.up * xSpeed * Time.deltaTime);
            }
        }

        if (verticalMovement > 10 || verticalMovement < -10)
        {
            if (verticalMovement > 0 && transform.localPosition.y < 1.5f) transform.Translate(Vector3.forward * ySpeed * Time.deltaTime);
            else if (verticalMovement < 0 && transform.localPosition.y > 1.4) transform.Translate(Vector3.back * ySpeed * Time.deltaTime);
        }
    }
}
