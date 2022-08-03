using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FingerController : MonoBehaviour
{
    public void Rotate(float x, float y, float z)
    {
        transform.localRotation = Quaternion.Euler(x, transform.localEulerAngles.y, transform.localEulerAngles.z);
    }
}
