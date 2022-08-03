using UnityEngine;
using System.IO.Ports;
using System;

public class SerialCommunication : MonoBehaviour
{
    [SerializeField]
    private HandController hand;

    [SerializeField]
    private FingerController thumb1, thumb2, thumb3;

    [SerializeField]
    private FingerController index1, index2, index3;

    [SerializeField]
    private FingerController middle1, middle2, middle3;

    [SerializeField]
    private FingerController ring1, ring2, ring3;

    [SerializeField]
    private FingerController pinky1, pinky2, pinky3;

    SerialPort port = new SerialPort("COM7", 9600);

    private bool startMessageRecieved = false;
    void Start()
    {
        port.Open();
        port.ReadTimeout = 5000;
    }

    // Update is called once per frame
    void Update()
    {
        if (port.IsOpen)
        {
            try
            {
                string dataStr = port.ReadTo(";");
                string[] flexAndImuData = dataStr.Split("~");

                if (dataStr.Contains("@") /* || (dataStr.Contains("|") && startMessageRecieved)*/)
                {
                    if (dataStr.Contains("@")) startMessageRecieved = true;
                    Debug.Log(dataStr);
                }
                if (flexAndImuData.Length < 2) return;

                string[] dataStrArrFlex = flexAndImuData[0].Split("|");
                string[] dataStrArrIMU = flexAndImuData[1].Split("|");

                if (dataStrArrFlex.Length >= 5 && dataStrArrFlex.Length >= 3)
                {
                    float[] dataFlex = new float[dataStrArrFlex.Length];
                    float[] dataIMU = new float[dataStrArrFlex.Length];

                    for (int i = 0; i < dataStrArrFlex.Length; i++) dataFlex[i] = Math.Min(80, Math.Max(0, float.Parse(dataStrArrFlex[i])));
                    for (int i = 0; i < dataStrArrIMU.Length; i++) dataIMU[i] = Math.Min(60, Math.Max(-60, float.Parse(dataStrArrIMU[i])));

                    Debug.Log(dataIMU[0] + " " + dataIMU[1]);

                    hand.Enact(dataIMU[0], dataIMU[1]);
/*
                    thumb1.Rotate(dataFlex[0], 0, 0);
                    thumb2.Rotate(dataFlex[0], 0, 0);
                    thumb3.Rotate(dataFlex[0], 0, 0);

                    index1.Rotate(dataFlex[1], 0, 0);
                    index2.Rotate(dataFlex[1], 0, 0);
                    index3.Rotate(dataFlex[1], 0, 0);

                    middle1.Rotate(dataFlex[2], 0, 0);
                    middle2.Rotate(dataFlex[2], 0, 0);
                    middle3.Rotate(dataFlex[2], 0, 0);

                    ring1.Rotate(dataFlex[3], 0, 0);
                    ring2.Rotate(dataFlex[3], 0, 0);
                    ring3.Rotate(dataFlex[3], 0, 0);

                    pinky1.Rotate(dataFlex[4], 0, 0);
                    pinky2.Rotate(dataFlex[4], 0, 0);
                    pinky3.Rotate(dataFlex[4], 0, 0);*/
                }
            }
            catch (Exception)
            {
                Debug.Log("lmao we have a problem");
            }
        }
    }
}