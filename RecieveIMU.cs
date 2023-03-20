using System;
using System.IO;
using System.IO.Ports;
using System.Threading;
using System.Collections.Generic;
using System.Collections;
using System.Text;
using System.Numerics;
using System.Numerics.Quaternion;
using System.Numerics.Vector3;




public class ReceiveIMUValues{
    static void Main() {
        // this should be our entry point
    }

    Vector3 position;
    Vector3 rotation;
    //Rotation Offset: -27.5,-250.6,-200.2
    public Vector3 rotationOffset;
    public float speedFactor = 15.0f;
    public string imuName = "r"; // You should ignore this if there is one IMU.

    Vector3 start_pos;

    Quaternion Quat_Offset; // the offset that the quaterion will have to be multiplied by to get it in the right position
    Quaternion Current_Rotation; //Please use the variables below depending on the intention
    Quaternion Current_Uncalibrated_Rotation;
    Quaternion Current_Calibrated_Rotation;
    Quaternion Last_Calibrated_Rotation; // The last Current_Rotation (Calibrated????)
    Quaternion Start_Quat;
    Quaternion Rotation_Quaternion;

    private int recording = 0; //0= not recordingg
    private int dataWriterSet = 0;
    private StreamWriter dataWriter;

    string portSerial = "/dev/cu.portusb-0001";
    string portBT = "/dev/cu.StraightShot";

    private SerialPort port = new SerialPort("/dev/cu.StraightShot",
      115200, Parity.None, 8, StopBits.One);

    private SerialPort stream;// = new SerialPort("/dev/cu.StraightShot", 115200, Parity.None, 8, StopBits.One);
    string portName;

    public Text recordingText;
    public float maxSpeedval;
    public string maxSpeed = "";
    Vector3 current_angle;
    Vector3 prev_angle;
    //Vector3 
    double maxSpeedCalc;

    private bool useBluetooth = true;

    // Used to determine if the club is in the correct position or not
    public static float maxGreenZone = 5;
    public static float minGreenZone = -5;
    // @jacob - it doesn't like this below - see console warnings

    private bool replayActive = false;

    long sum = 0;
    int count = 0;

    //runs at the setup of Unity
    void Start() {
        //  UduinoManager.Instance.OnDataReceived += ReadIMU;
        //  Note that here, we don't use the delegate but the Events, assigned in the Inpsector Panel
        start_pos = this.transform.eulerAngles;
        Quat_Offset = new Quaternion(0, 0, 0, 1);
        Current_Rotation = new Quaternion(0, 0, 0, 1); // want to remove this variable
        Start_Quat = Quaternion.Euler(180, 0, -30);
        Current_Uncalibrated_Rotation = new Quaternion(0, 0, 0, 1); 
        Current_Calibrated_Rotation = new Quaternion(0, 0, 0, 1); 
        recordingText.text = "";
        SwingSpeed.text = "";
        ClubAngleParallel.text = "";
        ShaftAngleParallel.text = "";
        ClubAngle.text = "";
        ShaftAngle.text = "";

        if (useBluetooth)
        {
            startThread();
        }
        
    }


    public bool looping = true;

    public void StopThread()
    {
        lock (this)
        {
            looping = false;
        }
    }

    public bool IsLooping()
    {
        lock (this)
        {
            return looping;
        }
    }

    private Thread thread;
    private Queue outputQueue;    // From Unity to Arduino
    private Queue inputQueue;    // From Arduino to Unity - this is the data queue
    private int timeout = 1000;

    public void startThread()
    {
        outputQueue = Queue.Synchronized(new Queue());
        inputQueue = Queue.Synchronized(new Queue());

        Debug.Log("Thread had begun");

        thread = new Thread(ThreadLoop);
        thread.Start();
    }

    public string ReadFromArduino()
    {
        stream.ReadTimeout = timeout;
        try
        {
            return stream.ReadLine();
        }
        catch (TimeoutException e)
        {
            Debug.Log("Timeout, Cannot Read from Arduino");
            return null;
        }
    }

    public void ThreadLoop()
    {
        connectIMU();
        while (IsLooping())
        {
            // Send to Arduino
            if (outputQueue.Count != 0)
            {
                string command = (string)outputQueue.Dequeue();
                WriteToArduino(command);
            }

            // Read from Arduino
            characteristic.ValueUpdated += (o, args) =>
            {
                read10lines_and_queue();
            };
            
            string result = ReadFromArduino(1000);// timeout = 1000
            //Debug.Log(result);
            if (result != null)
                inputQueue.Enqueue(result); // NEED TO RECONFIGURE THIS TO ACCEPT 10 STRINGS
            
        }
    }

    public void SendToArduino(string command)
    {
        outputQueue.Enqueue(command);
    }

    public void WriteToArduino(string message)
    {
        stream.WriteLine(message);
        stream.BaseStream.Flush();
    }


    //This would get called once a frame in Unity, needs to be in a loop
    void Update() {

        if(inputQueue.Count > 0)
        {
            ReadIMU((string)inputQueue.Dequeue());
        }
        //Debug.Log("Queue: " + inputQueue.Count.ToString());
        if(inputQueue.Count > 20)//dumps data points
        {
            for(int i = 0; i < 15; i++)
            {
                inputQueue.Dequeue();
            }
        }

    }


    //This was a calibration function that would return our orientation to a specified orientation
    public void ResetButtonPress()
    {
        Debug.Log("Pressed");
        Debug.Log(String.Format("Current: {0}", Current_Rotation));
        // Don't look at any stored data
        inputQueue.Clear();

        //Find the rotation offset quaternion by taking inverse of current roation and multiplying it by desired rotation
        Quat_Offset = Quaternion.Euler(new Vector3(180, 0, -30)) * Quaternion.Inverse(Current_Uncalibrated_Rotation);
        Current_Calibrated_Rotation = Quat_Offset * Current_Uncalibrated_Rotation ;
        metricsInstance.reset();
        tr.Clear();
        //Debug.Log(Quat_Offset.ToString());
        
    }
    
    //Opens and writes the data out to a file
    public void RecordButtonPress()
    {
        if (recording == 1)
        {
            recording = 0;
            dataWriter.Close();
            recordingText.text = "";
        }
        else
        {
            recording = 1;

            string path = Application.streamingAssetsPath + "/Recording-" +
                    DateTime.Now.ToString("yyyy_MM_dd_HH,mm-") + ".txt";
            Debug.Log(Application.streamingAssetsPath); // where are the files saved?
            dataWriter = new StreamWriter(path, true);
            recordingText.text = "RECORDING";
        }
    }


    //This is the function that parses the data out of the string and
    //does the heavy processing of the IMU data 
    public void ReadIMU(string data){ //, UduinoDevice device) {
        Last_Calibrated_Rotation = Current_Calibrated_Rotation; // global change
        data = data.ToString();
        //Debug.Log(data);
        string[] values = data.Split('/');
        bool gyro = false;
        Quaternion New_Rotation = new Quaternion(); // this is the 
        float dt = 0;
        if (values.Length == 6) // Originally used to determine Quaternion vs. Gyro, we will have to Update to include Acceleration data
        {
            Debug.Log("Quat");
            dt = float.Parse(values[0]);
            sum += (long) dt;
            count += 1;
            //Debug.Log(dt);
            float w = float.Parse(values[1]);
            float x = float.Parse(values[2]);
            float y = float.Parse(values[3]);
            float z = float.Parse(values[4]);
            New_Rotation =  (new Quaternion(x, y, z, w));  
            Vector3 posInt = New_Rotation.eulerAngles;
            posInt.z = posInt.z * -1;
            posInt.y = posInt.y * -1;
            New_Rotation = Quaternion.Euler(posInt);
            //Debug.Log(String.Format("Uncalibrated: {0}", New_Rotation));
            Current_Uncalibrated_Rotation = New_Rotation; // the absolute Quaternion given from sensor
            New_Rotation = Quat_Offset * New_Rotation; // this should now be calibrated
            Current_Calibrated_Rotation = New_Rotation; // now save calibrated
        }

        // not sure if this is still relevant
        if (1 - Mathf.Abs(Quaternion.Dot(New_Rotation, new Quaternion())) < 0.0001){
            Debug.Log("ERROR: New_Rotation not being updated");
        }
        

        //REcords the data to a file
        if(recording == 1) // Ryan moved the record writing here so that the data that we save is calibrated
        {
            StringBuilder sb = new StringBuilder(dt.ToString("0.0000"), 50);
            sb.Append('/');
            sb.Append(Current_Calibrated_Rotation.w.ToString("0.0000"));
            sb.Append('/');
            sb.Append(Current_Calibrated_Rotation.x.ToString("0.0000"));
            sb.Append('/');
            sb.Append(Current_Calibrated_Rotation.y.ToString("0.0000"));
            sb.Append('/');
            sb.Append(Current_Calibrated_Rotation.z.ToString("0.0000"));
            sb.Append('/');
            
            dataWriter.WriteLine(sb.ToString());
        } else { // these do not need to be done if we are recording
            this.transform.localRotation = Quaternion.Lerp(this.transform.localRotation, Current_Calibrated_Rotation, Time.deltaTime * speedFactor);
            StartCoroutine(CalulateMetric(Last_Calibrated_Rotation, Current_Calibrated_Rotation, dt)); 
        }
        /*else if (values.Length != 6)
        {
            Debug.LogWarning(data);
        }*/
        this.transform.parent.transform.eulerAngles = rotationOffset;
    }



    /*Takes in gyroscope data and creates a rotation quaternion from that data
    * @param dt time in microseconds
    * @param gx - gyro_x in rad/sec
    * @param gy - "
    * @param gz - "
    * @return returns a quaternion that represents the rotation between this sample and the last sample
    */
    Quaternion GyroToQuat(float dt, float gx, float gy, float gz) {
        gy = -gy; // flip direction for unity purposes
        gz = -gz;
	    float mag = Mathf.Sqrt(gx*gx + gy*gy + gz*gz);
	    float theta = mag * dt/1000000; //microseconds to seconds
        if (theta <0.0000001 && theta > -0.0000001) { // essentially no rotation, don't rotate
            return new Quaternion();
        }
        //Debug.Log(theta);
	    float rx = gx / mag;
	    float ry = gy / mag;
	    float rz = gz / mag;

	    Quaternion rotation = new Quaternion(rx * Mathf.Sin(theta/2), ry * Mathf.Sin(theta/2), rz * Mathf.Sin(theta/2), Mathf.Cos(theta/2));
        /*
        Debug.Log(rotation);
        Debug.Log(rotation.x);
        Debug.Log(rotation.y);
        Debug.Log(rotation.z);
        Debug.Log(rotation.w);
        */
	    return rotation;
    }


    public void connectIMU()
    {
        //Look at all ports and find Straight Shot
        string[] ports = SerialPort.GetPortNames();
            
        foreach(string port in ports)
        {
            Debug.Log(port);
            if (UsingWindows && port.Contains("COM8"))
            {
                portName = port;
            }
            else if (port.Contains("StraightShot"))
            {
                portName = port;
            }
        }
            Debug.Log("Trying to connect...");
            stream = new SerialPort(portName,
            115200, Parity.None, 8, StopBits.One);
            stream.ReadTimeout = 1000;
            try
        {
            stream.Open();
            Debug.Log(String.Format("Connected to Device. Port: {0}", portName));
        }
        catch (System.Exception ex)
        {
            Debug.Log("Could not connect to Straight Shot...");
            Debug.Log(ex);
        }
    
    }

}