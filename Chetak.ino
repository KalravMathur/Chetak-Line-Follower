#include <Motor_Shield.h>

// Ultrasonic connections
#define trigPin A2
#define echoPin 2
int distance;
long duration;
const int setDist = 12;
unsigned long lastUltrasonicTime = 0; // To track the last time the us data was fetched
const int ultrasonicInterval = 60;    // Interval between us data() calls

// motor defination
const int maxSpeed = 250;  // max speed of motor
const int baseSpeed = 150; // relative tomax speed NEED TO TEST

DCMotor Lmotor(1); // Creating instances for Motor 1
DCMotor Rmotor(2); // Creating instances for Motor 2

// ir array defination
#define NUM_SENSORS 5                                      // Number of sensors in the array
unsigned char sensorPins[NUM_SENSORS] = {13, 10, 9, 6, 5}; // pins available for IR {13, 10, 9 ,6 ,5}
const int centerPosition = ((NUM_SENSORS - 1) * 1000) / 2; // center of ir array
int sensorValues[NUM_SENSORS];                             // array to save sensor values for each iteration

// intersection conditions
#define INTERSECTION_THRESHOLD 4 // Minimum number of sensors that must read black to detect an intersection
#define INTERSECTION_DELAY 130   // milliseconds to delay in case of intersection
static bool intersectionFlag = false;

// PID parameters
const float Kp = 0.059;  // Proportional gain
const float Ki = 0.0;    // Integral gain
const float Kd = 0.0346; // Derivative gain
int lastError = 0;
float integral = 0;

void setup()
{
    // IR array
    for (int i = 0; i < NUM_SENSORS; i++) // setting up all ir pins as input
        pinMode(sensorPins[i], INPUT);

    // Ultra sonic sensor
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    distance = ultrasonicData();

    delay(2000);
}

void loop()
{
    // unblocking delay for ultrasonic sensor
    unsigned long currentTime = millis(); // fetch current time
    // Fetch distance at regular intervals
    if (currentTime - lastUltrasonicTime >= ultrasonicInterval)
    {
        lastUltrasonicTime = currentTime;
        distance = ultrasonicData(); // Fetch distance from obstacle
    }

    // obstacle detection
    if (distance <= setDist && distance > setDist - 3)
        evasiveManeuver();

    // line follow
    int position = getLinePosition();

    // all white (no line)
    if (position == -1)
        forward();

    // Intersection/blacksqare check
    else if (blackCount() > INTERSECTION_THRESHOLD)
    {
        if (!intersectionFlag) // intersection
        {
            intersectionFlag = true; // set the flag the first time we detect intersecton
            forward();
            delay(INTERSECTION_DELAY); // long enough for the the robot to pass the intersection
        }
        else // black sqare (stop condition)
        {
            stop();
            delay(5000);
        }
    }

    // follow line
    else
    {
        if (intersectionFlag) // if we detected an intersection and it wasnt a square box reset the flag
            intersectionFlag = false;

        // Calculate PID output
        int pidOutput = calculatePID(position);
        adjustMotorSpeeds(pidOutput); // adjust motor speeds according to correction
    }
}

// ir arary functions
void scanIR()
{
    for (int i = 0; i < NUM_SENSORS; i++)
        sensorValues[i] = digitalRead(sensorPins[i]); // read value from every ir pin declared in the array
}
int getLinePosition() // returns -1 if line not found, or postion from 0-4000
{
    scanIR();
    int linePosition = -1; // all white
    int i = 0;

    // finding first occurance of black (1)
    while (i < NUM_SENSORS && sensorValues[i] != 1) // terminate when we detect first occ of black or reach the end of arr
        i++;                                        // first occ of 1 saved in i

    if (i == NUM_SENSORS) // no black detected
        return linePosition;

    else if (i == NUM_SENSORS - 1) // if only last sensor detects black
        return i * 1000;
    else
    {
        if (sensorValues[i + 1] == 1)
            return i * 1000 + 500;
        else
            return i * 1000;
    }
}
// Obstacle avoidance functions
int blackCount() // returns number of many sensors reading black
{
    int bc = 0;
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        if (sensorValues[i] == 1) // black == 1
            bc++;
    }
    return bc;
}
bool onTrack() // returns true if robot is on track
{
    scanIR();             // reading sensors on every call
    if (blackCount() > 2) // checking >2 due to unintended black detections while arching
        return true;
    else
        return false;
}
long ultrasonicData() // Ultrasonic distance fetch function
{
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    duration = pulseIn(echoPin, HIGH);
    return duration / 29 / 2;
}
void evasiveManeuver(void) // smooth curvy turn
{
    stop();
    delay(10);

    // smoother apporach
    // 70 degree RIGHT turn
    Rmotor.run(BACKWARD);
    Lmotor.run(FORWARD);
    Rmotor.setSpeed(200); // adjust these based on testing
    Lmotor.setSpeed(200);
    delay(380); // delay for a 70 degree turn
    stop();
    delay(80);

    // smooth left arc turn untill back on track
    while (!onTrack())
    {
        // take a smooth curvy turn in left direction like an arc
        Rmotor.run(FORWARD);
        Lmotor.run(FORWARD);
        Rmotor.setSpeed(180); // adjust these based on testing
        Lmotor.setSpeed(95);
    }

    // to catch the line slighty turn right back to straighten up
    stop();
    delay(100);
    Rmotor.run(FORWARD);
    Lmotor.run(FORWARD);
    Rmotor.setSpeed(60);
    Lmotor.setSpeed(155);
    delay(700);
}
// line follower  funtions
int calculatePID(int position) // PID calculation
{
    int error = position - centerPosition; // error from centre()
    integral += error;
    int derivative = error - lastError;
    lastError = error;
    // Calculate PID output
    int output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    return output;
}
void adjustMotorSpeeds(int pidOutput) // Adjust motor speeds
{
    // Adjust motor speeds based on PID output
    int leftMotorSpeed = baseSpeed + pidOutput;
    int rightMotorSpeed = baseSpeed - pidOutput;

    // set right motor
    if (rightMotorSpeed < 0)
    {
        if (rightMotorSpeed < -250) // if out of bounds
            rightMotorSpeed = -250;
        Rmotor.run(BACKWARD);
        Rmotor.setSpeed(abs(rightMotorSpeed));
    }
    else
    {
        if (rightMotorSpeed > 250) // if out of bounds
            rightMotorSpeed = 250;
        Rmotor.run(FORWARD);
        Rmotor.setSpeed(rightMotorSpeed);
    }

    // Set Left Motor
    if (leftMotorSpeed < 0)
    {
        if (leftMotorSpeed < -250) // if out of bounds
            leftMotorSpeed = -250;
        Lmotor.run(BACKWARD);
        Lmotor.setSpeed(abs(leftMotorSpeed));
    }
    else
    {
        if (leftMotorSpeed > 250) // if out of bounds
            leftMotorSpeed = 250;
        Lmotor.run(FORWARD);
        Lmotor.setSpeed(leftMotorSpeed);
    }
}
void forward(void) // Forward Function
{
    Rmotor.run(FORWARD);
    Lmotor.run(FORWARD);
    Rmotor.setSpeed(baseSpeed);
    Lmotor.setSpeed(baseSpeed);
}
void stop(void) // All motors stop
{
    Rmotor.setSpeed(0);
    Lmotor.setSpeed(0);
}
