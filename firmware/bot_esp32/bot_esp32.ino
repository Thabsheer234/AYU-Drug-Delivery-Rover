#include <Arduino.h>

// --- 1. PIN DEFINITIONS ---
#define RXD2 16
#define TXD2 17

#define DIR1 25 // Left Motor Direction
#define PWM1 26 // Left Motor ON/OFF
#define PWM2 27 // Right Motor ON/OFF
#define DIR2 14 // Right Motor Direction

#define TRIG_PIN 5
#define ECHO_PIN 18

#define SERVO_PIN 13
#define BUZZER_PIN 19

// --- 2. STATE & TUNING VARIABLES ---
bool wantToMoveForward = false;
bool isPausedByObstacle = false;
const int SAFE_DISTANCE_CM = 15;
int msPer90Deg = 550;     // Tune this for 90-degree turn radius/timing
int pulseDurationMs = 35; // Micro-pulse duration for visual centering

unsigned long lastSensorCheck = 0;
unsigned long lastBuzzToggle = 0;
bool buzzerState = false;

// --- 3. NATIVE SERVO CONTROL ---
void setServoAngle(int angle)
{
    angle = constrain(angle, 0, 180);
    int pulseWidth = map(angle, 0, 180, 500, 2400);
    for (int i = 0; i < 25; i++)
    {
        digitalWrite(SERVO_PIN, HIGH);
        delayMicroseconds(pulseWidth);
        digitalWrite(SERVO_PIN, LOW);
        delayMicroseconds(20000 - pulseWidth);
    }
}

// --- 4. ULTRASONIC DISTANCE ---
long getDistanceCm()
{
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 25000);
    if (duration == 0)
        return 999;
    return duration * 0.034 / 2;
}

// --- 5. LOW-LEVEL MOTOR HARDWARE ---
void hardwareStop()
{
    digitalWrite(PWM1, LOW);
    digitalWrite(PWM2, LOW);
}

void hardwareForward()
{
    digitalWrite(DIR1, HIGH);
    digitalWrite(DIR2, HIGH);
    digitalWrite(PWM1, HIGH);
    digitalWrite(PWM2, HIGH);
}

void hardwareBackward()
{
    digitalWrite(DIR1, LOW);
    digitalWrite(DIR2, LOW);
    digitalWrite(PWM1, HIGH);
    digitalWrite(PWM2, HIGH);
}

void hardwareSpinLeft()
{
    digitalWrite(DIR1, LOW);
    digitalWrite(DIR2, HIGH);
    digitalWrite(PWM1, HIGH);
    digitalWrite(PWM2, HIGH);
}

void hardwareSpinRight()
{
    digitalWrite(DIR1, HIGH);
    digitalWrite(DIR2, LOW);
    digitalWrite(PWM1, HIGH);
    digitalWrite(PWM2, HIGH);
}

void beep(int times, int durationMs)
{
    for (int i = 0; i < times; i++)
    {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(durationMs);
        digitalWrite(BUZZER_PIN, LOW);
        if (i < times - 1)
            delay(durationMs);
    }
}

// --- 6. COMMAND PROCESSOR ---
void processCommand(String cmd, Stream &port)
{
    cmd.trim();
    if (cmd.length() == 0)
        return;

    if (cmd == "F" || cmd == "FORWARD")
    {
        wantToMoveForward = true;
        long d = getDistanceCm();
        if (d > 0 && d <= SAFE_DISTANCE_CM)
        {
            isPausedByObstacle = true;
            hardwareStop();
            port.println("OBSTACLE_STOP");
        }
        else
        {
            isPausedByObstacle = false;
            hardwareForward();
            port.println("ACK");
        }
    }
    else if (cmd == "B" || cmd == "BACK")
    {
        wantToMoveForward = false;
        isPausedByObstacle = false;
        digitalWrite(BUZZER_PIN, LOW);
        hardwareBackward();
        port.println("ACK");
    }
    else if (cmd == "S" || cmd == "STOP")
    {
        wantToMoveForward = false;
        isPausedByObstacle = false;
        digitalWrite(BUZZER_PIN, LOW);
        hardwareStop();
        port.println("ACK");
    }
    else if (cmd == "L" || cmd == "LEFT")
    {
        wantToMoveForward = false;
        hardwareSpinLeft();
        port.println("ACK");
    }
    else if (cmd == "R" || cmd == "RIGHT")
    {
        wantToMoveForward = false;
        hardwareSpinRight();
        port.println("ACK");
    }
    else if (cmd == "PULSE_L")
    {
        wantToMoveForward = false;
        hardwareSpinLeft();
        delay(pulseDurationMs);
        hardwareStop();
        port.println("ACK");
    }
    else if (cmd == "PULSE_R")
    {
        wantToMoveForward = false;
        hardwareSpinRight();
        delay(pulseDurationMs);
        hardwareStop();
        port.println("ACK");
    }
    else if (cmd == "PULSE_F")
    {
        wantToMoveForward = false;
        hardwareForward();
        delay(45);
        hardwareStop();
        port.println("ACK");
    }
    else if (cmd == "PULSE_B")
    {
        wantToMoveForward = false;
        hardwareBackward();
        delay(45);
        hardwareStop();
        port.println("ACK");
    }
    else if (cmd.startsWith("T:"))
    {
        wantToMoveForward = false;
        int angle = cmd.substring(2).toInt();
        int duration = (abs(angle) * msPer90Deg) / 90;
        if (angle > 0)
            hardwareSpinRight();
        else if (angle < 0)
            hardwareSpinLeft();
        delay(duration);
        hardwareStop();
        port.println("ACK");
    }
    else if (cmd.startsWith("SET_TURN_MS:"))
    {
        msPer90Deg = cmd.substring(12).toInt();
        port.println("ACK");
    }
    else if (cmd == "OPEN")
    {
        wantToMoveForward = false;
        hardwareStop();
        beep(2, 100);
        setServoAngle(90);
        port.println("ACK");
    }
    else if (cmd == "CLOSE")
    {
        setServoAngle(0);
        port.println("ACK");
    }
    else if (cmd == "BEEP")
    {
        beep(1, 200);
        port.println("ACK");
    }
    else if (cmd == "DIST")
    {
        long d = getDistanceCm();
        port.print("DIST:");
        port.println(d);
    }
}

void setup()
{
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

    pinMode(DIR1, OUTPUT);
    pinMode(PWM1, OUTPUT);
    pinMode(PWM2, OUTPUT);
    pinMode(DIR2, OUTPUT);
    hardwareStop();

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(SERVO_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    setServoAngle(0);
    beep(1, 100);
    Serial.println("ESP32 Ready");
    Serial2.println("ESP32_READY");
}

void loop()
{
    if (Serial2.available())
    {
        String piCmd = Serial2.readStringUntil('\n');
        processCommand(piCmd, Serial2);
    }
    if (Serial.available())
    {
        String usbCmd = Serial.readStringUntil('\n');
        processCommand(usbCmd, Serial);
    }

    // Check Ultrasonic Sensor every 50ms
    if (millis() - lastSensorCheck > 50)
    {
        lastSensorCheck = millis();
        long dist = getDistanceCm();

        // Obstacle detected (< 15cm)
        if (dist > 0 && dist <= SAFE_DISTANCE_CM)
        {
            if (wantToMoveForward && !isPausedByObstacle)
            {
                hardwareStop();
                isPausedByObstacle = true;
                Serial.println("OBSTACLE_STOP");
                Serial2.println("OBSTACLE_STOP");
            }
            // Continuous buzzing whenever anything is closer than 15cm
            if (millis() - lastBuzzToggle > 100)
            {
                lastBuzzToggle = millis();
                buzzerState = !buzzerState;
                digitalWrite(BUZZER_PIN, buzzerState ? HIGH : LOW);
            }
        }
        // Obstacle cleared (> 15cm)
        else
        {
            digitalWrite(BUZZER_PIN, LOW);
            buzzerState = false;
            if (wantToMoveForward && isPausedByObstacle)
            {
                isPausedByObstacle = false;
                hardwareForward();
                Serial.println("OBSTACLE_CLEAR");
                Serial2.println("OBSTACLE_CLEAR");
            }
        }
    }
}