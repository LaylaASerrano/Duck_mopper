#include <Arduino.h>
#include <ESP32Servo.h>


// ESP32 #1: ultrasonic sensor, wheel and scrubber motors, emergency stop, MQTT.

// servo
Servo broom_serv, sponge1_serv, sponge2_serv;
const int BROOM_PIN = 19, SPONGE_1_PIN = 21, SPONGE_2_PIN = 15; // servos

// motors
const int AIN1 = 14, AIN2 = 12;   // left side
const int BIN1 = 32, BIN2 = 33;   // right side

const int PWM_FREQ = 20000;   // 20 kHz: above hearing, no motor whine
const int PWM_RES  = 8;       // 8-bit: duty 0..255

// sensor
const int TRIG_PIN = 22, ECHO_PIN = 23;
const float DETECT_CM = 50.0;

// timing
const unsigned long SENS_INT = 60;
const unsigned long PAUSE_TIME = 100;
const unsigned long BACKUP_TIME = 400;

// state "machine "
enum State {
    DRIVING, PAUSE_BEFORE_BACKUP, BACKUP, TURNING, PAUSE_AFTER_TURN, STOPPED
};

State state = DRIVING;
unsigned long stateStart = 0;
unsigned long turnTime = 0;
unsigned long lastSensorRead = 0;
float lastDist = -1;


// motor controls
void setMotor(int pinA, int pinB, int speed){ // -255-255
    speed = constrain(speed, -255, 255);
    if (speed > 0) {
        ledcWrite(pinA, speed);
        ledcWrite(pinB, 0);
    }else if (speed < 0) {
        ledcWrite(pinA, 0);
        ledcWrite(pinB, -speed);
    } else {
        ledcWrite(pinA, 0);
        ledcWrite(pinB, 0);
    }
}

void setLeft(int speed) { setMotor(AIN1, AIN2, speed);}
void setRight(int speed){ setMotor(BIN1, BIN2, speed);}
void drive(int l, int r){ setLeft(l); setRight(r);}

void forward() { drive(180, 180);}
void backward() { drive(-180, -180);}
void spinLeft() {drive(-180, 180);}
void spinRight() {drive(180, -180);}
void stopBot() {drive(0, 0);}


// Servo
//sponge 1
//void spinSponge(int speed){ sponge1_serv.write(90 - speed); sponge2_serv.write(90-speed);}


// sensor
float readDistCm(){
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    unsigned long us = pulseIn(ECHO_PIN, HIGH, 30000); // 30 ms = 5 m
    return (us == 0 ? -1 : us * 0.0343f / 2.0f);
}

void updateSensor(){
    if (millis() - lastSensorRead >= SENS_INT){
        lastSensorRead = millis();
        lastDist = readDistCm();
    }
}

bool obstableDetected(){
    return lastDist > 0 && lastDist < DETECT_CM;
}



void randomTurn(){
    float d = readDistCm();
    if (d > 0 && d < DETECT_CM){
        Serial.printf("Obstacle at %.1f cm\n", d);
        stopBot(); delay(100);
        backward(); delay(400);
        if(random(2)) spinLeft(); else spinRight();
        delay(random(300, 900));
        stopBot(); delay(100);
    }else{
        forward();
    }
    delay(60);
}


// servos
// Call before setupMotors(): ESP32Servo picks LEDC channels without checking what
// ledcAttach already took, so a servo attached second would share a motor's channel.
void setupServos(){
    Servo* servos[] = {&broom_serv, &sponge1_serv, &sponge2_serv};
    const int pins[] = {BROOM_PIN, SPONGE_1_PIN, SPONGE_2_PIN};
    for (int i = 0; i < 3; i++){
        servos[i]->setPeriodHertz(50);
        servos[i]->attach(pins[i], 500, 2400);
    }
}

void setupMotors() {
    int pins[] = {AIN1, AIN2, BIN1, BIN2};
    for (int p : pins) ledcAttach(p, PWM_FREQ, PWM_RES);
    stopBot();
}

void setupSensor(){
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    digitalWrite(TRIG_PIN, LOW);
}

void setup(){
    Serial.begin(115200);
    setupSensor();
    setupServos();
    setupMotors();

    Serial.println("Setup done. Wheels will spin in 3 s, lift the robot off the ground.");
    delay(3000);
}

// hardware test: runs each part in turn and prints what should be happening
void testSensor(unsigned long ms){
    Serial.println("-- sensor (-1 = no echo)");
    unsigned long start = millis();
    while (millis() - start < ms){
        updateSensor();
        Serial.printf("%.1f cm%s\n", lastDist, obstableDetected() ? "  OBSTACLE" : "");
        delay(250);
    }
}

void testMove(const char* name, void (*move)()){
    Serial.printf("-- motors: %s\n", name);
    move();
    delay(1000);
    stopBot();
    delay(500);
}

void testServo(const char* name, Servo& servo){
    Serial.printf("-- servo: %s\n", name);
    servo.write(45);  delay(600);
    servo.write(135); delay(600);
    servo.write(90);  delay(600);
}

void loop(){
    testSensor(3000);

    testMove("forward", forward);
    testMove("backward", backward);
    testMove("spin left", spinLeft);
    testMove("spin right", spinRight);

    testServo("broom", broom_serv);
    testServo("sponge 1", sponge1_serv);
    testServo("sponge 2", sponge2_serv);

    Serial.println("== cycle done ==");
    delay(2000);
}
