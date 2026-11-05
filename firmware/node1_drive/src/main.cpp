#include <Arduino.h>
#include <atomic>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <PubSubClient.h>

// WiFi and broker settings. Copy secrets.example.h to secrets.h to enable MQTT;
// without it the robot still runs from the button.
#if __has_include("secrets.h")
#include "secrets.h"
#define HAVE_SECRETS 1
#endif

// ESP32 #1: ultrasonic sensor, wheel and scrubber motors, emergency stop, MQTT.

// ----------- Constants and structers ----------
// servo
Servo broom_serv, sponge1_serv, sponge2_serv;
const int BROOM_PIN = 19, SPONGE_1_PIN = 21, SPONGE_2_PIN = 18; // servos

// motors
const int AIN1 = 25, AIN2 = 26;   // left side NOTE: (12 breaks boot, 14 twitches at boot)
const int BIN1 = 32, BIN2 = 33;   // right side

// LEDC channels for the motors, kept away from the servos' channels so they
// never share a timer (channels share timers in pairs: 0/1, 2/3, ...)
const int MOTOR_CH[4] = {8, 9, 10, 11};

const int PWM_FREQ = 20000;   // 20 kHz: above hearing, no motor whine
const int PWM_RES  = 8;       // 8-bit: duty 0..255

// sensor
const int TRIG_PIN = 22, ECHO_PIN = 23; // ECHO needs the 5V -> 3.3V divider
const float DETECT_CM = 50.0;

// emergency stop: momentary button from this pin to GND
const int ESTOP_PIN = 27;
const unsigned long RESUME_HOLD = 2000;   // hold this long to start from STOPPED
const unsigned long RELEASE_TIME = 50;    // must read released this long (debounce)

// timing
const unsigned long SENS_INT = 60;
const unsigned long PAUSE_TIME = 100;
const unsigned long BACKUP_TIME = 400;

// state "machine "
enum State {
    DRIVING, PAUSE_BEFORE_BACKUP, BACKUP, TURNING, PAUSE_AFTER_TURN, STOPPED
};

State state = STOPPED;   // never moves on power-up: needs the button or an MQTT start
unsigned long stateStart = 0;
unsigned long turnTime = 0;
unsigned long lastSensorRead = 0;
float lastDist = -1;


// ---------------  motor control ------------

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

void setupMotors() {
    int pins[4] = {AIN1, AIN2, BIN1, BIN2};
    for (int i = 0; i < 4; i++){
        if (!ledcAttachChannel(pins[i], PWM_FREQ, PWM_RES, MOTOR_CH[i])){
            Serial.printf("LEDC attach failed on pin %d\n", pins[i]);
        }
    }
    stopBot();
}

// --------------- Servo --------------------
const unsigned long SWEEP_TIME = 400;   // ms per stroke
const int SWEEP_MIN = 30, SWEEP_MAX = 150, SWEEP_PARK = 90;

struct Sweeper {
    Servo* a;            // main servo
    Servo* b;            // optional partner, moves mirrored (nullptr if none)
    bool on;
    bool atMin;
    unsigned long lastFlip;
};

Sweeper broom  = {&broom_serv, nullptr, false, false, 0};
Sweeper sponge = {&sponge1_serv, &sponge2_serv, false, false, 0};

void writeSweeper(Sweeper& w, int angle) {
    w.a->write(angle);
    if (w.b) w.b->write(180 - angle);     // mirrored partner
}

void startSweep(Sweeper& w) {
    if (w.on) return;
    w.on = true;
    w.lastFlip = millis() - SWEEP_TIME;   // move on the very next update
}

void stopSweep(Sweeper& w) {
    w.on = false;
    writeSweeper(w, SWEEP_PARK);
}

void updateSweep(Sweeper& w) {
    if (!w.on) return;
    if (millis() - w.lastFlip >= SWEEP_TIME) {
        w.lastFlip = millis();
        w.atMin = !w.atMin;
        writeSweeper(w, w.atMin ? SWEEP_MIN : SWEEP_MAX);
    }
}

void updateScrubbers() {
    updateSweep(broom);
    updateSweep(sponge);
}

// Continuous-rotation servos: 90 = stop, further from 90 = faster.
void spongeSpin(int speed) {              // -90..90, 0 = stop
    speed = constrain(speed, -90, 90);
    sponge1_serv.write(90 + speed);
    sponge2_serv.write(90 - speed);       // mirrored mount; flip sign if needed
}

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


// -------------- sensor ---------------
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

bool obstacleDetected(){
    return lastDist > 0 && lastDist < DETECT_CM;
}

void setupSensor(){
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    digitalWrite(TRIG_PIN, LOW);
}

// ---------------- state machine ----------------
const char* stateName(State s){
    switch (s){
        case DRIVING: return "DRIVING";
        case PAUSE_BEFORE_BACKUP: return "PAUSE_BEFORE_BACKUP";
        case BACKUP: return "BACKUP";
        case TURNING: return "TURNING";
        case PAUSE_AFTER_TURN: return "PAUSE_AFTER_TURN";
        case STOPPED: return "STOPPED";
    }

    return "UNKNOWN";
}

void enterState(State next){
    state = next;
    stateStart = millis();
    Serial.printf("state: %s\n", stateName(next));
}

void startRobot(){
    if (state != STOPPED) return;
    startSweep(broom);
    startSweep(sponge);
    forward();
    enterState(DRIVING);
}

// Latches: nothing moves again until startRobot().
void stopRobot(){
    stopBot();
    stopSweep(broom);
    stopSweep(sponge);
    if (state != STOPPED) enterState(STOPPED);
}

void updateRobot(){
    unsigned long elapsed = millis() - stateStart;
    switch (state){
        case DRIVING:
            if (obstacleDetected()){
                Serial.printf("Obstacle at %.1f cm\n", lastDist);
                stopBot();
                enterState(PAUSE_BEFORE_BACKUP);
            }
            break;
        case PAUSE_BEFORE_BACKUP:
            if (elapsed >= PAUSE_TIME){
                backward();
                enterState(BACKUP);
            }
            break;
        case BACKUP:
            if (elapsed >= BACKUP_TIME){
                turnTime = random(300, 900);
                if (random(2)) spinLeft(); else spinRight();
                enterState(TURNING);
            }
            break;
        case TURNING:
            if (elapsed >= turnTime){
                stopBot();
                enterState(PAUSE_AFTER_TURN);
            }
            break;
        case PAUSE_AFTER_TURN:
            if (elapsed >= PAUSE_TIME){
                forward();
                enterState(DRIVING);
            }
            break;
        case STOPPED:
            break;
    }
}

// ---------------- emergency stop ----------------
// A press stops at once. From STOPPED, a 2 s hold starts the robot. The button
// must be seen released between actions, so a stuck button can't restart it.
bool btnWasUp = false;
unsigned long btnDownAt = 0;
unsigned long btnUpAt = 0;

void updateEStop(){
    if (digitalRead(ESTOP_PIN) == HIGH){
        if (btnUpAt == 0) btnUpAt = millis();
        if (millis() - btnUpAt >= RELEASE_TIME) btnWasUp = true;
        btnDownAt = 0;
        return;
    }
    btnUpAt = 0;
    if (!btnWasUp) return;

    if (state != STOPPED){
        Serial.println("E-stop: button");
        stopRobot();
        btnWasUp = false;
        return;
    }
    if (btnDownAt == 0) btnDownAt = millis();
    if (millis() - btnDownAt >= RESUME_HOLD){
        btnWasUp = false;
        startRobot();
    }
}

void setupEStop(){
    pinMode(ESTOP_PIN, INPUT_PULLUP);
}

// ---------------- WiFi and MQTT ----------------
// See docs/mqtt_topics.md. Connecting can block for seconds, so this runs in its
// own task on core 0 and loop() keeps driving and watching the e-stop on core 1.
enum Command { CMD_NONE, CMD_START, CMD_STOP };
std::atomic<Command> pendingCmd{CMD_NONE};   // set by the network task, used in loop()

void handleCommand(){
    Command cmd = pendingCmd.exchange(CMD_NONE);
    if (cmd == CMD_START) startRobot();
    if (cmd == CMD_STOP){
        Serial.println("E-stop: MQTT");
        stopRobot();
    }
}

#ifdef HAVE_SECRETS
const char* TOPIC_CMD       = "duckmopper/node1/cmd";
const char* TOPIC_STATUS    = "duckmopper/node1/status";
const char* TOPIC_TELEMETRY = "duckmopper/node1/telemetry";
const unsigned long TELEMETRY_INT = 500;
const unsigned long RECONNECT_INT = 5000;

WiFiClient netClient;
PubSubClient mqtt(netClient);

void onMqttMessage(char* topic, byte* payload, unsigned int len){
    String msg;
    for (unsigned int i = 0; i < len; i++) msg += (char)payload[i];
    if (msg == "start") pendingCmd = CMD_START;
    else if (msg == "stop" || msg == "estop") pendingCmd = CMD_STOP;
}

void networkTask(void*){
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    mqtt.setServer(MQTT_HOST, MQTT_PORT);
    mqtt.setCallback(onMqttMessage);

    unsigned long lastTry = 0, lastSend = 0;
    bool tried = false;
    for (;;){
        if (WiFi.status() == WL_CONNECTED && !mqtt.connected()
                && (!tried || millis() - lastTry >= RECONNECT_INT)){
            tried = true;
            lastTry = millis();
            if (mqtt.connect("duckmopper-node1", TOPIC_STATUS, 1, true, "offline")){
                mqtt.publish(TOPIC_STATUS, "online", true);
                mqtt.subscribe(TOPIC_CMD, 1);
                Serial.println("MQTT connected");
            }
        }
        if (mqtt.connected()){
            mqtt.loop();
            if (millis() - lastSend >= TELEMETRY_INT){
                lastSend = millis();
                char buf[80];
                snprintf(buf, sizeof(buf), "{\"dist_cm\":%.1f,\"state\":\"%s\"}",
                         lastDist, stateName(state));
                mqtt.publish(TOPIC_TELEMETRY, buf);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
#endif

void startNetwork(){
#ifdef HAVE_SECRETS
    xTaskCreatePinnedToCore(networkTask, "network", 8192, nullptr, 1, nullptr, 0);
#else
    Serial.println("No secrets.h: MQTT off, use the button.");
#endif
}

// ----- setup function for all hardware -----
void setup(){
    Serial.begin(115200);
    randomSeed(esp_random());
    setupSensor();
    setupEStop();
    setupServos();
    setupMotors();

#ifdef HARDWARE_TEST
    Serial.println("Setup done. Wheels will spin in 3 s, lift the robot off the ground.");
    delay(3000);
#else
    startNetwork();
    Serial.println("Setup done. Hold the button 2 s or send MQTT start.");
#endif
}


#ifdef HARDWARE_TEST
// --------- hardware test: runs each part in turn and prints what should be happening
void testSensor(unsigned long ms){
    Serial.println("-- sensor (-1 = no echo)");
    unsigned long start = millis();
    while (millis() - start < ms){
        updateSensor();
        Serial.printf("%.1f cm%s\n", lastDist, obstacleDetected() ? "  OBSTACLE" : "");
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

// Runs broom + sponges together for ms, without blocking: the loop keeps
// spinning, so the sensor keeps updating at the same time.
void testScrubbers(unsigned long ms){
    Serial.println("-- scrubbers: broom + sponges sweeping, sensor still live");
    startSweep(broom);
    startSweep(sponge);
    unsigned long start = millis();
    unsigned long lastPrint = 0;
    while (millis() - start < ms){
        updateScrubbers();
        updateSensor();
        if (millis() - lastPrint >= 500){
            lastPrint = millis();
            Serial.printf("   sensor: %.1f cm\n", lastDist);
        }
    }
    stopSweep(broom);
    stopSweep(sponge);
}

// -------- main loop (hardware test build) ----------
void loop(){
    testSensor(3000);

    testMove("forward", forward);
    testMove("backward", backward);
    testMove("spin left", spinLeft);
    testMove("spin right", spinRight);

    testServo("broom", broom_serv);
    testServo("sponge 1", sponge1_serv);
    testServo("sponge 2", sponge2_serv);

    testScrubbers(4000);

    Serial.println("== cycle done ==");
    delay(2000);
}

#else
// -------- main loop ----------
void loop(){
    updateEStop();
    handleCommand();
    updateSensor();
    updateRobot();
    updateScrubbers();
}
#endif
