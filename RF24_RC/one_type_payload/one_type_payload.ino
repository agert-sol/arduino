#include <SPI.h>
#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>
#include <Servo.h>

RF24           radio(9, 10);
Servo          myservo;

struct positions {
    int speed;
    int rudder;
};

const uint64_t pipe = 0xE8E8F0F0E1LL;

const int roleSetPin =  6;
const int roleGetPin =  7;

const int rudderPin = A0;
const int speedPin = A1;

const int AIn1Pin =  2;
const int AIn2Pin =  4;
const int PWMAPin =  3;
const int ServoPin = 5;

const int STBYPin =  8;

const int speedStep = 10;
const int speedZero = 250;
const int speedStartFrom = 20;

const int rudderZero = 90; //angle
const int rudderMax = 28; // angle
const int servoZero = 98; // angle

int roleSlave;

//For master
int lastSpeed;
int lastRudder;

//For slave
unsigned long payloadReceivedAt;
struct positions lastPayload;

void detectRole()
{
    pinMode(roleSetPin, OUTPUT);
    pinMode(roleGetPin, INPUT);

    digitalWrite(roleSetPin, HIGH);
    delay(15);
    roleSlave = digitalRead(roleGetPin);
}

void attachPins()
{
    //Slave pins
    pinMode(AIn1Pin, OUTPUT);
    pinMode(AIn2Pin, OUTPUT);
    pinMode(PWMAPin, OUTPUT);
    pinMode(STBYPin, OUTPUT);
    myservo.attach(ServoPin);
}

void initRadio()
{
    delay(2000);
    radio.begin();
    delay(3000);
    radio.setChannel(0x0c);
    radio.setDataRate(RF24_1MBPS); // RF24_250KBPS, RF24_1MBPS, RF24_2MBPS
    radio.setPALevel(RF24_PA_HIGH); // RF24_PA_MIN=-18dBm, RF24_PA_LOW=-12dBm, RF24_PA_HIGH=-6dBm, RF24_PA_MAX=0dBm

    if (roleSlave) {
        radio.openReadingPipe(1, pipe);
        radio.startListening();
        Serial.println("working as SLAVE");
    } else {
        radio.openWritingPipe(pipe);
        radio.stopListening();
        Serial.println("working as MASTER");
    }

    radio.printDetails();
}

void setup()
{
    Serial.begin(115200);
    printf_begin();

    detectRole();

    attachPins();

    initRadio();

    Serial.println("setup finished");
}

struct positions collectPinsData()
{
    int speedValue = analogRead(speedPin);
    int ruderValue = analogRead(rudderPin);

    return {speedValue, ruderValue};
}

int debounceValue(int current, int last, int maxDelta)
{
    int delta = abs(current - last);

    if (delta > maxDelta) {
        return current;
    }

    return last;
}

int roundValue(int value, int step)
{
    return round(value / step) * step;
}

struct positions prepareDataToSend(struct positions pins)
{
    int speed = pins.speed;
    speed = map(speed, 0, 1023, 0, 500);
    speed = debounceValue(speed, lastSpeed, 7);
    lastSpeed = speed;
    //Normalize speed
    speed = roundValue(speed, speedStep);
    speed = speed - speedZero;
    if (abs(speed) < speedStartFrom) {
        speed = 0;
    }

    int rudder = pins.rudder;
    rudder = map(rudder, 0, 1023, 0, 180);
    rudder = debounceValue(rudder, lastRudder, 2);
    lastRudder = rudder;
    //Normalize rudder
    rudder = rudder - rudderZero;
    rudder = constrain(rudder, -rudderMax, rudderMax);

    return {speed, rudder};
}

void writeMotorData(int speed, int AIn1, int AIn2, int STBY)
{
    digitalWrite(STBYPin, STBY);
    digitalWrite(AIn1Pin, AIn1);
    digitalWrite(AIn2Pin, AIn2);
    analogWrite(PWMAPin, speed);
}

void setSpeed(int speed)
{
    if (speed == 0) {
        //Stop moving
        writeMotorData(0, LOW, LOW, LOW);
        return;
    }

    int AIn1 = HIGH;
    int AIn2 = LOW;
    if (speed < 0) {
        AIn1 = LOW;
        AIn2 = HIGH;
    }

    writeMotorData(abs(speed), AIn1, AIn2, HIGH);
}

void setRudder(int rudder)
{
    myservo.write(servoZero + rudder);
    delay(15);
}

void applyControl(struct positions controlData)
{
    setSpeed(controlData.speed);

    setRudder(controlData.rudder);
}

void stopMotors()
{
    struct positions stoped = {0, 0};

    applyControl(stoped);
}

String positionsToString(struct positions data)
{
    return "speed - " + String(data.speed) + ", rudder - " + String(data.rudder);
}

void send(struct positions data)
{
    struct positions payload;
    payload = data;

    bool ok = radio.write(&payload, sizeof(payload));

    String result = "fail";
    if (ok) {
        result = "ok";
    }

    Serial.println(result + " " + positionsToString(payload));
}

void asMaster()
{
    struct positions pinsPositions;
    pinsPositions = collectPinsData();

    struct positions dataToSend;
    dataToSend = prepareDataToSend(pinsPositions);

    applyControl(dataToSend);

    send(dataToSend);
}

void asSlave()
{
    if (radio.available()) {
        struct positions payload;
        radio.read(&payload, sizeof(payload));

        Serial.println("radio active " + positionsToString(payload));

        lastPayload = payload;

        payloadReceivedAt = millis();
    }

    if (millis() - payloadReceivedAt > 500) {
        stopMotors();

        return;
    }

    applyControl(lastPayload);
}

void loop()
{
    if (roleSlave) {
        asSlave();
    } else {
        asMaster();
    }
}
