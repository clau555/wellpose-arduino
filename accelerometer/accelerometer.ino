#include "LSM6DS3.h"
#include "Wire.h"
#include "stdint.h"

#define ACCELERATION_SENSITIVITY 2.
#define BUZZER_PIN 2
#define LED_PIN 3
// TODO: Change this
#define FRAME_DURATION (5 * 1000)
#define AVG_DURATION (1 * 1000)
#define BUFFER_SIZE 60

LSM6DS3 gyro(I2C_MODE, 0x6A);

struct Vector
{
    float x;
    float y;
    float z;
};

struct AccelerationEntry
{
    Vector v;
    uint64_t timestamp;
};

AccelerationEntry accelerations[BUFFER_SIZE];

uint8_t accCount = 0;
uint64_t lastFrameTime = 0;
uint64_t lastAvgTime = 0;
bool avgMode = false;
uint16_t avgCount = 0;

void setup()
{
    Serial.begin(9600);
    while (!Serial);
    
    if (gyro.begin() != 0)
    {
        Serial.println("Device error");
    }
    else
    {
        Serial.println("Device OK!");
    }

    pinMode(LED_PIN, OUTPUT);
}

void loop()
{
    Vector acceleration = { gyro.readFloatAccelX(), gyro.readFloatAccelY(), gyro.readFloatAccelZ() };
    float accNorm = norm(acceleration);

    if (avgMode)
    {
        handleAvgMode(acceleration);
    }
    else if (accNorm > ACCELERATION_SENSITIVITY)
    {
        registerValues(acceleration);
    }

    if (millis() > lastFrameTime + FRAME_DURATION) 
    {
        sendData();
    }

    delay(10);
}

float norm(Vector v)
{
    return sqrt(sq(v.x) + sq(v.y) + sq(v.z));
}

void handleAvgMode(Vector acceleration)
{
    accelerations[accCount - 1].v.x += acceleration.x;
    accelerations[accCount - 1].v.y += acceleration.y;
    accelerations[accCount - 1].v.z += acceleration.z;
    ++avgCount;

    if (millis() > lastAvgTime + AVG_DURATION)
    {
        avgMode = false;
        accelerations[accCount - 1].v.x /= avgCount;
        accelerations[accCount - 1].v.y /= avgCount;
        accelerations[accCount - 1].v.z /= avgCount;
    }
}

void registerValues(Vector acceleration)
{
    if (accCount < BUFFER_SIZE) 
    {
        accelerations[accCount] = { acceleration, millis() - lastFrameTime };
        accCount++;
        avgMode = true;
        lastAvgTime = millis();
        avgCount = 0;
    }

    blinkLed();
}

void sendData() 
{
    Vector gyroscope = { gyro.readFloatGyroX(), gyro.readFloatGyroY(), gyro.readFloatGyroZ() };

    Serial.println("Sending data:");
    Serial.println(FRAME_DURATION);

    Serial.print(gyroscope.x);
    Serial.print(" ");
    Serial.print(gyroscope.y);
    Serial.print(" ");
    Serial.print(gyroscope.z);
    Serial.println("");

    Serial.println(accCount);

    for (int i = 0; i < accCount; ++i)
    {
        Serial.print((int) accelerations[i].timestamp);
        Serial.print(" ");
        Serial.print(accelerations[i].v.x);
        Serial.print(" ");
        Serial.print(accelerations[i].v.y);
        Serial.print(" ");
        Serial.print(accelerations[i].v.z);
        Serial.println("");
    }

    accCount = 0;
    lastFrameTime = millis();
}

void blinkLed()
{
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
}

void buzz()
{
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
}
