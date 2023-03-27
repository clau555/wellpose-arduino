#include "LSM6DS3.h"
#include "Wire.h"
#include "stdint.h"
#include <Arduino.h>
#include <lorae5.h>
#include "config.h"

// LoRaE5 initialisation
LoraE5 LoRaE5(devEUI, appEUI, appKey, devAddr, nwkSKey, appSKey);

#define ACCELERATION_THRESHOLD 2.
#define BUZZER_PIN 2
#define LED_PIN 3
// TODO: Change this
#define FRAME_DURATION (5 * 1000)
#define AVG_DURATION (1 * 1000)
#define BUFFER_SIZE 10

#define VERBOSE 1

LSM6DS3 gyro(I2C_MODE, 0x6A); // I2C device address 0x6A

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

// SETUP ##################################################

void setup()
{
    LoRa_Serial.begin(9600);
    USB_Serial.begin(9600);
    while (!USB_Serial)
    {
        USB_Serial.println("Waiting USB_Serial...");
    };

    setupGyro();
    setupLoRaWAN();
 
    USB_Serial.println("Setup complete\n");

    // hardware outputs
    // pinMode(LED_PIN, OUTPUT);
    // pinMode(BUZZER_PIN, OUTPUT);
}

void setupGyro()
{
    if (gyro.begin() != 0)
    {
        USB_Serial.println("Accelerometer/Gyroscope error");
    }
    else
    {
        USB_Serial.println("Accelerometer/Gyroscope OK!");
    }
}

void setupLoRaWAN()
{
    while(!LoRaE5.checkBoard());

    LoRaE5.setup(ACTIVATION_MODE, CLASS, SPREADING_FACTOR, ADAPTIVE_DR, CONFIRMED, PORT);
    LoRaE5.printInfo(SEND_BY_PUSH_BUTTON, FRAME_DELAY, LOW_POWER);

    if(ACTIVATION_MODE == OTAA)
    {
        LoRaE5.setDevEUI(devEUI);
        LoRaE5.setAppEUI(appEUI);
        LoRaE5.setAppKey(appKey);
        USB_Serial.println("\r\nJoin Procedure in progress...");
        while(LoRaE5.join() == false);
        delay(3000);
    }
    
    if(ACTIVATION_MODE == ABP)
    {
        LoRaE5.setDevAddr(devAddr);
        LoRaE5.setNwkSKey(nwkSKey);
        LoRaE5.setAppSKey(appSKey);
    }

    USB_Serial.println("LoRaWAN setup complete");
}

// LOOP ###################################################

void loop()
{
    Vector acceleration = { gyro.readFloatAccelX(), gyro.readFloatAccelY(), gyro.readFloatAccelZ() };
    // Vector acceleration = {10., 10., 10. };
    float accNorm = norm(acceleration);
    
    Vector gyroscope = { gyro.readFloatGyroX(), gyro.readFloatGyroY(), gyro.readFloatGyroZ() };
    // Vector gyroscope = {10., 10., 10. };

    if (avgMode)
    {
        handleAvgMode(acceleration);
    }
    else if (accNorm > ACCELERATION_THRESHOLD)
    {
        registerValues(acceleration);
    }

    if (millis() > lastFrameTime + FRAME_DURATION) 
    {
        sendData(gyroscope);
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

void sendData(Vector gyroscope) 
{

    // TODO send using LoRaWan
    
#if VERBOSE
    USB_Serial.println("\n------------");
    USB_Serial.println("Sending data");

    USB_Serial.println("\nGyroscope");

    USB_Serial.print(gyroscope.x);
    USB_Serial.print(" ");
    USB_Serial.print(gyroscope.y);
    USB_Serial.print(" ");
    USB_Serial.print(gyroscope.z);
    USB_Serial.println("");

    USB_Serial.println("\nAccelerations");

    for (int i = 0; i < accCount; ++i)
    {
        USB_Serial.print((int) accelerations[i].timestamp);
        USB_Serial.print(" ");
        USB_Serial.print(accelerations[i].v.x);
        USB_Serial.print(" ");
        USB_Serial.print(accelerations[i].v.y);
        USB_Serial.print(" ");
        USB_Serial.print(accelerations[i].v.z);
        USB_Serial.println("");
    }
#endif

    accCount = 0;
    lastFrameTime = millis();
}

void blinkLed()
{
    digitalWrite(LED_PIN, HIGH);
    delay(1000);
    digitalWrite(LED_PIN, LOW);
}

void buzz()
{
    digitalWrite(BUZZER_PIN, HIGH);
    delay(1000);
    digitalWrite(BUZZER_PIN, LOW);
}

Vector getAbsoluteGyro()
{
    const float sensi = 0.3;
    const float zero = 1.569;
    const float ADC_ref = 5.0;

    int16_t x = gyro.readRawGyroX();
    int16_t y = gyro.readRawGyroY();
    int16_t z = gyro.readRawGyroZ();

    float xv = (x / 1024.0 * ADC_ref - zero) / sensi;
    float yv = (y / 1024.0 * ADC_ref - zero) / sensi;
    float zv = (z / 1024.0 * ADC_ref - zero) / sensi;

    float ax = atan2(-yv, -zv) * 57.2957795 + 180;
    float ay = atan2(-xv, -zv) * 57.2957795 + 180;
    float az = atan2(-yv, -xv) * 57.2957795 + 180;

    return { ax, ay, az };
}
