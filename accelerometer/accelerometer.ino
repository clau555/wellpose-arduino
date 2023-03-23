#include "LSM6DS3.h"
#include "Wire.h"

#define ACCELERATION_SENSITIVITY 2.

#define FLOAT_PRECISION 1

#define LED_PIN 2

// TODO: change or remove in the future
#define BUFFERS_SIZE 50

LSM6DS3 gyro(I2C_MODE, 0x6A);

struct Vector
{
    float x;
    float y;
    float z;
};

Vector accelerations[BUFFERS_SIZE];
Vector gyroscopes[BUFFERS_SIZE];

int idx = 0;

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
    Vector gyroscope = { gyro.readFloatGyroX(), gyro.readFloatGyroY(), gyro.readFloatGyroZ() };

    float aNorm = norm(acceleration);
    if (aNorm > ACCELERATION_SENSITIVITY && idx < BUFFERS_SIZE)
    {
        registerValues(idx, acceleration, gyroscope);

        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(LED_PIN, LOW);

        // TODO: send data to server
    }
    else if (idx >= BUFFERS_SIZE)
    {
        idx = -1;
    }    

    logValues(acceleration, gyroscope);

    idx++;
    delay(10);
}

float norm(Vector v)
{
    return sqrt(sq(v.x) + sq(v.y) + sq(v.z));
}

void registerValues(int idx, Vector acceleration, Vector gyroscope)
{
    accelerations[idx] = acceleration;
    gyroscopes[idx] = gyroscope;
}

void logValues(Vector acceleration, Vector gyroscope)
{
    //Accelerometer
    Serial.print("\nAccelerometer:\n");
    Serial.print(" X1 = ");
    Serial.println(acceleration.x, FLOAT_PRECISION);
    Serial.print(" Y1 = ");
    Serial.println(acceleration.y, FLOAT_PRECISION);
    Serial.print(" Z1 = ");
    Serial.println(acceleration.z, FLOAT_PRECISION);

    //Gyroscope
    Serial.print("\nGyroscope:\n");
    Serial.print(" X1 = ");
    Serial.println(gyroscope.x, FLOAT_PRECISION);
    Serial.print(" Y1 = ");
    Serial.println(gyroscope.y, FLOAT_PRECISION);
    Serial.print(" Z1 = ");
    Serial.println(gyroscope.z, FLOAT_PRECISION);

    Serial.print("\n-----------------------\n");
}
