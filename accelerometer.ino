#include "LSM6DS3.h"
#include "Wire.h"

//Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A

const int FLOAT_PRECISION = 1;
const int BUFFERS_SIZE = 50; // change or remove in the future

const int ACCELERATION_SENSITIVITY = 3;

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
  
  //Call .begin() to configure the IMUs
  if (myIMU.begin() != 0)
  {
    Serial.println("Device error");
  }
  else
  {
    Serial.println("Device OK!");
  }
}

void loop()
{
  Vector acceleration = {myIMU.readFloatAccelX(), myIMU.readFloatAccelY(), myIMU.readFloatAccelZ()};
  Vector gyroscope = {myIMU.readFloatGyroX(), myIMU.readFloatGyroY(), myIMU.readFloatGyroZ()};

  float aNorm = norm(acceleration);
  if (aNorm > ACCELERATION_SENSITIVITY && idx < BUFFERS_SIZE)
  {
    registerValues(idx, acceleration, gyroscope);
    // TODO send data to server
  }
  
  else if (idx >= BUFFERS_SIZE)
    idx = -1;

  logValues(acceleration, gyroscope);

  idx++;
  delay(1000);
}

float norm(Vector v)
{
  return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
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
