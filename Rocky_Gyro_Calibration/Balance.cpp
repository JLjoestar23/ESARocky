#include <Wire.h>
#include "Balance.h"

int32_t gYZero;
int32_t angle; // millidegrees
int32_t angleRate; // degrees/s
int32_t distanceLeft;
int32_t speedLeft;
int32_t driveLeft;
int32_t distanceRight;
int32_t displacement;

int32_t speedRight;
int32_t driveRight;
int16_t motorSpeed;
int32_t  angle_accum;
int16_t angle_prev;
bool isBalancingStatus;
bool balanceUpdateDelayedStatus;
extern float speedCont;          // this is the motor speed 

#define KP  11
#define KI 1
#define KD 1


bool isBalancing()
{
  return isBalancingStatus;
}

bool balanceUpdateDelayed()
{
  return balanceUpdateDelayedStatus;
}

void balanceSetup()
{
  // Initialize IMU.
  Wire.begin();
  if (!imu.init())
  {
    while(true)
    {
      Serial.println("Failed to detect and initialize IMU!");
      delay(200);
    }
  }
  imu.enableDefault();
  imu.writeReg(LSM6::CTRL2_G, 0b01011000); // 208 Hz, 1000 deg/s

  // Wait for IMU readings to stabilize.
  delay(1000);

  // Calibrate the gyro.
  int32_t total = 0;
  for (int i = 0; i < CALIBRATION_ITERATIONS; i++)
  {
    imu.read();
    total += imu.g.y;
    delay(1);
  }

  gYZero = total / CALIBRATION_ITERATIONS;
}

// This function contains the core algorithm for balancing a
// Balboa 32U4 robot.
void balance()
{
  // Adjust toward angle=0 with timescale ~10s, to compensate for
  // gyro drift.  More advanced AHRS systems use the
  // accelerometer as a reference for finding the zero angle, but
  // this is a simpler technique: for a balancing robot, as long
  // as it is balancing, we know that the angle must be zero on
  // average, or we would fall over.
  angle = angle * 999 / 1000;

  // This variable measures how close we are to our basic
  // balancing goal - being on a trajectory that would cause us
  // to rise up to the vertical position with zero speed left at
  // the top.  This is similar to the fallingAngleOffset used
  // for LED feedback and a calibration procedure discussed at
  // the end of Balancer.ino.
  //
  // It is in units of millidegrees, like the angle variable, and
  // you can think of it as an angular estimate of how far off we
  // are from being balanced.
  int32_t risingAngleOffset = angleRate * ANGLE_RATE_RATIO + angle;

  // Combine risingAngleOffset with the distance and speed
  // variables, using the calibration constants defined in
  // Balance.h, to get our motor response.  Rather than becoming
  // the new motor speed setting, the response is an amount that
  // is added to the motor speeds, since a *change* in speed is
  // what causes the robot to tilt one way or the other.

  angle_accum = angle_accum + angle;
  
  motorSpeed = (
     -KP*angle - KI*angle_accum - KP*(angle-angle_prev)
    ) / 100 / GEAR_RATIO;

  angle_prev = angle; 
  
 // motorSpeed += (
 //  + ANGLE_RESPONSE * risingAngleOffset
 //   + DISTANCE_RESPONSE * (distanceLeft + distanceRight)
 //   + SPEED_RESPONSE * (speedLeft + speedRight)
 //   ) / 100 / GEAR_RATIO;

  if (motorSpeed > MOTOR_SPEED_LIMIT)
  {
    motorSpeed = MOTOR_SPEED_LIMIT;
  }
  if (motorSpeed < -MOTOR_SPEED_LIMIT)
  {
    motorSpeed = -MOTOR_SPEED_LIMIT;
  }

  // Adjust for differences in the left and right distances; this
  // will prevent the robot from rotating as it rocks back and
  // forth due to differences in the motors, and it allows the
  // robot to perform controlled turns.
  int16_t distanceDiff = distanceLeft - distanceRight;

  if(!DISABLE_MOTORS){
   motors.setSpeeds(
      motorSpeed + distanceDiff * DISTANCE_DIFF_RESPONSE / 100,
     motorSpeed - distanceDiff * DISTANCE_DIFF_RESPONSE / 100);
    }
}

void lyingDown()
{
  // Reset things so it doesn't go crazy.
  motorSpeed = 0;
  distanceLeft = 0;
  distanceRight = 0;
  motors.setSpeeds(0, 0);

  if (angleRate > -6 && angleRate < 6)
  {
    // It's really calm, so we know the angles.
    if (imu.a.z > 0)
    {
      
      // this is based on coarse measurement of what I think the angle would be resting on the flat surface. 
      // this corresponds to 94.8 degrees
      angle = 94827;
  
    }
    else
    {
      // this is based on coarse measurement of what I think the angle would be resting on the flat surface. 
         angle = -94827; 
    }
    distanceLeft = 0;
    distanceRight = 0;
  }
}

void integrateGyro()
{
  // Convert from full-scale 1000 deg/s to deg/s.
  angleRate = (imu.g.y - gYZero) / 29;

  angle += angleRate * UPDATE_TIME_MS;
}

void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq) {
  //DESCRIPTION: Attitude estimation through sensor fusion - 6DOF
  /*
   * See description of Madgwick() for more information. This is a 6DOF implimentation for when magnetometer data is not
   * available (for example when using the recommended MPU6050 IMU for the default setup).
   */
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  //Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  //Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    //Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    //Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    //Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); //normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    //Apply feedback step
    qDot1 -= B_madgwick * s0;
    qDot2 -= B_madgwick * s1;
    qDot3 -= B_madgwick * s2;
    qDot4 -= B_madgwick * s3;
  }

  //Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * invSampleFreq;
  q1 += qDot2 * invSampleFreq;
  q2 += qDot3 * invSampleFreq;
  q3 += qDot4 * invSampleFreq;

  //Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  //Compute angles
  roll_IMU = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*57.29577951; //degrees
  pitch_IMU = -asin(constrain(-2.0f * (q1*q3 - q0*q2),-0.999999,0.999999))*57.29577951; //degrees
  yaw_IMU = -atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)*57.29577951; //degrees
}



void integrateEncoders()
{
  static int16_t lastCountsLeft;
  int16_t countsLeft = encoders.getCountsLeft();
  speedLeft = (countsLeft - lastCountsLeft);
  distanceLeft += countsLeft - lastCountsLeft;
  lastCountsLeft = countsLeft;

  static int16_t lastCountsRight;
  int16_t countsRight = encoders.getCountsRight();
  speedRight = (countsRight - lastCountsRight);
  distanceRight += countsRight - lastCountsRight;
  lastCountsRight = countsRight;
  if(speedCont > 0)
    displacement = displacement + distanceLeft;
  else
    displacement = displacement - distanceLeft;
   
    
}

void balanceDrive(int16_t leftSpeed, int16_t rightSpeed)
{
  driveLeft = leftSpeed;
  driveRight = rightSpeed;
}

void balanceDoDriveTicks()
{
  distanceLeft -= driveLeft;
  distanceRight -= driveRight;
  speedLeft -= driveLeft;
  speedRight -= driveRight;
}

void balanceResetEncoders()
{
  distanceLeft = 0;
  distanceRight = 0;
}
uint32_t delta_ms = 0;
void balanceUpdateSensors()
{
  static uint32_t prev_ms = 0;
  uint32_t cur_ms = 0;

  cur_ms = millis();
  imu.read();
  integrateGyro();
  integrateEncoders();
  delta_ms = prev_ms - cur_ms;
  prev_ms = cur_ms;
}



