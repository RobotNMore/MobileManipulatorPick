#ifndef IMU_H
#define IMU_H

#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 mpu;

#define IMU_DEGREE_PER_SECOND       131.068 // 32767 / 250
// 자이로스코프의 adc는 16비트. 최대 각속도는 +-250도. 그래서 32767(int16_t최대값)/250(최대각속도)하면
// 각속도가 1일 때 자이로스코프 adc값. degree(자이로스코프에서 온 adc값) per second가 나옴
// IMU로부터 받은 gyro값에 위 값을 나눠주면 실제 degree(도 단위) per second가 나옴
#define IMU_GYRO_Z_OFFSET           -40 // 로봇을 가만히 두었을 때 나오는 gyro z값들을 0으로 만들기 위한 offset
#define IMU_READING_COUNT           5
//#define IMU_DEBUG

float yawAngle;      // 현재 각도값을 저장
float prevYawAngle;  // 직전 각도값을 저장

// IMU센서 초기화
void IMUInit();

// IMU gyro z각도 값 업데이트
void updateIMUValue();

// delta time 계산하는 함수. ( 현재 시간 - 이전 호출 시간 )/1000이 반환됨
float calcDT();

void IMUInit()
{
  // initialize device
#ifdef IMU_DEBUG
  DebuggingSerial.println(F("Initializing I2C devices..."));
#endif
  mpu.initialize();

// IMU_GYRO_Z_OFFSET 사용하지 않고 IMU 가속도, 자이로 값 보정할 때 사용
//  mpu.setXAccelOffset(-676); 
//  mpu.setYAccelOffset(-1685);
//  mpu.setZAccelOffset(924);
//  mpu.setXGyroOffset(74);
//  mpu.setYGyroOffset(-16);
//  mpu.setZGyroOffset(-4);

  updateIMUValue();
  prevYawAngle = yawAngle = 0; // Init을 할 시점의 각도를 0도로 설정
}

void updateIMUValue()
{
  float sum = 0;
  float yawAngleTemp = 0;
  int16_t ax, ay, az, gx, gy, gz;
  
  mpu.getMotion6( &ax, &ay, &az, &gx, &gy, &gz );

  for( int i = 0 ; i < IMU_READING_COUNT ; i++ ) // reading count만큼 읽어서 평균 내기
  {
    // ( gz - IMU_GYRO_Z_OFFSET ) / IMU_DEGREE_PER_SECOND )해서 나온 각속도값에 deltaTime을 곱하고 yawAngleTemp에 누적
    yawAngleTemp += ( ( gz - IMU_GYRO_Z_OFFSET ) / IMU_DEGREE_PER_SECOND ) * calcDT();
    sum += yawAngleTemp; // 나중에 평균내기 위해 sum변수에 합
  }
  yawAngle -= sum/IMU_READING_COUNT; // 여러 번 읽어 평균 낸 값을 현재 각도에 반영

  if( abs( yawAngle - prevYawAngle ) >= 10 ) // 급격하게 변화한 값 무시
    yawAngle = prevYawAngle;
  else
    prevYawAngle = yawAngle;
}

float calcDT()
{
  static uint32_t t_prev = 0;
  uint32_t t_now;

  float dt;
  
  t_now = millis();
  dt = (t_now - t_prev)/1000.0;
  t_prev = t_now;
  
  return dt;
}

#endif
