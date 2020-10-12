#include <SoftwareSerial.h>

#define DEBUGGING_SERIAL_RX         5
#define DEBUGGING_SERIAL_TX         6
#define DEBUGGING_SERIAL_BAUDRATE   115200

SoftwareSerial DebuggingSerial( DEBUGGING_SERIAL_RX, DEBUGGING_SERIAL_TX ); // RX, TX

// ------------------ for motor
#include "MOS_S2Motor.h"

// ------------------ for mobilebase
#define MOTOR_ID_FL                 0x01 // 전방 왼쪽 모터 아이디
#define MOTOR_ID_FR                 0x02 // 전방 오른쪽 모터 아이디
#define MOTOR_ID_BL                 0x03 // 후방 왼쪽 모터 아이디
#define MOTOR_ID_BR                 0x04 // 후방 오른쪽 모터 아이디

#define MOTOR_FL_SPEED_RATIO        1.0 // 전방 왼쪽 모터 속도 보정용
#define MOTOR_FR_SPEED_RATIO        1.0 // 전방 오른쪽 모터 속도 보정용
#define MOTOR_BL_SPEED_RATIO        1.0 // 후방 왼쪽 모터 속도 보정용
#define MOTOR_BR_SPEED_RATIO        1.0 // 후방 오른쪽 모터 속도 보정용

#define MOBILEBASE_MOTOR_RELATIVE_MODE        0 // 모바일베이스 모터를 항상 절대 위치 모드로 사용
#define MOBILEBASE_MOTOR_FREE_WHEEL_MODE      0 // 모바일베이스 모터 free wheel모드 사용하지 않음

#define MOTOR_VELOCITY_ABS_MIN_LIMIT       0 // 속도 최소값 (부호 없이)
#define MOTOR_VELOCITY_ABS_MAX_LIMIT       255 // 속도 최대값 (부호 없이)

#define MOBILEBASE_SPEED            100

/*
 * 한 개의 모터를 속도로 제어 준비
 * 제어 준비만 하는 함수이기 때문에 MOS_S2MotorSetSync();를 호출해야 동작 함
 * motorID : 바퀴로 사용하려는 모터 아이디
 * wheelSpeed : 회전 속도. +는 전진 방향, -는 후진 방향.
 */
void runOneMotorWithSpeed( uint8_t motorID, int16_t wheelSpeed );

/*
 * 모바일베이스 네 개의 모터를 각각 다른 속도로 제어
 * speedFL : 전방 왼쪽 바퀴 속도
 * speedFR : 전방 오른쪽 바퀴 속도
 * speedBL : 후방 왼쪽 바퀴 속도
 * speedBR : 후방 오른쪽 바퀴 속도
 */
void moveMobileBaseWithFourVelocity( int16_t speedFL, int16_t speedFR, int16_t speedBL, int16_t speedBR );

// ------------------ for robot arm

#define ARM_SERVO_ID_1      0x05 // 매니퓰레이터 base모터
#define ARM_SERVO_ID_2      0x06
#define ARM_SERVO_ID_3      0x07
#define ARM_SERVO_ID_4      0x08 // 매니퓰레이터 엔드이펙터에 가장 가까운 모터

#define ARM_SERVO_1_VALUE_MIN_LIMIT    0      // 각 모터의 최소, 최대 값
#define ARM_SERVO_1_VALUE_MAX_LIMIT    1023
#define ARM_SERVO_2_VALUE_MIN_LIMIT    75
#define ARM_SERVO_2_VALUE_MAX_LIMIT    765
#define ARM_SERVO_3_VALUE_MIN_LIMIT    271
#define ARM_SERVO_3_VALUE_MAX_LIMIT    958
#define ARM_SERVO_4_VALUE_MIN_LIMIT    188
#define ARM_SERVO_4_VALUE_MAX_LIMIT    679

#define TORQUE_DEFAULT                      2 // 매니퓰레이터 모터 토크 기본 값, 0이 최소, 2가 최대
#define ARM_SERVO_RELATIVE_MODE             0 // 매니퓰레이터 모터를 항상 절대 위치 모드로 사용

/*
 * 한 개의 모터를 각도로 제어 준비
 * 제어 준비만 하는 함수이기 때문에 MOS_S2MotorSetSync();를 호출해야 동작 함
 * motorID : 매니퓰레이터로 사용하려는 모터 아이디
 * angle : 목표 각도. 0이 중심
 *         1번 모터의 -는 CCW, +는 CW
 *         2, 3, 4번 모터의 -는 매니퓰레이터가 내려가는 방향 +는 매니퓰레이터가 올라가는 방향
 */
void moveRobotArmServoWithAngle( uint8_t motorID, float angle );

/*
 * 매니퓰레이터 네 개의 모터를 각각 다른 위치로 제어
 * a1 : 1번 모터 각도
 * a2 : 2번 모터 각도
 * a3 : 3번 모터 각도
 * a4 : 4번 모터 각도
 */
void robotArmForwardMove( float a1, float a2, float a3, float a4 );

// ------------------ for gripper
#include <Servo.h>

#define SERVO_GRIPPER_PIN         11  // 서보 핀

#define GRIP_ANGLE_CLOSE          100 // 그리퍼 닫히는 각도
#define GRIP_ANGLE_OPEN           20  // 그리퍼 열리는 각도

Servo gripper;

// ------------------ for IMU
#include "IMU.h"

// ------------------

void setup() {
  DebuggingSerial.begin( DEBUGGING_SERIAL_BAUDRATE ); // 디버깅 시리얼 초기화
  Serial.begin( MOS_S2_MOTOR_BAUDRATE ); // 모터 제어 버스 초기화

  runOneMotorWithSpeed( MOTOR_ID_FL, 0 ); // 모바일베이스 정지
  runOneMotorWithSpeed( MOTOR_ID_FR, 0 );
  runOneMotorWithSpeed( MOTOR_ID_BL, 0 );
  runOneMotorWithSpeed( MOTOR_ID_BR, 0 );

  MOS_S2MotorSetTorque( MOTOR_ID_FL, 0 ); // 모바일베이스 모터 토크 off
  MOS_S2MotorSetTorque( MOTOR_ID_FR, 0 );
  MOS_S2MotorSetTorque( MOTOR_ID_BL, 0 );
  MOS_S2MotorSetTorque( MOTOR_ID_BR, 0 );

  MOS_S2MotorSetTorque( ARM_SERVO_ID_1, 0 ); // 매니퓰레이터 모터 토크 off
  MOS_S2MotorSetTorque( ARM_SERVO_ID_2, 0 );
  MOS_S2MotorSetTorque( ARM_SERVO_ID_3, 0 );
  MOS_S2MotorSetTorque( ARM_SERVO_ID_4, 0 );
  
  gripper.attach(SERVO_GRIPPER_PIN); // 그리퍼 사용 준비

  // 그리퍼 열기
  gripper.write(GRIP_ANGLE_OPEN);

  Wire.begin();
  IMUInit(); // IMU초기화
  
  MOS_S2MotorSetTorque( MOTOR_ID_FL, 1 ); // 모바일베이스 모터 토크 on
  MOS_S2MotorSetTorque( MOTOR_ID_FR, 1 );
  MOS_S2MotorSetTorque( MOTOR_ID_BL, 1 );
  MOS_S2MotorSetTorque( MOTOR_ID_BR, 1 );

  MOS_S2MotorSetTorque( ARM_SERVO_ID_1, 1 ); // 매니퓰레이터 모터 토크 on
  MOS_S2MotorSetTorque( ARM_SERVO_ID_2, 1 );
  MOS_S2MotorSetTorque( ARM_SERVO_ID_3, 1 );
  MOS_S2MotorSetTorque( ARM_SERVO_ID_4, 1 );

  // 초기 팔 자세
  robotArmForwardMove( 0, 50, -142, 0 );

  delay(3000);
}

void loop() {
  static uint8_t step = 0;
  static uint8_t isWaiting = 0;
  static uint16_t prevTime = millis();
  static uint32_t delayTime = 0;
  static float goalAngle = 0;

  updateIMUValue(); // 최대한 빠르게 업데이트

  if( isWaiting )
  {
    if( millis() - prevTime >= delayTime )
    {
      isWaiting = 0;
      step++;
      if( step == 3 ) // 스텝 0, 1, 2를 반복
        step = 0;
    }
  }
  else
  {
    switch( step )
    {
      case 0 : // 2초 전진
      {
        runOneMotorWithSpeed( MOTOR_ID_FL, MOBILEBASE_SPEED );
        runOneMotorWithSpeed( MOTOR_ID_FR, MOBILEBASE_SPEED );
        runOneMotorWithSpeed( MOTOR_ID_BL, MOBILEBASE_SPEED );
        runOneMotorWithSpeed( MOTOR_ID_BR, MOBILEBASE_SPEED );
        
        MOS_S2MotorSetSync();
        
        isWaiting = 1;
        prevTime = millis();
        delayTime = 2000;
        
        break;
      }
        
      case 1 : // 1초 정지
      {
        runOneMotorWithSpeed( MOTOR_ID_FL, 0 );
        runOneMotorWithSpeed( MOTOR_ID_FR, 0 );
        runOneMotorWithSpeed( MOTOR_ID_BL, 0 );
        runOneMotorWithSpeed( MOTOR_ID_BR, 0 );
        
        MOS_S2MotorSetSync();

        isWaiting = 1;
        prevTime = millis();
        delayTime = 1000;
        goalAngle -= 90; // 좌로 직각이 되도록 회전하기 위한 세팅. 각도는 테스트하여 적당한 값을 찾아야 함
        
        break;
      }
        
      case 2 : // 90도 좌회전
      {
        float angleError = goalAngle - yawAngle; // 기준보다 더 좌회전 했다면 양수, 기준보다 더 우회전 했다면 음수
        if( abs( angleError ) > 0.5 )
        {
          runOneMotorWithSpeed( MOTOR_ID_FL, constrain( 40 + abs( angleError )*3, 0, 90 )*( angleError > 0 ? 1 : -1 ) ); // 좌회전을 하려면 FL, FR, BL, BR순서로
          runOneMotorWithSpeed( MOTOR_ID_FR, constrain( 40 + abs( angleError )*3, 0, 90 )*( angleError > 0 ? -1 : 1 ) ); // - + - + 값을 가지며,
          runOneMotorWithSpeed( MOTOR_ID_BL, constrain( 40 + abs( angleError )*3, 0, 90 )*( angleError > 0 ? 1 : -1 ) ); // 우회전을 하려면 FL, FR, BL, BR순서로
          runOneMotorWithSpeed( MOTOR_ID_BR, constrain( 40 + abs( angleError )*3, 0, 90 )*( angleError > 0 ? -1 : 1 ) ); // + - + - 값을 가짐

          MOS_S2MotorSetSync();
        }
        else
        {
          // 1초 정지
          runOneMotorWithSpeed( MOTOR_ID_FL, 0 );
          runOneMotorWithSpeed( MOTOR_ID_FR, 0 );
          runOneMotorWithSpeed( MOTOR_ID_BL, 0 );
          runOneMotorWithSpeed( MOTOR_ID_BR, 0 );
          
          MOS_S2MotorSetSync();

          isWaiting = 1;
          prevTime = millis();
          delayTime = 1000;
        }
        
        break;
      }
    }
  }
}

void runOneMotorWithSpeed( uint8_t motorID, int16_t wheelSpeed )
{
  uint8_t dir = 0;
  switch( motorID )
  {
    case MOTOR_ID_FL:
    case MOTOR_ID_BL:
      if( motorID == MOTOR_ID_FL )
        wheelSpeed *= MOTOR_FL_SPEED_RATIO;
      else // motorID == MOTOR_ID_BL
        wheelSpeed *= MOTOR_BL_SPEED_RATIO;
        
      if ( wheelSpeed < 0 ) // 속도가 음수
      {
        dir = MOS_S2_MOTOR_CW; // 후진방향
      }
      else // ( wheelSpeed > 0 ) or WheelSpeed == 0
      {
        dir = MOS_S2_MOTOR_CCW; // 전진방향
      }
      break;
    case MOTOR_ID_FR:
    case MOTOR_ID_BR:
      if( motorID == MOTOR_ID_FR )
        wheelSpeed *= MOTOR_FR_SPEED_RATIO;
      else // motorID == MOTOR_ID_BR
        wheelSpeed *= MOTOR_BR_SPEED_RATIO;
        
      if ( wheelSpeed < 0 ) // 속도가 음수
      {
        dir = MOS_S2_MOTOR_CCW; // 후진방향
      }
      else // ( wheelSpeed > 0 ) or WheelSpeed == 0
      {
        dir = MOS_S2_MOTOR_CW; // 전진방향
      }
      break;
    default:
      return;
  }
  MOS_S2MotorSetNextWheel( motorID, 0, 1, 1, MOBILEBASE_MOTOR_RELATIVE_MODE, MOBILEBASE_MOTOR_FREE_WHEEL_MODE, dir, constrain( abs( wheelSpeed ), MOTOR_VELOCITY_ABS_MIN_LIMIT, MOTOR_VELOCITY_ABS_MAX_LIMIT ) );
}

void moveMobileBaseWithFourVelocity( int16_t speedFL, int16_t speedFR, int16_t speedBL, int16_t speedBR )
{
  runOneMotorWithSpeed( MOTOR_ID_FL, speedFL );
  runOneMotorWithSpeed( MOTOR_ID_FR, speedFR );
  runOneMotorWithSpeed( MOTOR_ID_BL, speedBL );
  runOneMotorWithSpeed( MOTOR_ID_BR, speedBR );
  
  MOS_S2MotorSetSync();
}

void moveRobotArmServoWithAngle( uint8_t motorID, float angle )
{
  if( !isnan( angle ) )
  {
    // 인자로 입력받은 각도를 -170~170범위의 각도에서 모터의 포지션 범위 0~1024로 매핑
    uint16_t motorValueCalculated;
    if( motorID == ARM_SERVO_ID_3 )
    {
      motorValueCalculated = map( constrain( angle, -170, 170 ), 170, -170, 0, 1024 ); // 매니퓰레이터 구조상 3번 모터만 반대
    }
    else
    {
      motorValueCalculated = map( constrain( angle, -170, 170 ), -170, 170, 0, 1024 );
    }
    
    switch( motorID )
    {
      case ARM_SERVO_ID_1:
        if( motorValueCalculated < ARM_SERVO_1_VALUE_MIN_LIMIT || motorValueCalculated > ARM_SERVO_1_VALUE_MAX_LIMIT ) {
          motorValueCalculated = constrain( motorValueCalculated, ARM_SERVO_1_VALUE_MIN_LIMIT, ARM_SERVO_1_VALUE_MAX_LIMIT );
        }
        break;
      case ARM_SERVO_ID_2:
        if( motorValueCalculated < ARM_SERVO_2_VALUE_MIN_LIMIT || motorValueCalculated > ARM_SERVO_2_VALUE_MAX_LIMIT ) {
          motorValueCalculated = constrain( motorValueCalculated, ARM_SERVO_2_VALUE_MIN_LIMIT, ARM_SERVO_2_VALUE_MAX_LIMIT );
        }
        break;
      case ARM_SERVO_ID_3:
        if( motorValueCalculated < ARM_SERVO_3_VALUE_MIN_LIMIT || motorValueCalculated > ARM_SERVO_3_VALUE_MAX_LIMIT ) {
          motorValueCalculated = constrain( motorValueCalculated, ARM_SERVO_3_VALUE_MIN_LIMIT, ARM_SERVO_3_VALUE_MAX_LIMIT );
        }
        break;
      case ARM_SERVO_ID_4:
        if( motorValueCalculated < ARM_SERVO_4_VALUE_MIN_LIMIT || motorValueCalculated > ARM_SERVO_4_VALUE_MAX_LIMIT ) {
          motorValueCalculated = constrain( motorValueCalculated, ARM_SERVO_4_VALUE_MIN_LIMIT, ARM_SERVO_4_VALUE_MAX_LIMIT );
        }
        break;
    }

    MOS_S2MotorSetNextPosition( motorID, 0, 1, 1, TORQUE_DEFAULT, ARM_SERVO_RELATIVE_MODE, motorValueCalculated );
  }
}

void robotArmForwardMove( float a1, float a2, float a3, float a4 )
{
  moveRobotArmServoWithAngle(ARM_SERVO_ID_1, a1 );
  moveRobotArmServoWithAngle(ARM_SERVO_ID_2, a2 );
  moveRobotArmServoWithAngle(ARM_SERVO_ID_3, a3 );
  moveRobotArmServoWithAngle(ARM_SERVO_ID_4, a4 );

  MOS_S2MotorSetSync();
}
