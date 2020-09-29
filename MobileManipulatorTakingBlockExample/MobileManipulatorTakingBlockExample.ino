#include <SoftwareSerial.h>

#define DEBUGGING_SERIAL_RX         5
#define DEBUGGING_SERIAL_TX         6
#define DEBUGGING_SERIAL_BAUDRATE   115200

SoftwareSerial DebuggingSerial( DEBUGGING_SERIAL_RX, DEBUGGING_SERIAL_TX ); // RX, TX

// ------------------ for motor
#include "MOS_S2Motor.h"

// ------------------ for mobilebase
#define MOTOR_ID_FL                 0x10 // 전방 왼쪽 모터 아이디
#define MOTOR_ID_FR                 0x06 // 전방 오른쪽 모터 아이디
#define MOTOR_ID_BL                 0x0E // 후방 왼쪽 모터 아이디
#define MOTOR_ID_BR                 0x05 // 후방 오른쪽 모터 아이디

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

#define ARM_SERVO_ID_1      0x0D // 매니퓰레이터 base모터
#define ARM_SERVO_ID_2      0x08
#define ARM_SERVO_ID_3      0x03
#define ARM_SERVO_ID_4      0x07 // 매니퓰레이터 엔드이펙터에 가장 가까운 모터

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

// ------------------ for PSD
#include "PSD.h"

// ------------------ for pixy2 camera
#include <Pixy2I2C.h>
Pixy2I2C pixy;

// ------------------ 

void setup() {
  DebuggingSerial.begin( DEBUGGING_SERIAL_BAUDRATE ); // 디버깅 시리얼 초기화
  Serial.begin( MOS_S2_MOTOR_BAUDRATE ); // 모터 제어 버스 초기화
  
  MOS_S2MotorSetTorque( MOTOR_ID_FL, 1 ); // 모바일베이스 모터 토크 on
  MOS_S2MotorSetTorque( MOTOR_ID_FR, 1 );
  MOS_S2MotorSetTorque( MOTOR_ID_BL, 1 );
  MOS_S2MotorSetTorque( MOTOR_ID_BR, 1 );

  MOS_S2MotorSetTorque( ARM_SERVO_ID_1, 1 ); // 매니퓰레이터 모터 토크 on
  MOS_S2MotorSetTorque( ARM_SERVO_ID_2, 1 );
  MOS_S2MotorSetTorque( ARM_SERVO_ID_3, 1 );
  MOS_S2MotorSetTorque( ARM_SERVO_ID_4, 1 );
  
  gripper.attach(SERVO_GRIPPER_PIN); // 그리퍼 사용 준비

  // 초기 팔 자세
  robotArmForwardMove( 0, 50, -142, 0 );
  // 그리퍼 열기
  gripper.write(GRIP_ANGLE_OPEN);

  delay(3000);
}

const float leftGoalPSDDistance = 18.0;
const float rightGoalPSDDistance = 18.0;

void loop() {
  static uint8_t step = 0;
  static bool isWaiting = 0;
  static uint32_t prevTime = millis();
  static uint32_t delayTime = 0;

  if( isWaiting )
  {
    if( millis() - prevTime >= delayTime )
    {
      isWaiting = 0;
      step++;
    }
  }
  else
  {
    switch( step )
    {
      case 0: // PSD로 거리 맞춤
      {
        float leftPSDDistance, rightPSDDistance;
        float leftDistanceError, rightDistanceError;
        bool leftMovingComplete = 0, rightMovingComplete = 0;
        
        if( getValueFromFrontPSDSensor( leftPSDDistance, rightPSDDistance, 20 ) )
        {
          // PSD 거리 오차 구하기
          leftDistanceError = leftGoalPSDDistance - leftPSDDistance;
          rightDistanceError = rightGoalPSDDistance - rightPSDDistance;
    
          if( abs( leftDistanceError ) >= 0.3 ) // (목표 거리 - 현재 거리)의 결과가 양수라면 후진해야 함, 음수라면 전진해야 함
          {
            // 바퀴 명령 예약
            runOneMotorWithSpeed( MOTOR_ID_FL, constrain( abs( leftDistanceError )*5 , 0, 50 )*( leftDistanceError > 0 ? -1 : 1 ) );
            runOneMotorWithSpeed( MOTOR_ID_BL, constrain( abs( leftDistanceError )*5 , 0, 50 )*( leftDistanceError > 0 ? -1 : 1 ) );
          }
          else
          {
            runOneMotorWithSpeed( MOTOR_ID_FL, 0 );
            runOneMotorWithSpeed( MOTOR_ID_BL, 0 );
            leftMovingComplete = 1;
          }
          if( abs( rightDistanceError ) >= 0.3 )
          {
            runOneMotorWithSpeed( MOTOR_ID_FR, constrain( abs( rightDistanceError )*5 , 0, 50 )*( rightDistanceError > 0 ? -1 : 1 ) );
            runOneMotorWithSpeed( MOTOR_ID_BR, constrain( abs( rightDistanceError )*5 , 0, 50 )*( rightDistanceError > 0 ? -1 : 1 ) );
          }
          else
          {
            runOneMotorWithSpeed( MOTOR_ID_FR, 0 );
            runOneMotorWithSpeed( MOTOR_ID_BR, 0 );
            rightMovingComplete = 1;
          }
    
          MOS_S2MotorSetSync();
    
          if( leftMovingComplete && rightMovingComplete )
          {
            isWaiting = 1;
            prevTime = millis();
            delayTime = 1000;
          }
        }

        break;
      }
        
      case 1: // 블록이 카메라에 보이도록 매니퓰레이터 자세 변경
      {
        robotArmForwardMove( 0, 54, -145, -41 );
        isWaiting = 1;
        prevTime = millis();
        delayTime = 1000;
        
        break;
      }
        
      case 2: // 블록을 카메라 화면 중앙에 위치시키기
      {
        pixy.ccc.getBlocks( 1, 1 << 0 ); // 시그니처가 1번인 블록의 정보만 가져오기
        if( pixy.ccc.numBlocks >= 1 ) // 블록이 1개 이상 감지되었다면
        {
          int16_t blockXPosError = 157/*카메라 화면 x축 중심값*/ - pixy.ccc.blocks->m_x;
          if( abs( blockXPosError ) >= 3 )
          {
            // blockXPosError가 양수면 좌로 이동, 음수면 우로 이동 해야 함
            runOneMotorWithSpeed( MOTOR_ID_FL, constrain( abs( blockXPosError )*5 , 0, 50 )*( blockXPosError > 0 ? -1 : 1 ) ); // 좌로 이동하려면 FL, FR, BL, BR순서로
            runOneMotorWithSpeed( MOTOR_ID_BL, constrain( abs( blockXPosError )*5 , 0, 50 )*( blockXPosError > 0 ? 1 : -1 ) ); // - + + - 값을 가지며,
            runOneMotorWithSpeed( MOTOR_ID_FR, constrain( abs( blockXPosError )*5 , 0, 50 )*( blockXPosError > 0 ? 1 : -1 ) ); // 우로 이동하려면 FL, FR, BL, BR순서로
            runOneMotorWithSpeed( MOTOR_ID_BR, constrain( abs( blockXPosError )*5 , 0, 50 )*( blockXPosError > 0 ? -1 : 1 ) ); // + - - + 값을 가짐
          }
          else
          {
            runOneMotorWithSpeed( MOTOR_ID_FL, 0 );
            runOneMotorWithSpeed( MOTOR_ID_BL, 0 );
            runOneMotorWithSpeed( MOTOR_ID_FR, 0 );
            runOneMotorWithSpeed( MOTOR_ID_BR, 0 );

            isWaiting = 1;
            prevTime = millis();
            delayTime = 1000;
          }
          MOS_S2MotorSetSync();
        }
        
        break;
      }

      case 3: // 그리퍼를 블록 위치로 이동
      {
        pixy.ccc.getBlocks( 1, 1 << 0 ); // 시그니처가 1번인 블록의 정보만 가져오기
        if( pixy.ccc.numBlocks >= 1 ) // 블록이 1개 이상 감지되었다면
        {
          if( pixy.ccc.blocks->m_y < 115 )
          {
            robotArmForwardMove( 0, -10, -112, 38 );
          }
          else
          {
            robotArmForwardMove( 0, -63, -115, 73 );
          }
        }
        
        isWaiting = 1;
        prevTime = millis();
        delayTime = 1000;
        break;
      }
        
      case 4: // 전진
      {
        runOneMotorWithSpeed( MOTOR_ID_FL, MOBILEBASE_SPEED );
        runOneMotorWithSpeed( MOTOR_ID_FR, MOBILEBASE_SPEED );
        runOneMotorWithSpeed( MOTOR_ID_BL, MOBILEBASE_SPEED );
        runOneMotorWithSpeed( MOTOR_ID_BR, MOBILEBASE_SPEED );

        MOS_S2MotorSetSync();

        isWaiting = 1;
        prevTime = millis();
        delayTime = 500;

        break;
      }
        
      case 5: // 정지
      {
        runOneMotorWithSpeed( MOTOR_ID_FL, 0 );
        runOneMotorWithSpeed( MOTOR_ID_FR, 0 );
        runOneMotorWithSpeed( MOTOR_ID_BL, 0 );
        runOneMotorWithSpeed( MOTOR_ID_BR, 0 );

        MOS_S2MotorSetSync();

        isWaiting = 1;
        prevTime = millis();
        delayTime = 500;

        break;
      }
        
      case 6: // 그리퍼 닫기
      {
        gripper.write(GRIP_ANGLE_CLOSE);

        isWaiting = 1;
        prevTime = millis();
        delayTime = 500;

        break;
      }
        
      case 7: // 후진
      {
        runOneMotorWithSpeed( MOTOR_ID_FL, -MOBILEBASE_SPEED );
        runOneMotorWithSpeed( MOTOR_ID_FR, -MOBILEBASE_SPEED );
        runOneMotorWithSpeed( MOTOR_ID_BL, -MOBILEBASE_SPEED );
        runOneMotorWithSpeed( MOTOR_ID_BR, -MOBILEBASE_SPEED );

        MOS_S2MotorSetSync();

        isWaiting = 1;
        prevTime = millis();
        delayTime = 1000;

        break;
      }
        
      case 8: // 정지
      {
        runOneMotorWithSpeed( MOTOR_ID_FL, 0 );
        runOneMotorWithSpeed( MOTOR_ID_FR, 0 );
        runOneMotorWithSpeed( MOTOR_ID_BL, 0 );
        runOneMotorWithSpeed( MOTOR_ID_BR, 0 );

        MOS_S2MotorSetSync();

        isWaiting = 1;
        prevTime = millis();
        delayTime = 1000;

        break;
      }

      case 9: // 팔 왼쪽 45도 바닥으로
      {
        robotArmForwardMove( -47, -82, -71, -21 );

        isWaiting = 1;
        prevTime = millis();
        delayTime = 1000;

        break;
      }

      case 10: // 그리퍼 열기
      {
        gripper.write(GRIP_ANGLE_OPEN);

        // 프로그램 종료
        while(1){}
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
