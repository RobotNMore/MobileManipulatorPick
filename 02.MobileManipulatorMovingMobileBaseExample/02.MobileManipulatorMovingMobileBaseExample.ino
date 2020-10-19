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

// ------------------

void setup() {
  DebuggingSerial.begin( DEBUGGING_SERIAL_BAUDRATE ); // 디버깅 시리얼 초기화
  Serial.begin( MOS_S2_MOTOR_BAUDRATE ); // 모터 제어 버스 초기화

  runOneMotorWithSpeed( MOTOR_ID_FL, 0 ); // 모바일베이스 정지
  runOneMotorWithSpeed( MOTOR_ID_FR, 0 );
  runOneMotorWithSpeed( MOTOR_ID_BL, 0 );
  runOneMotorWithSpeed( MOTOR_ID_BR, 0 );

  MOS_S2MotorSetTorque( MOTOR_ID_FL, 1 ); // 모바일베이스 모터 토크 on
  MOS_S2MotorSetTorque( MOTOR_ID_FR, 1 );
  MOS_S2MotorSetTorque( MOTOR_ID_BL, 1 );
  MOS_S2MotorSetTorque( MOTOR_ID_BR, 1 );

  delay(3000);
}

void loop() {
  // 전진
  runOneMotorWithSpeed( MOTOR_ID_FL, MOBILEBASE_SPEED ); // 이런 방식으로 이동시킬 수 있고
  runOneMotorWithSpeed( MOTOR_ID_FR, MOBILEBASE_SPEED );
  runOneMotorWithSpeed( MOTOR_ID_BL, MOBILEBASE_SPEED );
  runOneMotorWithSpeed( MOTOR_ID_BR, MOBILEBASE_SPEED );
  MOS_S2MotorSetSync();
  delay(2000);

  // 정지
  moveMobileBaseWithFourVelocity( 0, 0, 0, 0 ); // 이런 방식으로도 이동시킬 수 있음
  delay(1000);
  
  // 좌로 수평이동
  moveMobileBaseWithFourVelocity( MOBILEBASE_SPEED, -MOBILEBASE_SPEED, -MOBILEBASE_SPEED, MOBILEBASE_SPEED );
  delay(3000);

  // 정지
  moveMobileBaseWithFourVelocity( 0, 0, 0, 0 );
  delay(1000);

  // 후진
  moveMobileBaseWithFourVelocity( -MOBILEBASE_SPEED, -MOBILEBASE_SPEED, -MOBILEBASE_SPEED, -MOBILEBASE_SPEED );
  delay(3000);

  // 정지
  moveMobileBaseWithFourVelocity( 0, 0, 0, 0 );
  delay(1000);
  
  // 우로 수평이동
  moveMobileBaseWithFourVelocity( -MOBILEBASE_SPEED, MOBILEBASE_SPEED, MOBILEBASE_SPEED, -MOBILEBASE_SPEED );
  delay(3000);

  // 정지
  moveMobileBaseWithFourVelocity( 0, 0, 0, 0 );
  delay(1000);
  
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
