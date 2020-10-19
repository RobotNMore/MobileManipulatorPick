// ------------------ for motor
#include "MOS_S2Motor.h"

// ------------------ for robot arm

#define ARM_SERVO_ID_1      0x05 // 매니퓰레이터 base모터
#define ARM_SERVO_ID_2      0x06
#define ARM_SERVO_ID_3      0x07
#define ARM_SERVO_ID_4      0x08 // 매니퓰레이터 엔드이펙터에 가장 가까운 모터

// ------------------ 

/*
 * 출발 신호를 로봇 LED로 표시하는 함수
 * 최소한의 지연시간으로 반복적으로 호출되어야 함
 * 초록색 0.5초 점등, 0.5초 소등을 3세트 반복하고 빨간색 0.5초 점등 후 소등되면 시퀀스가 종료
 * return : 시퀀스가 진행중이면 0이 반환되고, 시퀀스가 끝나면 1을 반환함
 */
uint8_t startSignalSequence();
/*
 * RGBFlag에 따라 모든 팔 모터에 LED색상을 변경하는 함수
 * RGBFlag : 0b00000000
 *           2번 비트는 R
 *           1번 비트는 G
 *           0번 비트(LSB)는 B
 *           나머지는 무시함
 */
void setArmMotorLEDColor( uint8_t RGBFlag );

void setup() {
  Serial.begin(115200);
}

void loop() {
  // delay를 사용한 방식
  MOS_S2MotorSetLED( ARM_SERVO_ID_1, 0, 1, 0 );
  MOS_S2MotorSetLED( ARM_SERVO_ID_2, 0, 1, 0 );
  MOS_S2MotorSetLED( ARM_SERVO_ID_3, 0, 1, 0 );
  MOS_S2MotorSetLED( ARM_SERVO_ID_4, 0, 1, 0 );
  delay(500);
  MOS_S2MotorSetLED( ARM_SERVO_ID_1, 0, 0, 0 );
  MOS_S2MotorSetLED( ARM_SERVO_ID_2, 0, 0, 0 );
  MOS_S2MotorSetLED( ARM_SERVO_ID_3, 0, 0, 0 );
  MOS_S2MotorSetLED( ARM_SERVO_ID_4, 0, 0, 0 );
  delay(500);
  
  MOS_S2MotorSetLED( ARM_SERVO_ID_1, 0, 1, 0 );
  MOS_S2MotorSetLED( ARM_SERVO_ID_2, 0, 1, 0 );
  MOS_S2MotorSetLED( ARM_SERVO_ID_3, 0, 1, 0 );
  MOS_S2MotorSetLED( ARM_SERVO_ID_4, 0, 1, 0 );
  delay(500);
  MOS_S2MotorSetLED( ARM_SERVO_ID_1, 0, 0, 0 );
  MOS_S2MotorSetLED( ARM_SERVO_ID_2, 0, 0, 0 );
  MOS_S2MotorSetLED( ARM_SERVO_ID_3, 0, 0, 0 );
  MOS_S2MotorSetLED( ARM_SERVO_ID_4, 0, 0, 0 );
  delay(500);
  
  MOS_S2MotorSetLED( ARM_SERVO_ID_1, 0, 1, 0 );
  MOS_S2MotorSetLED( ARM_SERVO_ID_2, 0, 1, 0 );
  MOS_S2MotorSetLED( ARM_SERVO_ID_3, 0, 1, 0 );
  MOS_S2MotorSetLED( ARM_SERVO_ID_4, 0, 1, 0 );
  delay(500);
  MOS_S2MotorSetLED( ARM_SERVO_ID_1, 0, 0, 0 );
  MOS_S2MotorSetLED( ARM_SERVO_ID_2, 0, 0, 0 );
  MOS_S2MotorSetLED( ARM_SERVO_ID_3, 0, 0, 0 );
  MOS_S2MotorSetLED( ARM_SERVO_ID_4, 0, 0, 0 );
  delay(500);
  
  MOS_S2MotorSetLED( ARM_SERVO_ID_1, 1, 0, 0 );
  MOS_S2MotorSetLED( ARM_SERVO_ID_2, 1, 0, 0 );
  MOS_S2MotorSetLED( ARM_SERVO_ID_3, 1, 0, 0 );
  MOS_S2MotorSetLED( ARM_SERVO_ID_4, 1, 0, 0 );
  delay(500);
  MOS_S2MotorSetLED( ARM_SERVO_ID_1, 0, 0, 0 );
  MOS_S2MotorSetLED( ARM_SERVO_ID_2, 0, 0, 0 );
  MOS_S2MotorSetLED( ARM_SERVO_ID_3, 0, 0, 0 );
  MOS_S2MotorSetLED( ARM_SERVO_ID_4, 0, 0, 0 );
  while(1){}
  
//  // delay를 사용하지 않는 방식
//  if( startSignalSequence() == 1 )
//  {
//    while(1){}
//  }
}

uint8_t startSignalSequence()
{
  static uint8_t step = 0;
  static uint32_t previousTime = 0;
  if ( millis() - previousTime >= 500 )
  {
    previousTime = millis();
    if ( step % 2 == 0 )
    {
      if ( step / 2 == 3 )
      {
        setArmMotorLEDColor( 2 );
      }
      else
      {
        setArmMotorLEDColor( 4 );
      }
    }
    else
    {
      setArmMotorLEDColor( 0 );
      if ( step / 2 == 3 )
      {
        step = 0;
        return 1;
      }
    }
    step ++;
  }
  return 0;
}

void setArmMotorLEDColor( uint8_t RGBFlag )
{
  MOS_S2MotorSetLED( ARM_SERVO_ID_1, RGBFlag & 0b00000100 , RGBFlag & 0b00000010 , RGBFlag & 0b00000001 );
  MOS_S2MotorSetLED( ARM_SERVO_ID_2, RGBFlag & 0b00000100 , RGBFlag & 0b00000010 , RGBFlag & 0b00000001 );
  MOS_S2MotorSetLED( ARM_SERVO_ID_3, RGBFlag & 0b00000100 , RGBFlag & 0b00000010 , RGBFlag & 0b00000001 );
  MOS_S2MotorSetLED( ARM_SERVO_ID_4, RGBFlag & 0b00000100 , RGBFlag & 0b00000010 , RGBFlag & 0b00000001 );
}
