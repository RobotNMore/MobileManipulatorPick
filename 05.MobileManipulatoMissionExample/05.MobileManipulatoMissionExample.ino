//////////////  프로그램으로 구현된 시리얼통신
#include <SoftwareSerial.h>

#define SW_SERIAL_RX         5
#define SW_SERIAL_TX         6

SoftwareSerial SWSerial( SW_SERIAL_RX, SW_SERIAL_TX ); // RX, TX

//////////////  Pixy2 카메라
#include <Pixy2I2C.h> // I2C로 통신하는 Pixy2 라이브러리를 포함시킴

Pixy2I2C pixy;        // I2C로 통신하는 Pixy2 객체 생성

//////////////  그리퍼를 서서히 오므렸다 벌리기

#include <Servo.h>

#define PIN_SERVO_GRIPPER   11  // 그리퍼에 사용된 서보모터 제어 핀 번호

Servo gripper;

//////////////  모터 구동 라이브러리
#include "MOS_S2Motor.h"

//////////////  모터 구동용 변수/함수
#define MOTOR_ID_FL                 0x01 // 전방 좌측 모터 아이디
#define MOTOR_ID_FR                 0x02 // 전방 우측 모터 아이디
#define MOTOR_ID_BL                 0x03 // 후방 좌측 모터 아이디
#define MOTOR_ID_BR                 0x04 // 후방 우측 모터 아이디

#define MOTOR_FL_SPEED_RATIO        1.0 // 전방 좌측 모터 속도 보정용
#define MOTOR_FR_SPEED_RATIO        1.0 // 전방 우측 모터 속도 보정용
#define MOTOR_BL_SPEED_RATIO        1.0 // 후방 좌측 모터 속도 보정용
#define MOTOR_BR_SPEED_RATIO        1.0 // 후방 우측 모터 속도 보정용

#define MOBILEBASE_MOTOR_RELATIVE_MODE    1 // 속도, 위치를 상대값으로 사용하는 모드
                                            // 0이면 절대값으로 사용
#define MOBILEBASE_MOTOR_FREE_WHEEL       1 // 모바일베이스 모터 free wheel

#define MOTOR_VELOCITY_ABS_MIN_LIMIT      0   // 속도 최소값 (부호 없이)
#define MOTOR_VELOCITY_ABS_MAX_LIMIT      255 // 속도 최대값 (부호 없이)
#define MOTOR_VELOCITY_MAX_LIMIT          255 // 속도 최소값
#define MOTOR_VELOCITY_MIN_LIMIT         -255 // 속도 최소값

#define MOBILEBASE_SPEED                  100

/*
 * 나중에 제어할 지정 모터의 속도를 미리 설정하는 함수
 * (참고로, 나중에 MOS_S2MotorSetSync()를 호출해야 실제 동작이 실행됨)
 * motorID : 바퀴로 사용하려는 모터 아이디
 * wheelSpeed : 회전 속도. +는 전진 방향, -는 후진 방향.
 */
void runOneMotorWithSpeed( uint8_t motorID, int16_t wheelSpeed );

/*
 * 모바일베이스에 장착된 4개의 모터를 각각 지정 속도로 제어하는 함수
 * speedFL : 전방 좌측 바퀴 속도
 * speedFR : 전방 우측 바퀴 속도
 * speedBL : 후방 좌측 바퀴 속도
 * speedBR : 후방 우측 바퀴 속도
 */
void moveMobileBaseWithFourVelocity( int16_t speedFL, int16_t speedFR,
                                     int16_t speedBL, int16_t speedBR );

//////////////  모바일베이스 직진 제어용 변수/함수

float controlL = 0; // 왼쪽 바퀴 제어량
float controlR = 0; // 오른쪽 바퀴 제어량

float goalYawAngle = 0; // setPoint
float prevYawAngle = 0; // prevInput

float speedPGain = 2;   // pgain
float speedIGain = 0.3; // igain
float speedDGain = 0.4; // dgain

float iTerm = 0;
float output = 0;

/*
 * 머니퓰레이터 모바일베이스 구동 방향 ID 값
 */
#define DIRECTION_ID_STOP                1 // 정지
#define DIRECTION_ID_0_DEGREE            2 // 전진
#define DIRECTION_ID_180_DEGREE          3 // 후진
#define DIRECTION_ID_270_DEGREE          4 // 좌진
#define DIRECTION_ID_90_DEGREE           5 // 우진

/*
 * PID제어를 사용한 모바일베이스 직진 주행
 * direction : 방향 값
 * speed : 전방 우측 바퀴 속도
 * pidReset : 후방 좌측 바퀴 속도
 */
void moveMobileBaseWithDirection( uint8_t direction, int16_t speed,
                                  bool pidReset = 0 );

//////////////  매니퓰레이터 모터 구동용 변수/함수

#define ARM_SERVO_ID_1      0x05 // 매니퓰레이터 base모터
#define ARM_SERVO_ID_2      0x06
#define ARM_SERVO_ID_3      0x07
#define ARM_SERVO_ID_4      0x08 // 매니퓰레이터 엔드이펙터에 가장 가까운 모터

#define ARM_SERVO_1_VALUE_MIN_LIMIT    0      // 각 모터의 최소, 최대 값
#define ARM_SERVO_1_VALUE_MAX_LIMIT    1023
#define ARM_SERVO_2_VALUE_MIN_LIMIT    0
#define ARM_SERVO_2_VALUE_MAX_LIMIT    1023
#define ARM_SERVO_3_VALUE_MIN_LIMIT    0
#define ARM_SERVO_3_VALUE_MAX_LIMIT    1023
#define ARM_SERVO_4_VALUE_MIN_LIMIT    0
#define ARM_SERVO_4_VALUE_MAX_LIMIT    1023

#define TORQUE_DEFAULT                 2 // 매니퓰레이터 모터 토크 기본 값,
                                         // 0이 최대, 2가 최소 토크
#define ARM_SERVO_RELATIVE_MODE        1 // 매니퓰레이터 모터를 상대 위치 모드로
                                         // 사용할 때의 값
#define pi      3.141592
#define DTR(x)  (x)*(pi/180) // degree to radian
#define RTD(x)  (x)*(180/pi) // radian to degree

const float d = 155; // 바닥에서 매니퓰레이터 2번 모터 회전축까지의 거리
const float L1 = 75; // 2번과 3번 모터의 회전축간 거리
const float L2 = 75; // 3번과 4번 모터의 회전축간 거리
const float L3 = 81; // 4번 모터 회전축과 그리퍼 끝부분 사이의 거리

/*
 * 나중에 제어할 지정 모터의 각도(위치)를 미리 설정하는 함수
 * (참고로, 나중에 MOS_S2MotorSetSync()를 호출해야 실제 동작이 실행됨)
 * motorID : 매니퓰레이터로 사용하려는 모터 아이디
 * angle : 목표 각도. 0이 가운데 중심 각도(위치)
 *         1번 모터 : -는 CCW, +는 CW 방향
 *         2, 3, 4번 모터 : -는 매니퓰레이터가 내려가는 방향, 
 *                         +는 매니퓰레이터가 올라가는 방향
 */
void moveRobotArmServoWithAngle( uint8_t motorID, float angle );

/*
 * 매니퓰레이터에 장착된 4개의 모터를 각각 다른 위치로 제어하는 함수
 * a1 : 1번 모터 각도
 * a2 : 2번 모터 각도
 * a3 : 3번 모터 각도
 * a4 : 4번 모터 각도
 */
void robotArmForwardMove( float a1, float a2, float a3, float a4 );

/*
 * 그리퍼의 위치와 각도로 4-DOF 매니퓰레이터를 제어하는 함수
 * x : 그리퍼 좌우 위치
 * y : 그리퍼 전후 위치
 * z : 그리퍼 높이
 * angle : 그리퍼 각도
 */
void robotArmInverseMove( int16_t x, uint16_t y, uint16_t z, int16_t angle );

//////////////  timer
/*
 * deltaTime을 계산하는 함수
 * 현재 시간과 이전에 calcDT함수가 호출된 시간의 차이를 반환함
 */
float calcDT();
float dt = 0; // IMU와 PID제어에 공통으로 사용하기 위한 dt변수

//////////////  PID 제어기 함수

void pidController(float &setPoint, float &input, float &prevInput,
                    float &pGain, float &iGain, float &dGain,
                    float &iTerm, float &output);

//////////////  경기 시작 신호 관련

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

//////////////  IMU

#include "I2Cdev.h"  // MPU6050과의 I2C통신을 관리하는 라이브러리
#include "MPU6050.h"  // MPU6050 라이브러리

MPU6050 accelgyro;  // MPU6050객체 생성

int16_t ax, ay, az;  // 가속도계에서 읽어진 x, y, z축의 가속도 값
int16_t gx, gy, gz;  // 자이로스코프에서 읽어진 x, y, z축의 각속도 값

float yawAngle = 0;  // 초기 자세를 0도로 설정
#define IMU_ADC_TO_DEGREE  131.068 // 32767/250와 같음
                                   // 센서 값을 각도로 변환하는데 사용

//////////////  전방 PSD센서

#define PIN_LEFT_PSD      A3 // 매니퓰레이터 좌측 PSD센서 연결 핀
#define PIN_RIGHT_PSD     A2 // 매니퓰레이터 우측 PSD센서 연결 핀

#define LEFT_PSD_INDEX    0  // readings변수에서 좌측 PSD값이 저장된 인덱스
#define RIGHT_PSD_INDEX   1  // readings변수에서 우측 PSD값이 저장된 인덱스

#define PSD_CNT           2  // PSD가 두 개 있음 (좌/우)

#define GOAL_DISTANCE     12.0

//////////////  지수가중이동평균필터

const float prevValueWeight = 0.9;  // 값이 크면 필터링 효과 커짐
float distance[PSD_CNT];            // 센서값으로 구해진 거리
float result[PSD_CNT];              // 결과 값
            
//////////////  메인 프로그램

void setup() {
  Serial.begin(115200);
  SWSerial.begin(115200);

  // 매니퓰레이터 초기 자세로
  robotArmForwardMove( 0, 52, -104, -92 );
  delay(1000);

  // 카메라 사용 준비
  pixy.init(); // pixy 객체 초기화

  // 그리퍼 사용 준비
  gripper.attach( PIN_SERVO_GRIPPER );
  gripper.write(20); // 그리퍼 열기
  delay(500);

  // IMU 사용 준비
  Wire.begin(); // I2C 통신 사용 시작
  accelgyro.initialize(); // MPU6050사용을 위한 초기화
//  accelgyro.setXAccelOffset(-517); // offset 설정
//  accelgyro.setYAccelOffset(-1719); // 
//  accelgyro.setZAccelOffset(1072);
//  accelgyro.setXGyroOffset(-260); 
//  accelgyro.setYGyroOffset(-1);    
//  accelgyro.setZGyroOffset(23);
  accelgyro.CalibrateGyro();
  accelgyro.CalibrateAccel();

  // PSD사용 준비
  // 필터에 사용할 변수들 초기화
  for (int16_t thisPSD = 0; thisPSD < PSD_CNT; thisPSD++)
  {
    distance[thisPSD] = 0;
    result[thisPSD] = 0;
  }

  // PSD센서 필터링 한 결과 값이 0에서 시작해서
  // 현재의 실제 값에 맞춰지기 위해 미리 조금 읽어 놓기
  for(int i = 0 ; i < 50 ; i++)
  {
    // 현재 거리 값 계산
    distance[LEFT_PSD_INDEX] = 1/((analogRead(PIN_LEFT_PSD)*2.19e-4) - 6.8e-3);
    distance[RIGHT_PSD_INDEX] = 1/((analogRead(PIN_RIGHT_PSD)*2.54e-4) - 2.38e-2);
    
    // 이전 결과 값 비율을 줄이고 현재 거리 값을 결과에 반영하여 결과에 저장
    result[LEFT_PSD_INDEX] = prevValueWeight*result[LEFT_PSD_INDEX]
                              + (1 - prevValueWeight)*distance[LEFT_PSD_INDEX];
    result[RIGHT_PSD_INDEX] = prevValueWeight*result[RIGHT_PSD_INDEX]
                              + (1 - prevValueWeight)*distance[RIGHT_PSD_INDEX];
  }
}

uint8_t currentStep = 1;
uint32_t timeStartMoving = 0;

void loop() {
  // 현재 각도 계산
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  dt = calcDT();
  yawAngle = yawAngle + gz*dt/IMU_ADC_TO_DEGREE;

  if( currentStep == 1 ) // 일정 시간 동안 진행
  {
    if( startSignalSequence() == 1 )
    {
      currentStep ++;
    }
  }
  // 현재 미션 진행 단계가 1또는 4일 때
  else if( currentStep == 2 || currentStep == 5 ) // PSD로 거리 측정하며 전진
  {
    // 현재 거리 값 계산. 센서의 거리-전압 특성을 이용해 직선의 방정식을 계산 후 사용
    distance[LEFT_PSD_INDEX] = 1/((analogRead(PIN_LEFT_PSD)*2.19e-4) - 6.8e-3);
    distance[RIGHT_PSD_INDEX] = 1/((analogRead(PIN_RIGHT_PSD)*2.54e-4) - 2.38e-2);
    
    // 지수가중이동평균필터. 이전 결과 값 비율을 줄이고 현재 거리 값을 결과에 반영하여 결과에 저장
    result[LEFT_PSD_INDEX] = prevValueWeight*result[LEFT_PSD_INDEX]
                              + (1 - prevValueWeight)*distance[LEFT_PSD_INDEX];
    result[RIGHT_PSD_INDEX] = prevValueWeight*result[RIGHT_PSD_INDEX]
                              + (1 - prevValueWeight)*distance[RIGHT_PSD_INDEX];
  
    // 양쪽 평균 거리 값을 사용해 주행 속도와 방향 계산
    float distanceError = GOAL_DISTANCE - ( result[LEFT_PSD_INDEX]
                                            + result[RIGHT_PSD_INDEX] )/2;
  
    if( abs(distanceError) >= 1.0 )
    {
      int16_t drivingSpeed = distanceError*3; // 비례 제어
  
      // 거리 오차에 따라 계산된 주행 속도 drivingSpeed가 양수면 후진, 음수면 전진
      moveMobileBaseWithDirection( drivingSpeed > 0
                                   ? DIRECTION_ID_180_DEGREE
                                   : DIRECTION_ID_0_DEGREE,
                                   abs( drivingSpeed ) );
    }
    else
    {
      moveMobileBaseWithDirection( DIRECTION_ID_STOP, 0 );
      
      currentStep ++; // 다음 스텝으로
    }
  }
  else if( currentStep == 3 ) // 색상 체크 후 signature1이 있는 방향으로 회전
  {
    pixy.ccc.getBlocks();

    if( pixy.ccc.numBlocks == 2 )
    {
      // 시그니처가 1인 블록을 찾아 인덱스를 저장
      int16_t signature1BlockIdx = -1;

      if( pixy.ccc.blocks[0].m_signature == 1)
      {
        signature1BlockIdx = 0;
      }

      if( pixy.ccc.blocks[1].m_signature == 1)
      {
        if( signature1BlockIdx == -1 )
        {
          signature1BlockIdx = 1;
        }
        else // 시그니처1인 블록이 여러 개 인식됨. 잘못된 상황
        {
          SWSerial.println("시그니처가 1인 블록이 여러 개 인식됨");
          return ;
        }
      }

      if( signature1BlockIdx == -1 ) // 시그니처1인 블록이 인식되지 않았음. 잘못된 상황
      {
        SWSerial.println("시그니처가 1인 블록이 없음");
        return ;
      }
      else
      {
        if( pixy.ccc.blocks[0].m_x != pixy.ccc.blocks[1].m_x )
        {
          goalYawAngle = ( pixy.ccc.blocks[signature1BlockIdx].m_x
                         >= pixy.ccc.blocks[1 - signature1BlockIdx].m_x ) ? 
                       -90 : 90; // signature1인 블록의 x축 좌표상 위치 값이 
                                 // 더 크면 우회전,아니면 좌회전 준비 

          currentStep ++; // 다음 스텝으로
        }
        else // 두 블록의 x좌표상 위치가 같게 인식됨
        {
          SWSerial.println("두 블록의 위치 값이 같음");
          return ;
        }
      }
    }
    else // 블록이 2개로 인식되지 않음. 잘못된 상황
    {
      SWSerial.println("인식된 블록이 너무 많거나 적음");
      return ;
    }
  }
  else if( currentStep == 4 ) // 목표 각도까지 회전
  {
    float yawAngleError = goalYawAngle - yawAngle;

    if( abs(yawAngleError) >= 0.3 )
    {
      int16_t rotationSpeed = constrain( yawAngleError*3, -100, 100 );

      // yawAngleError가 양수일 때 좌회전 해야 함
      moveMobileBaseWithFourVelocity( -rotationSpeed, rotationSpeed,
                                      -rotationSpeed, rotationSpeed );
    }
    else
    {
      moveMobileBaseWithDirection( DIRECTION_ID_STOP, 0 );
      
      currentStep ++; // 다음 스텝으로
    }
  }
  else if( currentStep == 6 ) // 잡을 블록을 보는 자세로 매니퓰레이터 제어
  {
    robotArmForwardMove( 0, 52, -104, -92 ); // 필요 시 수정
    timeStartMoving = millis();
    
    currentStep ++; // 다음 스텝으로
  }
  else if( currentStep == 7 ) // 매니퓰레이터 움직이는동안 대기하기
  {
    if( millis() - timeStartMoving >= 1000 ) // 1 초가 지나면
    {
      currentStep ++; // 다음 스텝으로
    }
  }
  else if( currentStep == 8 ) // 잡을 블록을 카메라 중앙에 맞추기
  {
    pixy.ccc.getBlocks();
    
    if( pixy.ccc.numBlocks == 1 )
    {
      if( pixy.ccc.blocks[0].m_signature == 3 )
      {
        int16_t blockXPosError = 157 - pixy.ccc.blocks[0].m_x;

        if( abs(blockXPosError) >= 5 ) // 카메라 중앙으로부터의 블록 위치 오차
                                       // 절댓값이 5 이상이면
        {
          // 블록 위치 오차가 양수면 좌진
          moveMobileBaseWithDirection( blockXPosError > 0
                                       ? DIRECTION_ID_270_DEGREE
                                       : DIRECTION_ID_90_DEGREE,
                                       constrain( 10 + abs( blockXPosError ),
                                                  0, 100 ) );
        }
        else // 오차 절댓값이 5 미만이면
        {
          moveMobileBaseWithDirection( DIRECTION_ID_STOP, 0 );
      
          currentStep ++; // 다음 스텝으로
        }
      }
      else // 시그니처 3인 블록이 인식되지 않음. 잘못된 상황
      {
        moveMobileBaseWithDirection( DIRECTION_ID_STOP, 0 );
        SWSerial.println("시그니처가 1인 블록이 없음");
        return ;
      }
    }
    else // 블록 개수가 많거나 적음. 잘못된 상황
    {
      moveMobileBaseWithDirection( DIRECTION_ID_STOP, 0 );
      SWSerial.println("인식된 블록이 너무 많거나 적음");
      return ;
    }
  }
  else if( currentStep == 9 ) { // 블록을 잡기 직전 위치로
    robotArmInverseMove( 0, 190, 200, 0 );
    timeStartMoving = millis();
    
    currentStep ++; // 다음 스텝으로
  }
  else if( currentStep == 10 ) { // 매니퓰레이터 이동 완료 대기
    if( millis() - timeStartMoving >= 1500 ) // 1.5 초가 지나면
    {
      currentStep ++; // 다음 스텝으로
    }
  }
  else if( currentStep == 11 ) { // 블록을 잡는 위치로
    robotArmInverseMove( 0, 190, 135, 0 );
    timeStartMoving = millis();
    
    currentStep ++; // 다음 스텝으로
  }
  else if( currentStep == 12 ) { // 매니퓰레이터 이동 완료 대기
    if( millis() - timeStartMoving >= 1500 )
    {
      currentStep ++; // 다음 스텝으로
    }
  }
  else if( currentStep == 13 ) { // 그리퍼 닫기
    gripper.write(100);
    timeStartMoving = millis();
    
    currentStep ++; // 다음 스텝으로
  }
  else if( currentStep == 14 ) { // 그리퍼 이동 완료 대기
    if( millis() - timeStartMoving >= 500 )
    {
      currentStep ++; // 다음 스텝으로
    }
  }
  else if( currentStep == 15 ) { // 매니퓰레이터 들어 올리기
    robotArmInverseMove( 0, 190, 200, 0 );
    timeStartMoving = millis();
    
    currentStep ++; // 다음 스텝으로
  }
  else if( currentStep == 16 ) { // 매니퓰레이터 이동 완료 대기
    if( millis() - timeStartMoving >= 1500 )
    {
      timeStartMoving = millis();
      currentStep ++; // 다음 스텝으로
    }
  }
  else if( currentStep == 17 ) { // 모바일베이스 후진하고 1초 후 정지
    if( millis() - timeStartMoving < 1000 )
    {
      moveMobileBaseWithDirection( DIRECTION_ID_180_DEGREE,
                                   MOBILEBASE_SPEED/2 );
    }
    else
    {
      moveMobileBaseWithDirection( DIRECTION_ID_STOP, 0 );
      
      currentStep ++; // 다음 스텝으로
    }
  }
  else if( currentStep == 18 ) // 블록을 바닥에 내려놓기
  {
    robotArmInverseMove( 0, 90, 200, 0 );
    delay(1500);
    robotArmInverseMove( 90, 70, 200, 0 ); // 살짝 왼쪽으로 회전
    delay(1500);
    robotArmInverseMove( 110, 70, 110, -90 ); // 그리퍼 각도 수직 
    delay(1500);
    robotArmInverseMove( 120, 80, 40, -90 ); // 내리기 
    delay(1500);
    gripper.write(20); // 그리퍼 열기
    delay(500);
    robotArmInverseMove( 120, 70, 130, -90 ); // 올리기
    delay(1500);
    robotArmInverseMove( 0, 100, 220, 0 ); // 초기 자세
    delay(1500);
    
    while(1){} // 정지
  }
}

void runOneMotorWithSpeed( uint8_t motorID, int16_t wheelSpeed )
{
  uint8_t dir = 0;
  
  switch( motorID )
  {
    case MOTOR_ID_FL:  // 앞쪽 왼쪽 모터
    case MOTOR_ID_BL:  // 뒤쪽 왼쪽 모터
    
      if( motorID == MOTOR_ID_FL )
        wheelSpeed *= MOTOR_FL_SPEED_RATIO; // 속도 보정값 적용
      else // motorID == MOTOR_ID_BL
        wheelSpeed *= MOTOR_BL_SPEED_RATIO;
        
      if ( wheelSpeed < 0 ) // 속도가 음수이면
      {
        dir = MOS_S2_MOTOR_CW; // 후진
      }
      else // ( wheelSpeed > 0 ) or WheelSpeed == 0
      {
        dir = MOS_S2_MOTOR_CCW; // 양수값은 전진
      }
      break;
      
    case MOTOR_ID_FR:  // 앞쪽 오른쪽 모터
    case MOTOR_ID_BR:  // 뒤쪽 오른쪽 모터
    
      if( motorID == MOTOR_ID_FR )
        wheelSpeed *= MOTOR_FR_SPEED_RATIO; // 속도 보정값 적용
      else // motorID == MOTOR_ID_BR
        wheelSpeed *= MOTOR_BR_SPEED_RATIO;
        
      if ( wheelSpeed < 0 ) // 속도가 음수이면
      {
        dir = MOS_S2_MOTOR_CCW; // 후진
      }
      else // ( wheelSpeed > 0 ) or WheelSpeed == 0
      {
        dir = MOS_S2_MOTOR_CW; // 양수값은 전진
      }
      break;
      
    default: // 만약 위의 4개 모터에 해당되지 않으면 무시하고 리턴함
      return;
  }

  // 위에서 결정된 방향과 속도 제어특성을 해당 모터에 설정함
  MOS_S2MotorSetNextWheel( motorID, 0, 1, 1,
                           ! MOBILEBASE_MOTOR_RELATIVE_MODE,
                           ! MOBILEBASE_MOTOR_FREE_WHEEL, dir,
                           constrain( abs( wheelSpeed ),
                                      MOTOR_VELOCITY_ABS_MIN_LIMIT,
                                      MOTOR_VELOCITY_ABS_MAX_LIMIT ) );
}

void moveMobileBaseWithFourVelocity( int16_t speedFL, int16_t speedFR,
                                     int16_t speedBL, int16_t speedBR )
{
  runOneMotorWithSpeed( MOTOR_ID_FL, speedFL );  // 앞쪽 왼쪽 모터 설정
  runOneMotorWithSpeed( MOTOR_ID_FR, speedFR );  // 앞쪽 오른쪽 모터 설정
  runOneMotorWithSpeed( MOTOR_ID_BL, speedBL );  // 뒤쪽 왼쪽 모터 설정
  runOneMotorWithSpeed( MOTOR_ID_BR, speedBR );  // 뒤쪽 오른쪽 모터 설정
  
  MOS_S2MotorSetSync();  // 제어 실행
}

void moveMobileBaseWithDirection( uint8_t direction, int16_t speed,
                                  bool pidReset )
{
  if(pidReset)
  {
    controlL = 0; // 바퀴 제어량 초기화
    controlR = 0;
    
    prevYawAngle = 0; // prevInput 초기화
    
    iTerm = 0;
  }

  pidController( goalYawAngle, yawAngle, prevYawAngle,
          speedPGain, speedIGain, speedDGain,
          iTerm, output );

  controlL -= output/2;
  controlR += output/2;
  controlL = constrain( controlL, -70, 70 ); // 제어량은 -70~70으로 제한함
  controlR = constrain( controlR, -70, 70 );

  switch(direction)
  {
    case 1: // stop
      moveMobileBaseWithFourVelocity( 0, 0, 0, 0 );
      break;
    case 2: // 0 degree
      moveMobileBaseWithFourVelocity( speed + controlL, speed + controlR,
                                      speed + controlL, speed + controlR );
      break;
    case 3: // 180 degree
      moveMobileBaseWithFourVelocity( -speed - controlR, -speed - controlL,
                                      -speed - controlR, -speed - controlL );
      break;
    case 4: // 270 degree
      moveMobileBaseWithFourVelocity( -speed - controlR, speed + controlR,
                                       speed + controlL, -speed - controlL );
      break;
    case 5: // 90 degree
      moveMobileBaseWithFourVelocity(  speed + controlL, -speed - controlL,
                                      -speed - controlR, speed + controlR );
      break;
  }
}

void moveRobotArmServoWithAngle( uint8_t motorID, float angle )
{
  if( ! isnan( angle ) )
  {
    // 인자로 입력받은 각도를 -160~160범위의 각도에서 모터의 포지션 범위 0~1023으로 매핑
    
    uint16_t motorValueCalculated;
    
    if( motorID == ARM_SERVO_ID_3 )
    {
      // 픽의 매니퓰레이터 구조상 3번 모터만 반대
      motorValueCalculated = map( constrain( angle, -160, 160 ),
                                  160, -160, 0, 1023 );
    }
    else
    {
      motorValueCalculated = map( constrain( angle, -160, 160 ),
                                  -160, 160, 0, 1023 );
    }
    
    switch( motorID )
    {
      case ARM_SERVO_ID_1:
        if( motorValueCalculated < ARM_SERVO_1_VALUE_MIN_LIMIT
          || motorValueCalculated > ARM_SERVO_1_VALUE_MAX_LIMIT )
        {
          motorValueCalculated = constrain( motorValueCalculated, 
                                            ARM_SERVO_1_VALUE_MIN_LIMIT,
                                            ARM_SERVO_1_VALUE_MAX_LIMIT );
        }
        break;
      case ARM_SERVO_ID_2:
        if( motorValueCalculated < ARM_SERVO_2_VALUE_MIN_LIMIT
          || motorValueCalculated > ARM_SERVO_2_VALUE_MAX_LIMIT )
        {
          motorValueCalculated = constrain( motorValueCalculated,
                                            ARM_SERVO_2_VALUE_MIN_LIMIT,
                                            ARM_SERVO_2_VALUE_MAX_LIMIT );
        }
        break;
      case ARM_SERVO_ID_3:
        if( motorValueCalculated < ARM_SERVO_3_VALUE_MIN_LIMIT
          || motorValueCalculated > ARM_SERVO_3_VALUE_MAX_LIMIT )
        {
          motorValueCalculated = constrain( motorValueCalculated,
                                            ARM_SERVO_3_VALUE_MIN_LIMIT,
                                            ARM_SERVO_3_VALUE_MAX_LIMIT );
        }
        break;
      case ARM_SERVO_ID_4:
        if( motorValueCalculated < ARM_SERVO_4_VALUE_MIN_LIMIT
          || motorValueCalculated > ARM_SERVO_4_VALUE_MAX_LIMIT )
        {
          motorValueCalculated = constrain( motorValueCalculated,
                                            ARM_SERVO_4_VALUE_MIN_LIMIT,
                                            ARM_SERVO_4_VALUE_MAX_LIMIT );
        }
        break;
    }

    MOS_S2MotorSetNextPosition( motorID, 0, 1, 1, TORQUE_DEFAULT,
                               ! ARM_SERVO_RELATIVE_MODE, motorValueCalculated );
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

void robotArmInverseMove( int16_t x, uint16_t y, uint16_t z, int16_t angle )
{
  int16_t zFromMotor2 = z - d; // 계산의 편의를 위해 2번 모터 관절과 지면 사이의 거리를
                               // 무시한 z축(위/아래) 거리
  float yd = sqrt(x*x + y*y); // 매니퓰레이터를 측면에서 봤을 때 그리퍼의 전진 거리
  float y2 = yd - L3*cos(DTR(angle));
  float z2 = zFromMotor2 - L3*sin(DTR(angle)); // 매니퓰레이터를 측면에서 봤을 때
                                               // 2번째 링크가 끝나는 점의 위치
  // 세타1을 구하기 위해 사용되는 변수
  float k1 = L1 + L2*(y2*y2 + z2*z2 - L1*L1 - L2*L2)/(2*L1*L2);
  float k2 = L2*sqrt(1 - pow((y2*y2 + z2*z2 - L1*L1 - L2*L2)/(2*L1*L2), 2));
  float theta1 = 0, theta2 = 0, theta3 = 0, theta4 = 0; // 모터별 세타 변수

  theta1 = RTD(atan2(x, y));
  theta2 = RTD(atan2(z2, y2)) + RTD(atan2(k2, k1));
  theta3 = -1*RTD(atan2(sqrt(1 - pow((y2*y2 + z2*z2 - L1*L1 - L2*L2)/(2*L1*L2), 2)),
                     (y2*y2 + z2*z2 - L1*L1 - L2*L2)/(2*L1*L2)));
  theta4 = angle - theta2 - theta3;

  // 지면과의 각도 theta2를 모터 각도로 변환
  robotArmForwardMove(theta1, -90 + theta2, theta3, theta4);
}

float calcDT()
{
  static uint32_t prevTime = millis();
  uint32_t currentTime = millis();
  
  float dt;

  dt = (currentTime - prevTime)/1000.0; // 밀리초를 초 단위로 변환
  
  prevTime = currentTime;
  
  return dt;
}

void pidController(float &setPoint, float &input, float &prevInput,
                    float &pGain, float &iGain, float &dGain,
                    float &iTerm, float &output)
{
  float error;
  float dInput;
  float pTerm, dTerm;

  error = setPoint - input; // 오차 = 설정값 - 현재 입력값
  dInput = input - prevInput;
  prevInput = input; // 다음 주기에 사용하기 위해서 현재 입력값을 저장

  pTerm = pGain * error; // 비례항
  iTerm += iGain * error * dt; // 적분항
  dTerm = -dGain * dInput / dt; // 미분항 : 미분항은 외력에 의한 변경이므로
                             // setPoint에 의한 내부적인 요소를 제외하고 나면
                             // - 부호가 붙음
  output = pTerm + iTerm + dTerm; // 각각 p, i, d항을 합해 output을 만듦

  if( isinf(output) || isnan(output) ) // 결과 값이 숫자가 아닌 경우 무시함
    output = 0;
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
