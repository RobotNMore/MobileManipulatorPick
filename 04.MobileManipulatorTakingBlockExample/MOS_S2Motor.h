#ifndef MOS_S2_MOTOR_H
#define MOS_S2_MOTOR_H

#include "MOS_S2MotorProtocolDefine.h"

//#define MOS_S2_MOTOR_DEBUG

#define MOS_S2_MOTOR_SEND_BUFFER_SIZE        10
#define MOS_S2_MOTOR_RECV_BUFFER_SIZE        11

#define MOS_S2_MOTOR_RECV_TIMEOUT_DEFAULT    500

#define MOS_S2_MOTOR_CW                    0
#define MOS_S2_MOTOR_CCW                   1

#define MOS_S2_MOTOR_BAUDRATE                115200

#define NO_RESPONSE_COMMAND_RETRY       3
#define COMMAND_RETRY                   1

uint8_t MOS_S2MotorSendBuf[MOS_S2_MOTOR_SEND_BUFFER_SIZE] = {0,};

uint8_t MOS_S2MotorRecvBuf[MOS_S2_MOTOR_RECV_BUFFER_SIZE] = {0,};
uint8_t MOS_S2MotorRecvBufIndex = 0;
uint16_t MOS_S2MotorRecvTimeout = 500;

bool isRecvPacketBegan = 0;
uint8_t prevRecvData = 0;
uint8_t recvPacketLength = 0;

#define MOS_S2_MOTOR_BUS    Serial

// -----------------------------------------------------public

void MOS_S2MotorResetData( uint8_t id );
void MOS_S2MotorReboot( uint8_t id );
int16_t MOS_S2MotorGetStatus( uint8_t id );
void MOS_S2MotorSetID( uint8_t id, uint8_t idToChange );
void MOS_S2MotorSetPIDNoSave( uint8_t id, uint8_t pGain, uint8_t iGain, uint8_t dGain );
void MOS_S2MotorSetPosition( uint8_t id, bool ledR, bool ledG, bool ledB, uint8_t torque, bool relativeMode, uint16_t position );
void MOS_S2MotorSetWheel( uint8_t id, bool ledR, bool ledG, bool ledB, bool relativeMode, bool freeWheel, bool direction, uint8_t speed );
void MOS_S2MotorSetOffset( uint8_t id, uint8_t offset );
void MOS_S2MotorSetMargin( uint8_t id, uint8_t margin );
void MOS_S2MotorSetPositionLimit( uint8_t id, uint16_t minLimit, uint16_t maxLimit );
void MOS_S2MotorSetTorque( uint8_t id, bool torqueFlag );
void MOS_S2MotorSetLED( uint8_t id, bool ledR, bool ledG, bool ledB );
int16_t MOS_S2MotorGetPID( uint8_t id );
int16_t MOS_S2MotorGetTemperature( uint8_t id );
int16_t MOS_S2MotorGetPosition( uint8_t id );
int16_t MOS_S2MotorGetOffset( uint8_t id );
int16_t MOS_S2MotorGetMargin( uint8_t id );
int16_t MOS_S2MotorGetPositionLimit( uint8_t id );
void MOS_S2MotorSetSync();
void MOS_S2MotorSetNextPosition( uint8_t id, bool ledR, bool ledG, bool ledB, uint8_t torque, bool relativeMode, uint16_t position );
void MOS_S2MotorSetNextWheel( uint8_t id, bool ledR, bool ledG, bool ledB, bool relativeMode, bool freeWheel, bool direction, uint8_t speed );
void MOS_S2MotorSetBaudrate( uint8_t id, uint8_t baudrate );
void MOS_S2MotorSetZeroPosition( uint8_t id );
void MOS_S2MotorSetKineticMemoryCPS( uint8_t id, bool captureStartOrEnd, bool playRunOrStop );
void MOS_S2MotorSetKineticMemoryFlag( uint8_t id, bool initialFlag, bool autoRunFlag );
int16_t MOS_S2MotorGetKineticMemorySize( uint8_t id );

// -----------------------------------------------------private

void sendPacket( uint8_t retry = NO_RESPONSE_COMMAND_RETRY );
int16_t sendPacketAndRecvResponse();
void clearBuffer( uint8_t* buffer, uint8_t bufferSize );
uint8_t calcCheckSum( uint8_t* buffer );
void printBufferStatus( uint8_t* buffer, uint8_t bufferSize );
int16_t recvPacket();
int16_t parseData( uint8_t* buffer );

// ------------------------------------------------------------

void MOS_S2MotorResetData( uint8_t id )
{
  uint8_t index = 2;
  clearBuffer( MOS_S2MotorSendBuf, sizeof( MOS_S2MotorSendBuf ) );
  MOS_S2MotorSendBuf[index++] = id;
  MOS_S2MotorSendBuf[index++] = SIZE_RESET_DATA;
  MOS_S2MotorSendBuf[index++] = INST_RESET_DATA;
  sendPacket();
}

void MOS_S2MotorReboot( uint8_t id )
{
  uint8_t index = 2;
  clearBuffer( MOS_S2MotorSendBuf, sizeof( MOS_S2MotorSendBuf ) );
  MOS_S2MotorSendBuf[index++] = id;
  MOS_S2MotorSendBuf[index++] = SIZE_REBOOT;
  MOS_S2MotorSendBuf[index++] = INST_REBOOT;
  sendPacket();
}

int16_t MOS_S2MotorGetStatus( uint8_t id )
{
  uint8_t index = 2;
  clearBuffer( MOS_S2MotorSendBuf, sizeof( MOS_S2MotorSendBuf ) );
  MOS_S2MotorSendBuf[index++] = id;
  MOS_S2MotorSendBuf[index++] = SIZE_GET_STATUS;
  MOS_S2MotorSendBuf[index++] = INST_GET_STATUS;
  return sendPacketAndRecvResponse();
}

void MOS_S2MotorSetID( uint8_t id, uint8_t idToChange )
{
  uint8_t index = 2;
  clearBuffer( MOS_S2MotorSendBuf, sizeof( MOS_S2MotorSendBuf ) );
  MOS_S2MotorSendBuf[index++] = id;
  MOS_S2MotorSendBuf[index++] = SIZE_SET_ID;
  MOS_S2MotorSendBuf[index++] = INST_SET_ID;
  MOS_S2MotorSendBuf[index++] = idToChange;
  sendPacket();
}

void MOS_S2MotorSetPIDNoSave( uint8_t id, uint8_t pGain, uint8_t iGain, uint8_t dGain )
{
  uint8_t index = 2;
  clearBuffer( MOS_S2MotorSendBuf, sizeof( MOS_S2MotorSendBuf ) );
  MOS_S2MotorSendBuf[index++] = id;
  MOS_S2MotorSendBuf[index++] = SIZE_SET_PID_NO_SAVE;
  MOS_S2MotorSendBuf[index++] = INST_SET_PID_NO_SAVE;
  MOS_S2MotorSendBuf[index++] = pGain;
  MOS_S2MotorSendBuf[index++] = iGain;
  MOS_S2MotorSendBuf[index++] = dGain;
  sendPacket();
}

void MOS_S2MotorSetPosition( uint8_t id, bool ledR, bool ledG, bool ledB, uint8_t torque, bool relativeMode, uint16_t position )
{
  uint16_t data = 0;
  data = data | ledR << 15;
  data = data | ledG << 14;
  data = data | ledB << 13;
  torque = constrain( torque, 0, 2 );
  data = data | ( torque & 0x03 ) << 11;
  data = data | relativeMode << 10;
  data = data | ( position & 0x03FF );

  uint8_t index = 2;
  clearBuffer( MOS_S2MotorSendBuf, sizeof( MOS_S2MotorSendBuf ) );
  MOS_S2MotorSendBuf[index++] = id;
  MOS_S2MotorSendBuf[index++] = SIZE_SET_POSITION_OR_WHEEL;
  MOS_S2MotorSendBuf[index++] = INST_SET_POSITION_OR_WHEEL;
  MOS_S2MotorSendBuf[index++] = ( data & 0xFF00 ) >> 8;
  MOS_S2MotorSendBuf[index++] = data & 0x00FF;
  sendPacket();
}

void MOS_S2MotorSetWheel( uint8_t id, bool ledR, bool ledG, bool ledB, bool relativeMode, bool freeWheel, bool direction, uint8_t speed )
{
  uint8_t data = 0;
  data = data | ledR << 7;
  data = data | ledG << 6;
  data = data | ledB << 5;
  data = data | 1 << 4;
  data = data | 1 << 3;
  data = data | relativeMode << 2;
  data = data | freeWheel << 1;
  data = data | direction;

  uint8_t index = 2;
  clearBuffer( MOS_S2MotorSendBuf, sizeof( MOS_S2MotorSendBuf ) );
  MOS_S2MotorSendBuf[index++] = id;
  MOS_S2MotorSendBuf[index++] = SIZE_SET_POSITION_OR_WHEEL;
  MOS_S2MotorSendBuf[index++] = INST_SET_POSITION_OR_WHEEL;
  MOS_S2MotorSendBuf[index++] = data;
  MOS_S2MotorSendBuf[index++] = speed;
  sendPacket();
}

void MOS_S2MotorSetOffset( uint8_t id, uint8_t offset )
{
  uint8_t index = 2;
  clearBuffer( MOS_S2MotorSendBuf, sizeof( MOS_S2MotorSendBuf ) );
  MOS_S2MotorSendBuf[index++] = id;
  MOS_S2MotorSendBuf[index++] = SIZE_SET_OFFSET;
  MOS_S2MotorSendBuf[index++] = INST_SET_OFFSET;
  MOS_S2MotorSendBuf[index++] = offset;
  sendPacket();
}

void MOS_S2MotorSetMargin( uint8_t id, uint8_t margin )
{
  uint8_t index = 2;
  clearBuffer( MOS_S2MotorSendBuf, sizeof( MOS_S2MotorSendBuf ) );
  MOS_S2MotorSendBuf[index++] = id;
  MOS_S2MotorSendBuf[index++] = SIZE_SET_MARGIN;
  MOS_S2MotorSendBuf[index++] = INST_SET_MARGIN;
  MOS_S2MotorSendBuf[index++] = margin;
  sendPacket();
}

void MOS_S2MotorSetPositionLimit( uint8_t id, uint16_t minLimit, uint16_t maxLimit )
{
  minLimit = constrain( minLimit, 1, 0x03FF );
  maxLimit = constrain( maxLimit, 1, 0x03FF );

  uint8_t index = 2;
  clearBuffer( MOS_S2MotorSendBuf, sizeof( MOS_S2MotorSendBuf ) );
  MOS_S2MotorSendBuf[index++] = id;
  MOS_S2MotorSendBuf[index++] = SIZE_SET_POSITION_LIMIT;
  MOS_S2MotorSendBuf[index++] = INST_SET_POSITION_LIMIT;
  MOS_S2MotorSendBuf[index++] = ( minLimit & 0xFF00 ) >> 8;
  MOS_S2MotorSendBuf[index++] = minLimit & 0x00FF;
  MOS_S2MotorSendBuf[index++] = ( maxLimit & 0xFF00 ) >> 8;
  MOS_S2MotorSendBuf[index++] = maxLimit & 0x00FF;
  sendPacket();
}

void MOS_S2MotorSetTorque( uint8_t id, bool torqueFlag )
{
  uint8_t data = 0;
  data = data | torqueFlag;

  uint8_t index = 2;
  clearBuffer( MOS_S2MotorSendBuf, sizeof( MOS_S2MotorSendBuf ) );
  MOS_S2MotorSendBuf[index++] = id;
  MOS_S2MotorSendBuf[index++] = SIZE_SET_TORQUE;
  MOS_S2MotorSendBuf[index++] = INST_SET_TORQUE;
  MOS_S2MotorSendBuf[index++] = data;
  sendPacket();
}

void MOS_S2MotorSetLED( uint8_t id, bool ledR, bool ledG, bool ledB )
{
  uint8_t data = 0;
  data = data | ledR << 7;
  data = data | ledG << 6;
  data = data | ledB << 5;

  uint8_t index = 2;
  clearBuffer( MOS_S2MotorSendBuf, sizeof( MOS_S2MotorSendBuf ) );
  MOS_S2MotorSendBuf[index++] = id;
  MOS_S2MotorSendBuf[index++] = SIZE_SET_LED;
  MOS_S2MotorSendBuf[index++] = INST_SET_LED;
  MOS_S2MotorSendBuf[index++] = data;
  sendPacket();
}

int16_t MOS_S2MotorGetPID( uint8_t id )
{
  uint8_t index = 2;
  clearBuffer( MOS_S2MotorSendBuf, sizeof( MOS_S2MotorSendBuf ) );
  MOS_S2MotorSendBuf[index++] = id;
  MOS_S2MotorSendBuf[index++] = SIZE_GET_PID;
  MOS_S2MotorSendBuf[index++] = INST_GET_PID;
  return sendPacketAndRecvResponse();
}

int16_t MOS_S2MotorGetTemperature( uint8_t id )
{
  uint8_t index = 2;
  clearBuffer( MOS_S2MotorSendBuf, sizeof( MOS_S2MotorSendBuf ) );
  MOS_S2MotorSendBuf[index++] = id;
  MOS_S2MotorSendBuf[index++] = SIZE_GET_TEMPERATURE;
  MOS_S2MotorSendBuf[index++] = INST_GET_TEMPERATURE;
  return sendPacketAndRecvResponse();
}

int16_t MOS_S2MotorGetPosition( uint8_t id )
{
  uint8_t index = 2;
  clearBuffer( MOS_S2MotorSendBuf, sizeof( MOS_S2MotorSendBuf ) );
  MOS_S2MotorSendBuf[index++] = id;
  MOS_S2MotorSendBuf[index++] = SIZE_GET_POSITION;
  MOS_S2MotorSendBuf[index++] = INST_GET_POSITION;
  int16_t temp = sendPacketAndRecvResponse();
  return temp;
}

int16_t MOS_S2MotorGetOffset( uint8_t id )
{
  uint8_t index = 2;
  clearBuffer( MOS_S2MotorSendBuf, sizeof( MOS_S2MotorSendBuf ) );
  MOS_S2MotorSendBuf[index++] = id;
  MOS_S2MotorSendBuf[index++] = SIZE_GET_OFFSET;
  MOS_S2MotorSendBuf[index++] = INST_GET_OFFSET;
  return sendPacketAndRecvResponse();
}

int16_t MOS_S2MotorGetMargin( uint8_t id )
{
  uint8_t index = 2;
  clearBuffer( MOS_S2MotorSendBuf, sizeof( MOS_S2MotorSendBuf ) );
  MOS_S2MotorSendBuf[index++] = id;
  MOS_S2MotorSendBuf[index++] = SIZE_GET_MARGIN;
  MOS_S2MotorSendBuf[index++] = INST_GET_MARGIN;
  return sendPacketAndRecvResponse();
}

int16_t MOS_S2MotorGetPositionLimit( uint8_t id )
{
  uint8_t index = 2;
  clearBuffer( MOS_S2MotorSendBuf, sizeof( MOS_S2MotorSendBuf ) );
  MOS_S2MotorSendBuf[index++] = id;
  MOS_S2MotorSendBuf[index++] = SIZE_GET_POSITION_LIMIT;
  MOS_S2MotorSendBuf[index++] = INST_GET_POSITION_LIMIT;
  return sendPacketAndRecvResponse();
}

void MOS_S2MotorSetSync()
{
  uint8_t index = 2;
  clearBuffer( MOS_S2MotorSendBuf, sizeof( MOS_S2MotorSendBuf ) );
  MOS_S2MotorSendBuf[index++] = 0xFE;
  MOS_S2MotorSendBuf[index++] = SIZE_SET_SYNC;
  MOS_S2MotorSendBuf[index++] = INST_SET_SYNC;
  sendPacket();
}

void MOS_S2MotorSetNextPosition( uint8_t id, bool ledR, bool ledG, bool ledB, uint8_t torque, bool relativeMode, uint16_t position )
{
  uint16_t data = 0;
  data = data | ledR << 15;
  data = data | ledG << 14;
  data = data | ledB << 13;
  torque = constrain( torque, 0, 2 );
  data = data | ( torque & 0x03 ) << 11;
  data = data | relativeMode << 10;
  data = data | ( position & 0x03FF );

  uint8_t index = 2;
  clearBuffer( MOS_S2MotorSendBuf, sizeof( MOS_S2MotorSendBuf ) );
  MOS_S2MotorSendBuf[index++] = id;
  MOS_S2MotorSendBuf[index++] = SIZE_SET_NEXT_POSITION_OR_WHEEL;
  MOS_S2MotorSendBuf[index++] = INST_SET_NEXT_POSITION_OR_WHEEL;
  MOS_S2MotorSendBuf[index++] = ( data & 0xFF00 ) >> 8;
  MOS_S2MotorSendBuf[index++] = data & 0x00FF;
  sendPacket();
}

void MOS_S2MotorSetNextWheel( uint8_t id, bool ledR, bool ledG, bool ledB, bool relativeMode, bool freeWheel, bool direction, uint8_t speed )
{
  uint8_t data = 0;
  data = data | ledR << 7;
  data = data | ledG << 6;
  data = data | ledB << 5;
  data = data | 1 << 4;
  data = data | 1 << 3;
  data = data | relativeMode << 2;
  data = data | freeWheel << 1;
  data = data | direction;

  uint8_t index = 2;
  clearBuffer( MOS_S2MotorSendBuf, sizeof( MOS_S2MotorSendBuf ) );
  MOS_S2MotorSendBuf[index++] = id;
  MOS_S2MotorSendBuf[index++] = SIZE_SET_NEXT_POSITION_OR_WHEEL;
  MOS_S2MotorSendBuf[index++] = INST_SET_NEXT_POSITION_OR_WHEEL;
  MOS_S2MotorSendBuf[index++] = data;
  MOS_S2MotorSendBuf[index++] = speed;
  sendPacket();
}

void MOS_S2MotorSetBaudrate( uint8_t id, uint8_t baudrate )
{
  uint8_t data = baudrate;

  switch( data )
  {
    case 0x01:
    case 0x02:
    case 0x04:
    case 0x06:
    case 0x0C:
    case 0x18:
    case 0x30:
    case 0x60:
      break;
    default :
      data = 0;
  }

  uint8_t index = 2;
  clearBuffer( MOS_S2MotorSendBuf, sizeof( MOS_S2MotorSendBuf ) );
  MOS_S2MotorSendBuf[index++] = id;
  MOS_S2MotorSendBuf[index++] = SIZE_SET_BAUDRATE;
  MOS_S2MotorSendBuf[index++] = INST_SET_BAUDRATE;
  MOS_S2MotorSendBuf[index++] = data;
  sendPacket();
}

void MOS_S2MotorSetZeroPosition( uint8_t id )
{
  uint8_t index = 2;
  clearBuffer( MOS_S2MotorSendBuf, sizeof( MOS_S2MotorSendBuf ) );
  MOS_S2MotorSendBuf[index++] = id;
  MOS_S2MotorSendBuf[index++] = SIZE_SET_ZERO_POSITION;
  MOS_S2MotorSendBuf[index++] = INST_SET_ZERO_POSITION;
  sendPacket();
}

void MOS_S2MotorSetKineticMemoryCPS( uint8_t id, bool captureStartOrEnd, bool playRunOrStop )
{
  uint8_t data = 0;
  data = data | captureStartOrEnd << 1;
  data = data | playRunOrStop;

  uint8_t index = 2;
  clearBuffer( MOS_S2MotorSendBuf, sizeof( MOS_S2MotorSendBuf ) );
  MOS_S2MotorSendBuf[index++] = id;
  MOS_S2MotorSendBuf[index++] = SIZE_SET_KINETIC_MEMORY_CPS;
  MOS_S2MotorSendBuf[index++] = INST_SET_KINETIC_MEMORY_CPS;
  MOS_S2MotorSendBuf[index++] = data;
  sendPacket();
}

void MOS_S2MotorSetKineticMemoryFlag( uint8_t id, bool initialFlag, bool autoRunFlag )
{
  uint8_t data = 0;
  data = data | initialFlag << 1;
  data = data | autoRunFlag;

  uint8_t index = 2;
  clearBuffer( MOS_S2MotorSendBuf, sizeof( MOS_S2MotorSendBuf ) );
  MOS_S2MotorSendBuf[index++] = id;
  MOS_S2MotorSendBuf[index++] = SIZE_SET_KINETIC_MEMORY_FLAG;
  MOS_S2MotorSendBuf[index++] = INST_SET_KINETIC_MEMORY_FLAG;
  MOS_S2MotorSendBuf[index++] = data;
  sendPacket();
}

int16_t MOS_S2MotorGetKineticMemorySize( uint8_t id )
{
  uint8_t index = 2;
  clearBuffer( MOS_S2MotorSendBuf, sizeof( MOS_S2MotorSendBuf ) );
  MOS_S2MotorSendBuf[index++] = id;
  MOS_S2MotorSendBuf[index++] = SIZE_GET_KINETIC_MEMORY_SIZE;
  MOS_S2MotorSendBuf[index++] = INST_GET_KINETIC_MEMORY_SIZE;
  return sendPacketAndRecvResponse();
}

void sendPacket( uint8_t retry )
{
  uint8_t packetSize = MOS_S2MotorSendBuf[3];

  // header
  MOS_S2MotorSendBuf[0] = 0xFF;
  MOS_S2MotorSendBuf[1] = 0xFF;

  // checksum
  MOS_S2MotorSendBuf[packetSize - 1] = calcCheckSum( MOS_S2MotorSendBuf );

  for( int j = 0 ; j < retry ; j++ )
  {
    for( int i = 0 ; i < packetSize ; i++ )
    {
      MOS_S2_MOTOR_BUS.write( MOS_S2MotorSendBuf[i] );
    }
  }
}

/*
 * 응답을 정상적으로 받으면 결과값을 return
 * 타임아웃이면 -1을 return
 */
int16_t sendPacketAndRecvResponse()
{
  sendPacket( COMMAND_RETRY );

  uint32_t startTime = millis();
  bool isReceived = 0;

  while( millis() - startTime < MOS_S2_MOTOR_RECV_TIMEOUT_DEFAULT )
  {
    int16_t result = recvPacket();
    if( result != -1 ) // 파싱 준비되지 않음/파싱 실패
    {
      // recvPacket안에서 parsing함
      return result;
    }
  }
  return -1; // timeout
}

void clearBuffer( uint8_t* buffer, uint8_t bufferSize )
{
  for( int i = 0 ; i < bufferSize ; i++ )
  {
    buffer[i] = 0;
  }
}

uint8_t calcCheckSum( uint8_t* buffer )
{
  uint8_t sum = 0, checksum = 0;
  uint8_t dataLength = buffer[3];
  for( int i = 0 ; i < dataLength - 1 ; i++ )
  {
    sum += buffer[i];
  }
  checksum = ~sum;
  checksum += 0x01;
  return checksum;
}

void printBufferStatus( uint8_t* buffer, uint8_t bufferSize )
{
  for( int i = 0 ; i < bufferSize ; i++ ) {
#ifdef MOS_S2_MOTOR_DEBUG
    SWSerial.print( buffer[i], HEX );
#endif
    if( i == bufferSize - 1 )
    {
#ifdef MOS_S2_MOTOR_DEBUG
      SWSerial.println();
#endif
    }
    else {
#ifdef MOS_S2_MOTOR_DEBUG
      SWSerial.print(", ");
#endif
    }
  }
}

/*
 * 파싱 되면 결과값을 return
 * 파싱 준비되지 않았으면 -1을 return
 */
int16_t recvPacket()
{
  if( MOS_S2_MOTOR_BUS.available() )
  {
    uint8_t recvData = MOS_S2_MOTOR_BUS.read();
    if( !isRecvPacketBegan )
    {
      if( recvData == 0xFF )
      {
        if( prevRecvData == 0xFF ) // 0xFF 0xFF 헤더 발견
        {
          isRecvPacketBegan = 1;
          MOS_S2MotorRecvBuf[0] = 0xFF;
          MOS_S2MotorRecvBuf[1] = 0xFF;
          MOS_S2MotorRecvBufIndex = 2;
        }
      }
    }
    else
    {
      MOS_S2MotorRecvBuf[MOS_S2MotorRecvBufIndex] = recvData;
      if( MOS_S2MotorRecvBufIndex == 3 )
      {
        recvPacketLength = recvData;
      }
      if( recvPacketLength != 0 )
      {
        if( MOS_S2MotorRecvBufIndex == recvPacketLength - 1 )
        {
          uint8_t checksum = calcCheckSum( MOS_S2MotorRecvBuf );
          int16_t result = 0;
          if( checksum == MOS_S2MotorRecvBuf[recvPacketLength - 1] )
          {
            result = parseData( MOS_S2MotorRecvBuf ); // 잘못된 ackvalue면 -1을 리턴
          }
          clearBuffer( MOS_S2MotorRecvBuf, sizeof( MOS_S2MotorRecvBuf ) );
          isRecvPacketBegan = 0;
          prevRecvData = 0;
          recvPacketLength = 0;
          MOS_S2MotorRecvBufIndex = 0;
          return result;
        }
      }
      MOS_S2MotorRecvBufIndex ++;
    }
    prevRecvData = recvData;
  }
  return -1; // 파싱 준비되지 않음
}

/*
 * 잘못된 ackvalue일 때는 -1을 리턴.
 * 값이 없거나 값이 여러 개 들어오는 경우
 *   (예를 들면 PID값을 요청하면 3개를 받아야 함) 0을 리턴
 * 값이 한 개만 들어오면 해당 값을 리턴
 */
int16_t parseData( uint8_t* buffer )
{
  uint8_t motorID = buffer[2];
  uint8_t recvPacketLength = buffer[3];
  uint8_t ackValue = buffer[4];

  int16_t result = -1;

  switch( ackValue )
  {
    case ACK_GET_STATUS:
#ifdef MOS_S2_MOTOR_DEBUG
      SWSerial.println( F( "ack for getStatus is received" ) );
      printBufferStatus( MOS_S2MotorRecvBuf, sizeof( MOS_S2MotorRecvBuf ) );
#endif
      result = 0;
      break;
    case ACK_GET_PID:
#ifdef MOS_S2_MOTOR_DEBUG
      SWSerial.println( F( "ack for getPID is received" ) );
      SWSerial.print( F( "motorID " ) );
      SWSerial.print( buffer[2] );
      SWSerial.print( F( "'s pGain : " ) );
      SWSerial.print( buffer[6] );
      SWSerial.print( F( ", iGain : " ) );
      SWSerial.print( buffer[7] );
      SWSerial.print( F( ", dGain : " ) );
      SWSerial.println( buffer[8] );
      printBufferStatus( MOS_S2MotorRecvBuf, sizeof( MOS_S2MotorRecvBuf ) );
#endif
      result = 0;
      break;
    case ACK_GET_TEMPERATURE:
#ifdef MOS_S2_MOTOR_DEBUG
      SWSerial.println( F( "ack for getTemperature is received" ) );
      SWSerial.print( F( "motorID " ) );
      SWSerial.print( buffer[2] );
      SWSerial.print( F( "'s temperature : " ) );
      SWSerial.println( buffer[6] );
      printBufferStatus( MOS_S2MotorRecvBuf, sizeof( MOS_S2MotorRecvBuf ) );
#endif
      result = buffer[6];
      break;
    case ACK_GET_POSITION:
#ifdef MOS_S2_MOTOR_DEBUG
      SWSerial.println( F( "ack for getPosition is received" ) );
      SWSerial.print( F( "motorID " ) );
      SWSerial.print( buffer[2] );
      SWSerial.print( F( "'s position : " ) );
      SWSerial.println( (uint16_t)( buffer[6] << 8 | buffer[7] ) );
      printBufferStatus( MOS_S2MotorRecvBuf, sizeof( MOS_S2MotorRecvBuf ) );
#endif
      result = buffer[6] << 8 | buffer[7];
      break;
    case ACK_GET_OFFSET:
#ifdef MOS_S2_MOTOR_DEBUG
      SWSerial.println( F( "ack for getOffset is received" ) );
      SWSerial.print( F( "motorID " ) );
      SWSerial.print( buffer[2] );
      SWSerial.print( F( "'s offset : " ) );
      SWSerial.println( buffer[6] );
      printBufferStatus( MOS_S2MotorRecvBuf, sizeof( MOS_S2MotorRecvBuf ) );
#endif
      result = buffer[6];
      break;
    case ACK_GET_MARGIN:
#ifdef MOS_S2_MOTOR_DEBUG
      SWSerial.println( F( "ack for getMargin is received" ) );
      SWSerial.print( F( "motorID " ) );
      SWSerial.print( buffer[2] );
      SWSerial.print( F( "'s margin : " ) );
      SWSerial.println( buffer[6] );
      printBufferStatus( MOS_S2MotorRecvBuf, sizeof( MOS_S2MotorRecvBuf ) );
#endif
      result = buffer[6];
      break;
    case ACK_GET_POSITION_LIMIT:
#ifdef MOS_S2_MOTOR_DEBUG
      SWSerial.println( F( "ack for getPositionLimit is received" ) );
      SWSerial.print( F( "motorID " ) );
      SWSerial.print( buffer[2] );
      SWSerial.print( F( "'s min position limit : " ) );
      SWSerial.print( (uint16_t)( buffer[6] << 8 | buffer[7] ) );
      SWSerial.print( F( ", max position limit : " ) );
      SWSerial.println( (uint16_t)( buffer[8] << 8 | buffer[9] ) );
      printBufferStatus( MOS_S2MotorRecvBuf, sizeof( MOS_S2MotorRecvBuf ) );
#endif
      result = 0;
      break;
    case ACK_GET_KINETIC_MEMORY_SIZE:
#ifdef MOS_S2_MOTOR_DEBUG
      SWSerial.println( F( "ack for getKineticMemoryGetSize is received" ) );
      SWSerial.print( F( "motorID " ) );
      SWSerial.print( buffer[2] );
      SWSerial.print( F( "'s auto-run flag : " ) );
      SWSerial.print( buffer[6] >> 7 | 0x00FF );
      SWSerial.print( F( ", size : " ) );
      SWSerial.println( (uint16_t)( buffer[6] << 8 | buffer[7] ) );
      printBufferStatus( MOS_S2MotorRecvBuf, sizeof( MOS_S2MotorRecvBuf ) );
#endif
      result = buffer[6] << 8 | buffer[7]; // 2바이트 안에 플래그랑 사이즈가 같이 나옴
      break;
  }
#ifdef MOS_S2_MOTOR_DEBUG
  SWSerial.println();
#endif
  return result;
}

#endif
