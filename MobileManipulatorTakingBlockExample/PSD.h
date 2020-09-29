#ifndef PSD_H
#define PSD_H

// 좌, 우 PSD센서 핀 번호
#define PSD_SENSOR_LEFT_PIN                      A3
#define PSD_SENSOR_RIGHT_PIN                     A2

// 필터 보정용 변수
#define PSD_READING_COUNT_TO_CALC_THRESHOLD          800
#define PSD_DIVISOR_TO_CALC_THRESHOLD                5.0
uint16_t leftFilterThreshold = 7;
uint16_t rightFilterThreshold = 6;

// 거리 보정용 변수
// 각 거리( 10cm, 15cm, 20cm, 25cm, 30cm, 35cm )별로 읽어진 값을 배열에 저장
// 보간법을 이용한 거리계산 함수에 사용
#define PSD_DISTANCE_CALIBRATION_STEP                6
const uint16_t leftPSDValuePer10cm[PSD_DISTANCE_CALIBRATION_STEP] = { 638, 444, 337, 260, 214, 177 };
const uint16_t rightPSDValuePer10cm[PSD_DISTANCE_CALIBRATION_STEP] = { 573, 399, 301, 256, 206, 187 };

// PSD센서에서 값을 받을 때 타입( raw or distance ) 지정
#define PSD_RETURN_TYPE_DISTANCE                     0
#define PSD_RETURN_TYPE_VALUE                        1

// PSD센서에서 값을 받을 때 몇 번 읽어서 평균을 낼 것인지
#define PSD_DEFAULT_READING_COUNT                    20


// PSD값이 튀는걸 잡기 위한 Threshold를 조정하는 함수
// PSD센서와 장애물의 거리를 12cm정도로 두고 실행해야 함
void PSDFilterThresholdCalibration();

// PSD센서 값을 받아와서 변수에 저장하는 함수. 값을 읽는 중이라면 0을 반환하고,
// 정해진 횟수만큼 값을 읽고 평균을 내서 결과가 나왔다면 1이 반환됨(1이 반환되었을 때만 left와 right에 있는 값이 유효)
// left : left PSD센서 값을 저장할 변수
// right : right PSD센서 값을 저장할 변수
// countToRead : 몇 번 읽어 평균 낸 값을 받아올 것인지. 최대 약 400
// returnType : 
//      PSD_RETURN_TYPE_DISTANCE : cm로 받아옴
//      PSD_RETURN_TYPE_VALUE : adc값으로 받아옴
uint8_t getValueFromFrontPSDSensor( float& left, float& right, uint16_t countToRead = PSD_DEFAULT_READING_COUNT, bool returnType = PSD_RETURN_TYPE_DISTANCE );

// 점 세 개로 보간 식을 만드는 함수. x값을 입력하면 그에 대응하는 y값이 반환됨
// x0, y0 : 첫 번째 점의 x값과 y값
// x1, y1 : 두 번째 점의 x값과 y값
// x2, y2 : 세 번째 점의 x값과 y값
// x : y값을 구하려는 점의 x좌표. 이에 대응하는 y값이 반환됨 
float lagrangeInterpolation( float x0, float x1, float x2, float y0, float y1, float y2, float x );

// PSD센서를 선택하고 해당 PSD센서에서 나온 adc값으로 거리를 구하는 함수. 거리값이 반환됨
// PSDPin : 거리값을 받아오려는 PSD센서의 핀
// data : 거리값을 받아오려는 PSD센서에서 읽혀진 값
float GP2Y0A_getDistance( uint16_t PSDPin, unsigned int data);


void PSDFilterThresholdCalibration()
{
  uint16_t leftMin = 65535, leftMax = 0;
  uint16_t rightMin = 65535, rightMax = 0;
  
  for( int i = 0 ; i < PSD_READING_COUNT_TO_CALC_THRESHOLD ; i++ )
  {
    uint16_t leftPSD = analogRead( PSD_SENSOR_LEFT_PIN );
    uint16_t rightPSD = analogRead( PSD_SENSOR_RIGHT_PIN );
    if( leftPSD < leftMin )
      leftMin = leftPSD;
    else if( leftPSD > leftMax )
      leftMax = leftPSD;
    if( rightPSD < rightMin )
      rightMin = rightPSD;
    else if( rightPSD > rightMax )
      rightMax = rightPSD;
  }

  leftFilterThreshold = round( ( leftMax - leftMin )/PSD_DIVISOR_TO_CALC_THRESHOLD );
  rightFilterThreshold = round( ( rightMax - rightMin )/PSD_DIVISOR_TO_CALC_THRESHOLD );
}

uint8_t getValueFromFrontPSDSensor( float& left, float& right, uint16_t countToRead = 20, bool returnType = PSD_RETURN_TYPE_DISTANCE )
{
  static uint16_t count = 0;

  static uint32_t leftSum = 0;
  static uint32_t rightSum = 0;
  static uint16_t prevLeftPSD = 65535;
  static uint16_t prevRightPSD = 65535;

  uint16_t leftPSD = analogRead( PSD_SENSOR_LEFT_PIN );
  uint16_t rightPSD = analogRead( PSD_SENSOR_RIGHT_PIN );

  if( (int32_t)leftPSD - prevLeftPSD >= leftFilterThreshold ) // 튀는 값 필터링
  {
    leftPSD = prevLeftPSD;
  }
  else
  {
    prevLeftPSD = leftPSD;
  }
  if( (int32_t)rightPSD - prevRightPSD >= rightFilterThreshold )
  {
    rightPSD = prevRightPSD;
  }
  else
  {
    prevRightPSD = rightPSD;
  }

  leftSum += leftPSD;
  rightSum += rightPSD;
  
  count++;

  if( count == countToRead )
  {
    float leftAvg = (float)leftSum/countToRead;
    float rightAvg = (float)rightSum/countToRead;
  
    left = ( returnType == PSD_RETURN_TYPE_DISTANCE ) ? GP2Y0A_getDistance( PSD_SENSOR_LEFT_PIN, leftAvg ) : leftAvg;
    right = ( returnType == PSD_RETURN_TYPE_DISTANCE ) ? GP2Y0A_getDistance( PSD_SENSOR_RIGHT_PIN, rightAvg ) : rightAvg;

    count = 0;
    leftSum = rightSum = 0;
    prevLeftPSD = prevRightPSD = 65535;
    return 1;
  }
  return 0;
}

float lagrangeInterpolation( float x0, float x1, float x2, float y0, float y1, float y2, float x )
{
  float Lg0, Lg1, Lg2;

  float y;

  Lg0 = ((x - x1)*(x - x2)) / ((x0 - x1)*(x0 - x2));
  Lg1 = ((x - x0)*(x - x2)) / ((x1 - x0)*(x1 - x2));
  Lg2 = ((x - x0)*(x - x1)) / ((x2 - x0)*(x2 - x1));
   
  y = (Lg0 * y0) + (Lg1 * y1) + (Lg2 * y2);
   
  return y;
}

float GP2Y0A_getDistance( uint16_t PSDPin, unsigned int data)
{
  float dis;

  uint16_t *PSDValueArray;
  if( PSDPin == PSD_SENSOR_LEFT_PIN )
  {
    PSDValueArray = leftPSDValuePer10cm;
  }
  else // PSDPin == PSD_SENSOR_RIGHT_PIN
  {
    PSDValueArray = rightPSDValuePer10cm;
  }
  
  if( ( data <= PSDValueArray[0] ) && ( data > PSDValueArray[1] ) )
    dis = lagrangeInterpolation( PSDValueArray[0], PSDValueArray[1], PSDValueArray[2], 10, 15, 20, data );
  else if( ( data <= PSDValueArray[1] ) && ( data > PSDValueArray[2] ) )
    dis = lagrangeInterpolation( PSDValueArray[1], PSDValueArray[2], PSDValueArray[3], 15, 20, 25, data );
  else if( ( data <= PSDValueArray[2] ) && ( data > PSDValueArray[3] ) )
    dis = lagrangeInterpolation( PSDValueArray[2], PSDValueArray[3], PSDValueArray[4], 20, 25, 30, data );
  else if( ( data <= PSDValueArray[3] ) && ( data > PSDValueArray[4] ) )
    dis = lagrangeInterpolation( PSDValueArray[3], PSDValueArray[4], PSDValueArray[5], 25, 30, 35, data );
  else
    dis = 0;

   return dis;
}

#endif
