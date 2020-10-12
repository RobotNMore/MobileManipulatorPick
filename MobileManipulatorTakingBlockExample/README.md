# TakingBlockExample

### 프로그램 개요
PSD센서와 모바일베이스, 매니퓰레이터, Pixy2카메라를 이용해 적재함에 있는 물류미션물품을 꺼내고, 바닥에 내려놓는 예제입니다.

### 소스 파일 구성
 - MobileManipulatorTakingBlockExample.ino : 모바일베이스, 매니퓰레이터, 그리퍼를 사용하기 위한 변수와 함수가 정의되어있고, 메인 프로그램이 작성된 파일

 - PSD.h : PSD센서를 사용하기 위한 함수가 정의된 파일

 - MOS_S2Motor.h : 모터를 사용하기 위한 함수가 정의된 파일

 - MOS_S2MotorProtocolDefine.h : 모터 프로토콜에 관련된 상수 값들이 정의된 파일

### 사용 전 세팅
 - Pixy2 카메라가 I2C 인터페이스를 사용하도록 설정되어있어야 합니다. MobileManipulator_Pick의 README.md에서 Pixy2 카메라 관련 항목을 참고하시기 바랍니다.

 - 시그니처 1번으로 인식 된 블록을 잡도록 프로그램이 작성되어 있기 때문에 Pixy2 카메라의 signature1에 색상이 학습되어있어야 합니다. MobileManipulator_Pick의 README.md에서 Pixy2 카메라 관련 항목의 하위 항목인 Pixy2 물체 인식시키기를 참고하시기 바랍니다.

 - PSD센서를 사용하기 때문에 프로그램을 최초로 실행하기 전에 PSD센서 거리값 보정, 임계값 보정을 해야 합니다. 자세한 내용은 PSD.h 소스코드의 주석을 참고하시기 바랍니다.

 - 물류미션물품 적재함에 시그니처 1번으로 학습시킨 블록 한 개가 대회 규격대로 올려져 있고, Pick이 적재함을 바라보고 약 20cm 떨어진 상태로 프로그램이 시작해야 합니다.
