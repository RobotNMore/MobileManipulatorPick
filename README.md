# MobileManipulator_Pick

### 개요

이 저장소의 코드들은 각 물류미션물품을 정해진 순서에 맞춰 정해진 위치로 옮기는 물류분류로봇의 기본 동작들이 구현되어 있습니다.

MobileManipulator_Pick은 아두이노 우노 호환보드를 중심으로 전방향 구동이 가능한 메카넘 휠로 구성된 모바일베이스, 물류미션물품을 인식하기 위한 Pixy2카메라, 물류미션물품을 이동시키기 위한 4 자유도의 매니퓰레이터와 그리퍼, Pick의 이동 상태를 파악하기 위한 두 개의 PSD센서와 6축 IMU센서 그리고 블루투스 모듈로 구성됩니다.

 * 모바일 매니퓰레이터 Pick에 사용되는 스마트 서보 모터는 로보로보의 MOS-S2 제품을 사용하였습니다.\
 소스코드 안에서는 MOS_S2_MOTOR 또는 MOS_S2Motor로 명시되어 있습니다. 참고 부탁드립니다.
 * Serial은 Pick의 모터와 통신하는데 사용하기 때문에 센서 값이나 Pick의 주행 상태 확인, 디버깅을\
 위해서는 소프트웨어 시리얼과 블루투스 모듈을 사용해야 합니다.

코드는 네 개의 샘플 프로젝트로 구성되어 있습니다.

```01.MobileManipulatorStartSignalSequenceExample.ino``` 는 모터에 내장된 LED를 사용하여 출발 신호를 표시하는 예제입니다.

```02.MobileManipulatorMovingMobileBaseExample.ino``` 는 모바일베이스를 이용하여 전진, 후진 그리고 좌, 우 수평이동으로 사각형의 경로를 그리며 이동하는 예제입니다.

```03.MobileManipulatorMobileBaseAndIMUExample.ino``` 는 모바일베이스와 IMU를 이용해 Pick이 전진과 90도 좌회전을 반복하여 사각형을 그리며 이동하는 예제입니다.

```04.MobileManipulatorTakingBlockExample.ino``` 는 PSD센서와 모바일베이스, 매니퓰레이터, Pixy2카메라를 이용해 적재함에 있는 물류미션물품을 꺼내고, 바닥에 내려놓는 예제입니다.

이 프로젝트들을 빌드하여 Pick에서 테스트 하기 위해서는 기본적인 아두이노 개발환경을 설치하고 USB 포트를 통하여 보드와 연결하고 코드를 다운로드 할 준비가 되어 있어야 합니다. 다음 단락에서는 코드를 이 저장소에서 내려받고 아두이노 개발툴에서 샘플을 Pick으로 다운로드 하는 방법을 설명합니다.

아두이노 툴을 다운로드하고 설치하는 방법은 다른 인터넷의 많은 가이드들을 참고하기 바랍니다.

### 코드 다운로드

모든 코드는 Git 툴을 이용하거나 이 페이지에서 zip 파일로 다운로드 할수 있습니다.
파일목록 바로 위의 버튼들 중 "Clone or download" 을 선택하여 "Download ZIP" 메뉴를 선택하면 이 코드저장소의 모든 파일을 다운로드 받을 수 있습니다.
데스크탑에서 다운받은 zip 파일을 압축 풀어 둡니다.

### 외부라이브러리 설치

 - 물류미션물품을 구분하여 이동 순서와 이동 위치를 정하기 위해 Pixy2카메라를 사용하며, 이 카메라를 사용하기 위해 Pixy2 라이브러리를 사용합니다.\
라이브러리는 아래 링크에서 다운로드하실 수 있습니다.\
\
Pixy2 라이브러리 다운로드 : https://pixycam.com/downloads-pixy2/ \
\
링크를 클릭한 후, Arduino libraries and examples 항목에서 다운로드합니다.

 - Pick의 이동 상태 확인을 위해 IMU센서를 사용하며, 이 IMU센서를 사용하기 위해 I2CDev와 MPU6050라이브러리를 사용합니다.\
라이브러리는 아래 링크에서 다운로드하실 수 있습니다.\
\
I2Cdev와 MPU6050 Github 저장소 : https://github.com/jrowberg/i2cdevlib \
\
링크에서 zip파일을 다운받은 후, 압축을 해제합니다. 압축 해제된 폴더를 열어 안에 있는 Arduino 폴더의 안에 있는 I2Cdev라이브러리와 MPU6050라이브러리의 소스를 아두이노 툴에서 사용할 수 있도록 설치합니다.


### 빌드 및 다운로드

Pick의 보드의 후면에는 개발용 PC와 Pick을 연결하기 위한 RJ-45커넥터가 있습니다. 제공된 전용 케이블을 사용하여 연결해야 하며, 프로그램을 업로드 할 때 연결된 상태여야 합니다.\
Pick의 보드의 좌측면에는 보드의 전원 스위치(흰색 사각형)가 있습니다. 프로그램을 업로드 할 때 스위치가 눌려서 보드가 켜진 상태여야 합니다.

1. 아두이노 툴에서 Pick의 연결을 설정합니다. 보드 : Arduino Uno, 포트 : Pick과 연결된 시리얼 포트 선택
2. 업로드 버튼을 클릭하여 코드를 빌드하고 Pick으로 코드를 업로드합니다.

Pick의 우측면에는 Pick의 전체 전원(메인보드, 모터, 블루투스 모듈에 공급되는)을 켜고 끄는 스위치(빨간색)가 있습니다. 흰 점이 표시된 부분이 눌려있으면 켜진 상태이며, 프로그램을 업로드한 후 로봇이 동작하기 위해서는 이 스위치가 켜짐 상태여야 합니다.

### 코드 설명

각 예제 프로젝트의 README.md 파일과, 예제 소스코드의 주석을 참고하시기 바랍니다.

### 모듈 정보
 - 블루투스 : HC-06\
 datasheet : https://html.alldatasheet.co.kr/html-pdf/1179032/ETC1/HC-06/109/1/HC-06.html
 - PSD : GP2Y0A21YK0F\
 datasheet : https://global.sharp/products/device/lineup/data/pdf/datasheet/gp2y0a21yk_e.pdf
 - IMU : MPU6050\
 datasheet : https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf \
 register map : https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
 - 카메라 : Pixy2\
 documentation : https://docs.pixycam.com/wiki/doku.php?id=wiki:v2:start

### 기본 설정값

#### 블루투스 모듈
 - 모듈 이름 : RNM0000 (0000부분은 다른 숫자 값일 수 있음)
 - baudrate : 9600
 - password : 1234

#### 매니퓰레이터
매니퓰레이터의 모든 모터 position을 센터로 이동시켰을 때, 직선이 아닌 링크를 사용한 매니퓰레이터가 일직선이 되기 위해 6번 모터와 7번 모터에 offset이 설정되어있습니다.
 - 6번 모터 offset : 0x97
 - 7번 모터 offset : 0x0F

### 스마트 서보 모터 관련
#### SetPosition
 - 0이 CCW방향이고, 1023이 CW방향입니다.
 - 센터에서 CCW방향으로 약 170도, CW방향으로 약 170도 범위를 가집니다.

#### 모터 offset 설정
모터 위치값의 범위는 0~1023이고, 512 위치로 이동시키면 센터가 됩니다. 모터 센터가 맞지 않는 경우에는 모터 전원을 끈 상태에서 손으로 모터를 천천히 돌려 센터 위치에 맞춰주고, MOS_S2MotorSetZeroPosition( uint8_t id ) 함수를 사용하면 모터의 현재 위치를 센터 위치로 설정할 수 있습니다.

### Pixy2 카메라 관련
Pixy2 카메라는 물체 인식을 위한 시그니처 설정, 카메라 밝기 등의 고급 설정을 위해 PixyMon v2 프로그램을 사용합니다.
PixyMon v2 프로그램은 아래 링크에서 다운받을 수 있습니다.\
PixyMon v2 다운로드 링크 : https://pixycam.com/downloads-pixy2/

다운받은 후 PixyMon v2를 실행하고, "Configure(톱니바퀴 아이콘)"-"Pixy Parameters (saved on Pixy)"-"Interface"에서 "Data out port" 를 "I2C"로 꼭 설정해주어야 합니다.

그 외 설정은 Pixy2 문서를 참고해주시기 바랍니다.\
Pixy2 문서 : https://docs.pixycam.com/wiki/doku.php?id=wiki:v2:start \
Pixy2 물체 인식시키기 : https://docs.pixycam.com/wiki/doku.php?id=wiki:v2:teach_pixy_an_object_2 \
PixyMon v2 문서 : https://docs.pixycam.com/wiki/doku.php?id=wiki:v2:pixymon_index
