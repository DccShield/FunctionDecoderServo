#ifndef SERVODRV_H_
#define SERVODRV_H_

#include <Servo.h>
#include "NmraDcc.h"

#define PIN_LED_DIV1 6  // D6 PD6 SERVO1用DIV(分岐) LED
#define PIN_LED_STR1 7  // D7 PD7 SERVO1用STR(直進) LED
#define PIN_LED_DIV2 8  // D8 PB0 SERVO2用DIV(分岐) LED
#define PIN_LED_STR2 9  // D9 PB1 SERVO2用STR(直進) LED
#define ON 1
#define OFF 0

struct ServoParameterPluseWidth
{
  float nowDeg; // 現在の角度
  int onDeg;    // ON時の角度
  int offDeg;   // OFF時の角度
  float onDelta;  // OFF->ONのスピード
  float offDelta; // ON->OFFのスピード
};

extern uint8_t gState_F0;
extern uint8_t gState_F1;
extern uint8_t gState_F2;
extern uint8_t gState_F3;
extern uint8_t gState_F4;
extern uint8_t gState_F5;
extern uint8_t gState_F6;
extern uint8_t gState_F7;
extern uint8_t gState_F8;
extern uint8_t gState_F9;
extern uint8_t gState_F10;
extern uint8_t gState_F11; 
extern uint8_t gState_F12;
extern uint8_t gState_F13;
extern uint8_t gState_F14; 
extern uint8_t gState_F15;
extern uint8_t gState_F16;
extern uint8_t gState_Function;


// 状態基底クラス
class ServoDriver
{
public:
  ServoDriver(void);
  Set(void);
  void stateCheck();
  int nowState();

  unsigned char ch;       // 0 or 1
  unsigned char port;     
  unsigned char onDeg;    // ON時の角度
  unsigned char offDeg;   // OFF時の角度
  unsigned char initDeg;  // 電源切る前の角度
  unsigned char onSpeed;  // OFF->ONのスピード
  unsigned char offSpeed; // ON->OFFのスピード
  unsigned char sdir;     // gDirの最新値保存用
  unsigned char cv;       // EEP-ROM アドレス
  unsigned char servoAddress;  // ファンクションアドレス
  unsigned char STR;      // STR LEDポート
  unsigned char DIV;      // DIV LEDポート
  
  unsigned int MinAngle;
  unsigned int MaxAngle;

  
//  uint8_t gState_Function;
  
private:
  void SVattach(void);
  void SVdetach(void);
  void gState(void);
  void ServoWite(int ref);
  void writeCV(void);  
  void led(void);
  
  char state = ST_STANDABY;    // ステート
  char updownFlg = 0;         // 0:up 1:down

  int nextDeg;
  float nextDelta;

  ServoParameterPluseWidth lPW;

  Servo   SERVO;  // create servo object to control a servo
  NmraDcc   Dcc;
  
  enum{
      ST_STANDABY = 0,
      ST_IDLE,
      ST_RUN,
  };

  enum{
    DOWN = 0,
    STY,
    UP,
  };
};

#endif
