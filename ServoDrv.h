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

struct ServoParameter
{
  unsigned char ch;       // 0 or 1
  unsigned char port;     
  unsigned char onDeg;    // ON時の角度
  unsigned char offDeg;   // OFF時の角度
  unsigned char initDeg;  // 電源切る前の角度
  unsigned char onSpeed;  // OFF->ONのスピード
  unsigned char offSpeed; // ON->OFFのスピード
  unsigned char sdir;     // gDirの最新値保存用
  unsigned char servoAddress;  // ファンクションアドレス
  unsigned int MinAngle;
  unsigned int MaxAngle;
};
//ServoParameter *gServoParam = new ServoParameter[MAX_SERVO];

struct ServoParameterPluseWidth
{
  float nowDeg; // 現在の角度
  int onDeg;    // ON時の角度
  int offDeg;   // OFF時の角度
  float onDelta;  // OFF->ONのスピード
  float offDelta; // ON->OFFのスピード
};
//ServoParameterPluseWidth *gServoPW = new ServoParameterPluseWidth;



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
  ServoDriver(struct ServoParameter sp);
  void stateCheck();
  int nowState();
  
//  uint8_t gState_Function;
  
private:
  void SVattach(char ch);
  void SVdetach(char ch);
  void gState( void );
  void servoABwite(char ch, int ref);
  void writeCV(char ch);  
  void led(unsigned char ch);
  
  char state = ST_STANDABY;    // ステート
  char updownFlg = 0;         // 0:up 1:down
  char svch;
  char adr;

  int nextDeg;
  float nextDelta;

  ServoParameter lSP;
  ServoParameterPluseWidth lPW;
  
//  unsigned char port;

  float PwmRef;
  float deltPwm;            // 10msあたりのpwm増加量
  
  int nowPwm;
  int nextPwm;
  int styTime;

  Servo   ServoA;  // create servo object to control a servo
  Servo   ServoB;  // create servo object to control a servo
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
