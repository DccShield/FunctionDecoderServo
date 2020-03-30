//------------------------------------------------------------------------------
// ServoSequenceクラス
//------------------------------------------------------------------------------
#include <arduino.h>
#include "ServoDrv.h"

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// コンストラクタ
ServoDriver::ServoDriver()
{
  // 何もしない
}
ServoDriver::Set()
{
  pinMode(port, OUTPUT);
  digitalWrite(port, HIGH);

  lPW.onDeg = map((float)onDeg,offDeg,onDeg,MinAngle,MaxAngle);    // ON時の角度
  lPW.offDeg = map((float)offDeg,offDeg,onDeg,MinAngle,MaxAngle);   // OFF時の角度
  lPW.onDelta = mapfloat(((float)(onDeg - offDeg) / onSpeed / 100),(float)offDeg,(float)onDeg,0,(float)(MaxAngle-MinAngle)); // offDegからonDegまでの10ms時の移動量を算出
  lPW.offDelta = mapfloat(((float)(offDeg - onDeg) / offSpeed / 100),(float)offDeg,(float)onDeg,0,(float)(MaxAngle-MinAngle)); // offDegからonDegまでの10ms時の移動量を算出
  lPW.nowDeg = 0;

  SERVO.attach(port, MinAngle, MaxAngle);
  delay(20);
  SERVO.detach();

  state = ST_STANDABY;
}

int ServoDriver::nowState()
{
  return state; 
}


void ServoDriver::writeCV(void)
{
  Dcc.setCV(cv,gState_Function);        // 最終値のアクセサリ番号をCV_sdirに書き込み
}

void ServoDriver::SVattach(char ch)
{

  SERVO.attach(port, MinAngle, MaxAngle);
}

void ServoDriver::servoABwite(char ch, int ref)
{
  SERVO.writeMicroseconds(ref);
}

void ServoDriver::SVdetach(char ch)
{
  SERVO.detach();  
}

void ServoDriver::led( void )
{
  if(gState_Function == 0){ //0:div/ 1:str|
    digitalWrite(STR,OFF);
    digitalWrite(DIV,ON);
  } else {
    digitalWrite(STR,ON);
    digitalWrite(DIV,OFF);
  }
}

// ServoSequence ステートマシン（状態遷移）
void ServoDriver::stateCheck()
{
  gState();    
        
  switch(state){
      case ST_STANDABY:               // 起動時一回だけの処理
        led();
        if(gState_Function == sdir ){ // 前回最後のSTR/DIVが同じ？
          if(gState_Function == 0){   // OFF?
            SVattach(ch);
            lPW.nowDeg = lPW.offDeg;
            servoABwite(ch,(int)lPW.nowDeg);
          } else {                    // ON?
            SVattach(ch);
            lPW.nowDeg = lPW.onDeg;
            servoABwite(ch,(int)lPW.nowDeg);
          }
          SVdetach(ch);
          state = ST_IDLE;
          break;
        } else { // EEPROMの状態とコマンドステーションが異なっていた時の初回処理
          if(sdir != 0 and gState_Function == 0){
              nextDeg = lPW.offDeg;
              nextDelta = lPW.offDelta;
              state = ST_RUN;         
          } else {
              nextDeg = lPW.onDeg;
              nextDelta = lPW.onDelta;
              state = ST_RUN;
          }
        }
        break;

      case ST_IDLE: // 1 
     
            if(gState_Function == 0 ){           // ServoB:OFF
              if(lPW.nowDeg == lPW.offDeg){   // 最終値まで行っていたら抜ける
                state = ST_IDLE;
                return;
              }
              SVattach(ch);
              nextDeg = lPW.offDeg;
              nextDelta = lPW.offDelta;
            } else if(gState_Function != 0 ){    // ServoB:ON
              if(lPW.nowDeg == lPW.onDeg){    // 最終値まで行っていたら抜ける
                state = ST_IDLE;
                return;
              }
              SVattach(ch);
              nextDeg = lPW.onDeg;
              nextDelta = lPW.onDelta;
            }
                  
            if(lPW.nowDeg - nextDeg < 0){
              updownFlg = UP;
            } else {
              updownFlg = DOWN;
            }
            state = ST_RUN;
      
            break;

    case ST_RUN:  //3
                  servoABwite(ch,(int)lPW.nowDeg);
                  lPW.nowDeg = lPW.nowDeg + nextDelta;

                  if( ((updownFlg == DOWN) && (lPW.nowDeg <= nextDeg)) || ((updownFlg == UP) && (lPW.nowDeg >= nextDeg)) ) {       // 下りONまで行った？ or 上りONまで行った？
                    lPW.nowDeg = nextDeg;
                    servoABwite(ch,(int)nextDeg);
                    writeCV();
                    SVdetach(ch);
                    led();
                    state = ST_IDLE;
                  }
                  break;
      default:
            break;
  }   
}

void ServoDriver::gState( void )
{
  switch( servoAddress ){
    case 0:
            gState_Function = gState_F0;
            break;
    case 1:
            gState_Function = gState_F1;
            break;
    case 2:
            gState_Function = gState_F2;
            break;
    case 3:
            gState_Function = gState_F3;
            break;
    case 4:
            gState_Function = gState_F4;
            break;
    case 5:
            gState_Function = gState_F5;
            break;
    case 6:
            gState_Function = gState_F6;
            break;
    case 7:
            gState_Function = gState_F7;
            break;
    case 8:
            gState_Function = gState_F8;
            break;
    case 9:
            gState_Function = gState_F9;
            break;
    case 10:
            gState_Function = gState_F10;
            break;
    case 11:
            gState_Function = gState_F11;
            break;
    case 12:
            gState_Function = gState_F12;
            break;

    default:
            break;                    
  }
}
