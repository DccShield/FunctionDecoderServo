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
ServoDriver::ServoDriver(struct ServoParameter sp)
{
  lSP = sp;

  pinMode(lSP.port, OUTPUT);
  digitalWrite(lSP.port, HIGH);

  lPW.onDeg = map((float)lSP.onDeg,lSP.offDeg,lSP.onDeg,lSP.MinAngle,lSP.MaxAngle);    // ON時の角度
  lPW.offDeg = map((float)lSP.offDeg,lSP.offDeg,lSP.onDeg,lSP.MinAngle,lSP.MaxAngle);   // OFF時の角度
  lPW.onDelta = mapfloat(((float)(lSP.onDeg - lSP.offDeg) / lSP.onSpeed / 100),(float)lSP.offDeg,(float)lSP.onDeg,0,(float)(lSP.MaxAngle-lSP.MinAngle)); // offDegからonDegまでの10ms時の移動量を算出
  lPW.offDelta = mapfloat(((float)(lSP.offDeg - lSP.onDeg) / lSP.offSpeed / 100),(float)lSP.offDeg,(float)lSP.onDeg,0,(float)(lSP.MaxAngle-lSP.MinAngle)); // offDegからonDegまでの10ms時の移動量を算出
  lPW.nowDeg = 0;

  adr = lSP.servoAddress;

  if(lSP.ch == 0){
    ServoA.attach(lSP.port, lSP.MinAngle, lSP.MaxAngle);
    delay(20);
    ServoA.detach();
  } else if(lSP.ch == 1){
    ServoB.attach(lSP.port, lSP.MinAngle, lSP.MaxAngle);
    delay(20);
    ServoB.detach();
  }
  state = ST_STANDABY;
}

int ServoDriver::nowState()
{
  return state; 
}


void ServoDriver::writeCV(char ch)
{
  if(ch == 0){
    Dcc.setCV(52,gState_Function);        // 最終値のアクセサリ番号をCV_sdirに書き込み
  }
  if(ch == 1){
    Dcc.setCV(59,gState_Function);        // 最終値のアクセサリ番号をCV_sdirに書き込み
  }
}

void ServoDriver::SVattach(char ch)
{
  if(ch == 0){
    ServoA.attach(lSP.port, lSP.MinAngle, lSP.MaxAngle);
  }
  if(ch == 1){
    ServoB.attach(lSP.port, lSP.MinAngle, lSP.MaxAngle);
  }
}

void ServoDriver::servoABwite(char ch, int ref)
{
  if(ch == 0){
    ServoA.writeMicroseconds(ref);
  }else if(ch == 1){
    ServoB.writeMicroseconds(ref);
  }
}

void ServoDriver::SVdetach(char ch)
{
  if(ch == 0){
    ServoA.detach();
  }
  if(ch == 1){
    ServoB.detach();
  }
}

void ServoDriver::led(unsigned char ch)
{
  if(ch == 0){
    if(gState_Function == 0){ //0:div/ 1:str|
      digitalWrite(PIN_LED_STR1,OFF);
      digitalWrite(PIN_LED_DIV1,ON);
    } else {
      digitalWrite(PIN_LED_STR1,ON);
      digitalWrite(PIN_LED_DIV1,OFF);
    }
  }
  if(ch == 1){
    if(gState_Function == 0){
      digitalWrite(PIN_LED_STR2,OFF);
      digitalWrite(PIN_LED_DIV2,ON);
    } else {
      digitalWrite(PIN_LED_STR2,ON);
      digitalWrite(PIN_LED_DIV2,OFF);
    }
  }
}

// ServoSequence ステートマシン（状態遷移）
void ServoDriver::stateCheck()
{
  gState();    
        
  switch(state){
      case ST_STANDABY:               // 起動時一回だけの処理
        led(lSP.ch);
        if(gState_Function == lSP.sdir ){ // 前回最後のSTR/DIVが同じ？
          if(gState_Function == 0){   // OFF?
            SVattach(lSP.ch);
            lPW.nowDeg = lPW.offDeg;
            servoABwite(lSP.ch,(int)lPW.nowDeg);
          } else {                    // ON?
            SVattach(lSP.ch);
            lPW.nowDeg = lPW.onDeg;
            servoABwite(lSP.ch,(int)lPW.nowDeg);
          }
          SVdetach(lSP.ch);
          state = ST_IDLE;
          break;
        } else { // EEPROMの状態とコマンドステーションが異なっていた時の初回処理
          if(lSP.sdir != 0 and gState_Function == 0){
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
              SVattach(lSP.ch);
              nextDeg = lPW.offDeg;
              nextDelta = lPW.offDelta;
            } else if(gState_Function != 0 ){    // ServoB:ON
              if(lPW.nowDeg == lPW.onDeg){    // 最終値まで行っていたら抜ける
                state = ST_IDLE;
                return;
              }
              SVattach(lSP.ch);
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
                  servoABwite(lSP.ch,(int)lPW.nowDeg);
                  lPW.nowDeg = lPW.nowDeg + nextDelta;

                  if( ((updownFlg == DOWN) && (lPW.nowDeg <= nextDeg)) || ((updownFlg == UP) && (lPW.nowDeg >= nextDeg)) ) {       // 下りONまで行った？ or 上りONまで行った？
                    lPW.nowDeg = nextDeg;
                    servoABwite(lSP.ch,(int)nextDeg);
                    writeCV(lSP.ch);
                    SVdetach(lSP.ch);
                    led(lSP.ch);
                    state = ST_IDLE;
                  }
                  break;
      default:
            break;
  }   
}

void ServoDriver::gState( void )
{
  switch( adr ){
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
