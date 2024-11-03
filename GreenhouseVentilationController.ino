/////////////////////////////////////////////////////////////////
// グリーンンハウス　フィルム巻上げ換気装置制御装置用 Arduinoスケッチ
// Arduino sketch for greenhouse ventilation controller
// https://github.com/citriena/GreenhouseVentilationController
// V1.1.0
// Copyright (C) 2024 citriena
////////////////////////////////////////////////////////////////
// 
// 1.1.0: 2024年11月02日
// ・手動停止モード時は電源入れ直してもリセット動作しないように修正
// ・日付が変わった時にリセット動作するのを止めた。
// 
////////////////////////////////////////////////////////////////
//                  コンパイル設定
////////////////////////////////////////////////////////////////
#define HEATER                  // 加温機を使う場合
#define ELECTRICITY_LIMIT       // 加温機が電力制限いっぱいの場合はモーター動作中に加温機を動かさないようにする。
#define RTC_SD                  // RTCやSDを使う場合；将来的には時間帯で温度設定を変更出来るようにしたい。
#define SdFatLite               // 標準のSDではなくSdFatライブラリを使う場合
//#define SERIAL_MONITOR          // シリアルモニタ使用（デバッグ用）
//#define ETHERNET_SHIELD       // Ethernetシールドを使う場合に設定（未実装）
#define PELTIER_R100            // 使用するシールド基板を設定

///////////////////////////////////////////////////////////
//               外部ライブラリ読みこみ
////////////////////////////////////////////////////////////////
#include <Arduino.h>
#include <EEPROM.h>
#include <LiquidTWI2.h>         // https://github.com/lincomatic/LiquidTWI2
#include <simpleKeypad_I2C.h>   // https://github.com/citriena/simpleKeypad_I2C
#ifdef RTC_SD
#include <SPI.h>
#ifdef SdFatLite                // 標準のSDライブラリに比べてSdFatライブラリではスケッチ容量が大幅に小さくなる。
#include <SdFat.h>              // https://github.com/greiman/SdFat
#else                           // SdFatConfig.h内の[#define USE_LONG_FILE_NAMES 0]と[#define SDFAT_FILE_TYPE 1]は有効にしている。
#include <SD.h>
#endif
#include <TimeLib.h>            // https://github.com/PaulStoffregen/Time
#include <DS3232RTC.h>          // https://github.com/JChristensen/DS3232RTC
#endif
#include <SHthermistor.h>       // https://github.com/citriena/SHthermistor
#include <CytronMotorDriver.h>  // https://github.com/CytronTechnologies/CytronMotorDriver

///////////////////////////////////////////////////////////
//               SDカードSPI SSピン設定
///////////////////////////////////////////////////////////
#ifdef ETHERNET_SHIELD
const int chipSelect =  4;
#else
const int chipSelect = 10;
#endif
///////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////
//            Arduino ハードウェア設定
///////////////////////////////////////////////////////////
#if defined(PELTIER_R100) || defined(PELTIER_R110)

#define THERMISTOR_EXC     2 // サーミスタ電圧デジタルピン（測定時だけ電圧をかけるため、Vccにはつながない（発熱抑制）
#define THERMISTOR_ADC    A2 // サーミスタ入力アナログピン；サーミスタ電圧ピンとの間に分圧抵抗、GNDとの間にサーミスタ接続
#ifdef HEATER
#define HEATER_PIN         5 // 加温ヒーター動作ピン 出力制御をする場合は、3, 5, 6, 9のどれか
#define HEATER_LIMIT_PIN   3 // 加温出力制限時入力ピン
#endif
#define PWM_PIN            6 // Motor Driver PWM Output
#define DIR_PIN            7 // Motor Driver DIR Output
#define RAIN_SENSOR_PIN    8 // 降雨センサピン　雨検出でLOW
#endif

#define PARAM_RSRV_ADDR    0 // 設定値のバックアップデータ保存用EEPROMアドレス
#define PARAM_RSRV_MARK 0x48 // 設定値保存済みマーク（0xFF以外） 

///////////////////////////////////////////////////////////
//  Constructor        使用外部ライブラリのコンストラクタ処理
///////////////////////////////////////////////////////////

// Configure the motor driver.
CytronMD motor(PWM_DIR, PWM_PIN, DIR_PIN);  // Cytron SHIELD-MD10 制御

// サーミスタによる温度測定用ライブラリのコンストラクタ
// 使用するサーミスタ、および使用条件に応じた値を以下に設定する。詳細はSHthermistorの説明参照
// 以下の数値は秋月電子通商で扱っているSEMITEC株式会社103AT-11の場合
SHthermistor thermistor(0, 25, 50, 27280, 10000, 4160, 10000, THERMISTOR_ADC, NTC_GND, THERMISTOR_EXC, 0.0);
// 他の温度では -20C:67770, -10C: 42470, 10C: 17960, 60: 3020, 70:3464, 80: 2468

LiquidTWI2 lcd(0x20);           // LCDのI2Cアドレス設定
simpleKeypad keypad(200, 800);  //キーのリピート間隔、リピート開始時間(msec)

#ifdef RTC_SD
#ifdef SdFatLite
SdFat SD;
#endif

DS3232RTC myRTC;
#endif
///////////////////////////////////////////////////////////
//                     マクロ定義
///////////////////////////////////////////////////////////

#define TIME_LIMIT      300000 // 時計設定制限時間(ms)
#define KEY_TIMER           60 // 何も操作しないと計測画面に戻るまでの秒数
#define MIN_MAX              5 // 最低最高温度を記憶する日数
#define POWER_RATE          70 // 加熱出力制限時の比率％
#define HYSTERESIS           0 // ヒーター動作時の応差温度（ヒステリシス）


///////////////////////////////////////////////////////////
//                     制御初期値
///////////////////////////////////////////////////////////
#define SET_TEMP          25  // 設定温度（℃）
#define SET_SENS          10  // 制御感度（℃x10）10なら設定温度±1℃に維持しようとする。
#define SET_HEAT_TEMP      5  // 加温開始温度（℃）
#define SET_MAX_WIND     150  // 最大累積巻上時間（秒）
#define SET_WIND_TIME     10  // 一回の巻上時間（秒）
#define SET_PAUSE_TIME    30  // 休止時間（秒）
#define SET_EXTRA_REWIND  20  // 確実に閉めるための追加巻下時間（秒）；巻上が裾から下がりきるまでの時間とする。
#define SET_DEHUMID_TIME   5  // 除湿時間（分）
#define SET_LOG_MODE       0  // ログモード（0は記録しない）

////////////////////////////////////////////////////////////////
//                       型宣言
/////////////////////////////////////////////////////////////////

typedef enum {// LCD表示画面
  MANU_MODE,  // 動作モード設定
  TEMP_MODE,  // 温度設定
#ifdef HEATER
  HEAT_MODE,  // 加温開始温度設定
#endif
  SENS_MODE,  // 感度設定
  LENG_MODE,  // 累積巻上時間上限設定
  WIND_MODE,  // 一回の巻上時間設定
  INTV_MODE,  // 休止時間設定　
  REWD_MODE,  // 確実に閉じるように巻下追加時間（秒）設定
  DEHM_MODE,  // 除湿（全開）時間の設定
  OFS_MODE,   // 温度オフセット設定設定
#ifdef RTC_SD
  LOG_MODE,   // microSDにデータ記録設定
  CLK_MODE,   // 時計表示等
#endif
//  RST_MODE,   // リセット（書き込んだ設定を無効化し、初期状態に戻す）
  MES_MODE,   // 計測値表示（基本）
  END_MODE,   // 最後の画面を過ぎたことの判定用
#ifdef RTC_SD
  MIX_MODE    // 最高最低温度画面　表示手順が違うので、END_MODEの後
#endif
} dispMode_t;


typedef enum {       // 制御モード
  C_AUTO_MODE,       // 自動制御
  C_STOP_MODE,       // 制御停止
  C_CLOSE_MODE,      // 全閉
  C_OPEN_MODE,       // 全開
  C_DEHUMID_MODE,    // 除湿モード（終了したら自動制御）
  C_RESET_MODE       // 一旦全閉して巻上カウンターリセット
} controlMode_t;


typedef enum {       // 開閉動作モード
  W_OPEN_MODE,       // 開動作（自動制御中）
  W_CLOSE_MODE,      // 閉動作（自動制御中）
  W_PAUSE_MODE,      // 開閉休止（自動制御中）
  W_FULL_OPEN_MODE,  // 完全開（手動設定）
  W_FULL_CLOSE_MODE, // 完全閉（手動設定）
  W_DEHUMID_MODE,    // 除湿時間（手動設定　その後自動モード）
  W_RESET_MODE,      // 電源投入後等一旦全閉して巻上カウンタをリセットする。
  W_BREAK_MODE,      // 現在の自動制御動作を中止して休止モードにする。
  W_STOP_MODE        // 開閉動作停止
} windingMode_t;


typedef enum {       // 側窓モーター動作モード
  MOTOR_STOP_MODE,   // モーター停止
  UPWINDING_MODE,    // 巻上動作
  DOWNWINDING_MODE,  // 巻下動作
  UPPER_LIMIT_MODE,  // 巻上限界
  LOWER_LIMIT_MODE   // 巻下限界
} motorMode_t;


typedef enum {       // 加熱器の動作状態
  POWER_OFF,         // 出力ゼロ
  POWER_LOW,        // 出力制限（現在は半分）
  POWER_HIGH         // 出力最大
} heaterPower_t;


typedef struct {  // 設定変更時用
  int pValue;     // 設定値
  int pMax;       // 最大設定値
  int pMin;       // 最小設定値
  int pInc;       // 設定値変更幅
} ctlParam_t;


typedef struct {
  byte Month;
  byte Day;
  byte minHour;
  byte minMinute;
  byte maxHour;
  byte maxMinute;
  int minTenFold;
  int maxTenFold;
} minMax_t;


typedef struct {
  byte Year;
  byte Month;
  byte Day;
  byte Hour;
  byte Minute;
  int  TempTenFold;     // 温度 x 10
} logData_t;            // 全体で7バイト

#define BUFF_NO     4   // 7 * 4 = 28（書き込みバッファの30バイト以内にする。）

////////////////////////////////////////////////////////////////
//                       広域変数宣言
/////////////////////////////////////////////////////////////////

controlMode_t gControlMode = C_AUTO_MODE;           // 制御モード；モード変更はこれを使う
motorMode_t gMotorMode = MOTOR_STOP_MODE;           // モーター動作モード；プログラムで自動処理するので人為的には変更しない。

bool gEditing = false;                              // 設定変更時に使用
int gTotalUpWindingTime = 0;                        // 累計巻上時間（上限設定のため）
int gUpWindingTimeLimit = SET_MAX_WIND;             // 累計巻上時間の上限
int gAdditionalDownWindingTime = SET_EXTRA_REWIND;  // 確実に締めるための追加巻下時間
int gPauseTime = SET_PAUSE_TIME;                    // 自動制御時の一時停止時間
int gWindingTime = SET_WIND_TIME;                   // 自動制御時の1回の巻上巻下時間
int gDehumidTime = SET_DEHUMID_TIME;                //

int gSetPoint = SET_TEMP;                 // 制御設定温度
int gSensitivity = SET_SENS;              // 制御感度

#ifdef HEATER
int gHeatTemp = SET_HEAT_TEMP;            // 加温開始温度
heaterPower_t gHeaterPower = POWER_OFF;   // 加温の状態
#endif

float gActualTemp;                        // 実測温度
int gOffsetTemp = 0;                      // 温度オフセット

dispMode_t gDispMode = MES_MODE;          // 画面モード番号。最初は動作状況画面

#ifdef RTC_SD
byte gLogMode = 0;                        // 0:記録無, 1~4:記録間隔（gLogIntの要素番号）
const byte gLogInt[5] = {0, 1, 2, 5, 10}; // 記録間隔（分）
bool gIsSD = false;                       // SDカードが正常に設定完了したらtrue
logData_t gLogData[BUFF_NO];              // 記録データのバッファ。1データ7バイトなので、書き込みバッファ30バイト以内にするためデータ4回分
minMax_t gMinMax[MIN_MAX];                // 最低最高気温
byte gMinMaxPt[MIN_MAX];                  // 0:当日, 1前日のgMinMax要素番号; 99はデータなし
#endif

// パラメータ数の設定
#ifdef RTC_SD
const int PARAM_NUM = LOG_MODE + 1;
#else 
const int PARAM_NUM = OFS_MODE + 1;
#endif

ctlParam_t gCtlParam[PARAM_NUM] = {  // dispMode_tと同じ番号；最大設定値、最小設定値、設定値変更幅
  {0,  5,  0,  1},  // 0 0:自動制御, 1:全開, 2:全閉, 3:手動停止, 4:巻上リセット, 5:除湿
  {0, 40,  0,  1},  // 1 制御温度（℃）
#ifdef HEATER
  {0, 30,  0,  1},  //  加温温度（℃）
#endif
  {0, 50,  0,  1},  //  感度（℃x10）
  {0,600,  0,  1},  //  累積巻上時間上限（秒）
  {0,120,  1,  1},  //  １回の巻上時間（秒）
  {0,120,  0,  1},  //  休止時間（秒）
  {0,120,  0,  1},  //  巻下追加時間（秒）
  {0, 60,  0,  1},  //  除湿時間（分）
  {0, 50,-50,  1},  //  温度補正値 使うときは /10
#ifdef RTC_SD
  {0,  4,  0,  1}   //  ログ記録間隔 {0, 1, 2, 5, 10} 分
#endif
};


///////////////////////////////////////////////////////////
//                  オブジェクト
///////////////////////////////////////////////////////////
#ifdef RTC_SD
// the logging file
#ifdef SdFatLite
SdFile logfile;
#else
File logfile;            // SDカード記録に使用するファイルオブジェクト
#endif
#endif

///////////////////////////////////////////////////////////
//                  メインコード
///////////////////////////////////////////////////////////
void setup() {
#ifdef SERIAL_MONITOR
  Serial.begin(9600);                // シリアル通信初期化
#endif
  lcd.setMCPType(LTI_TYPE_MCP23017); // I2C LCDキーパッドシールドの設定（IOエキスパンダーの指定）
  lcd.begin(16, 2);
  if (keypad.read_buttons() == btnNONE) {  // 何もキーが押されていなかったら設定読みこみ
    readParam();
  } else {
    EEPROM.write(PARAM_RSRV_ADDR, 0xFF);   // 設定無効化
    lcd.setCursor(0, 0);
    lcd.print(F("RESET COMPLETED"));
    do {
    } while (keypad.read_buttons() != btnNONE);
  }
  lcd.clear();

#ifdef RTC_SD
  initMinMax();
  // 完全巻下げは動作モードを最初リセットモードにしているのでここでは不要
#endif
#ifdef HEATER
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(HEATER_LIMIT_PIN, INPUT_PULLUP);
  pinMode(RAIN_SENSOR_PIN, INPUT_PULLUP);
#endif

}


void loop() {
  static char keyTimer = KEY_TIMER;       // 一定時間キーを操作しなかったらLCD表示を計測画面にするためのタイマー変数
  unsigned long now = millis();
  unsigned long sec = now / 1000;
  static unsigned long secPrev;

  if (sec != secPrev) {
    secPrev = sec;
    gActualTemp = thermistor.readTemp() + (float)gOffsetTemp / 10;  // 現在温度取得
    windowControl(false);
#ifdef HEATER
    heatControl();
#endif
    if (keyTimer > 0) {
      keyTimer--;
    } else {
      lcd.clear();
      gDispMode = MES_MODE;       // 測定画面に戻る.
    }
    if (gDispMode == MES_MODE)  {
      keyTimer = KEY_TIMER;       // 時間がたつとkeyTimerが0になって画面がちらつくのを防止
      dispMainMode();
    }
#ifdef RTC_SD
    dataLogging();
#endif
  }
  if (read_keypad()) keyTimer = KEY_TIMER;
}

#ifdef RTC_SD

void dataLogging() {
  tmElements_t tm;
  static byte preMinute = 99;
  static int sumData = 0;
  static int sumNo = 0;
  static byte currentDay = 0;
  int tempTenFold;

  myRTC.read(tm);
  if (tm.Minute != preMinute) {
    preMinute = tm.Minute;
    tempTenFold = (int)(gActualTemp * 10 + 0.5);    // intに変換するときに小数点以下は切り下げされるので、四捨五入になるように0.5加える
    minMax(tm, tempTenFold);
    sumData += tempTenFold;
    sumNo++;
    if (isLogTime(tm)) {
      writeLog(tm, sumData / sumNo, false);         // 50℃時にログ間隔60分でもオーバーフローしない。
      sumData = 0;
      sumNo = 0;
    }
  }

/*
  if ((currentDay != tm.Day) && (gControlMode == C_AUTO_MODE)) {  // 自動モードでは日が変わるときにリセットし、側窓が確実に閉まるようにする。
    gControlMode = C_RESET_MODE;
    currentDay = tm.Day;
  }
*/
}    

void minMax(tmElements_t tm, int tempTenFold) {
  static byte prevDay = 0;

  if (prevDay != tm.Day) {
    prevDay = tm.Day;
    shiftMinMax();
    gMinMax[gMinMaxPt[0]].Month = tm.Month;
    gMinMax[gMinMaxPt[0]].Day = tm.Day;
  }
  if (tempTenFold > gMinMax[gMinMaxPt[0]].maxTenFold) {
    gMinMax[gMinMaxPt[0]].maxTenFold = tempTenFold;
    gMinMax[gMinMaxPt[0]].maxHour = tm.Hour;
    gMinMax[gMinMaxPt[0]].maxMinute = tm.Minute;
  }
  if (tempTenFold < gMinMax[gMinMaxPt[0]].minTenFold) {
    gMinMax[gMinMaxPt[0]].minTenFold = tempTenFold;
    gMinMax[gMinMaxPt[0]].minHour = tm.Hour;
    gMinMax[gMinMaxPt[0]].minMinute = tm.Minute;
  }
}


void shiftMinMax() {  // ５日前のデータ領域を今日のデータ領域にし、その他は１日ずらす。
  byte newMinMaxPt = gMinMaxPt[MIN_MAX - 1];  // 最後のデータ領域を記録しておき、新しいデータ用に使う。
  byte i;

  for (i = MIN_MAX - 1; i > 0; i--) {
    gMinMaxPt[i] = gMinMaxPt[i - 1];          // データを1日ずらす。
  }
  gMinMaxPt[0] = newMinMaxPt;                 // 最後日のデータだった領域を今日のデータ領域にする。
  gMinMax[newMinMaxPt].minTenFold =  999;     // x10なので 99.9°
  gMinMax[newMinMaxPt].maxTenFold = -999;     // x10なので-99.9°
  gMinMax[newMinMaxPt].Day = 0;
}


void initMinMax() {
  for (byte i = 0; i < MIN_MAX; i++) {
    gMinMaxPt[i] = i;     // 最初は番号通り
    gMinMax[i].Day = 0;   // 0はデータ無しを示す。
  }
//  shiftMinMax();              // データリセット（手抜き）
}

#endif

///////////////////////////////////////////////////////////
//                   窓開閉の制御コード
///////////////////////////////////////////////////////////

void windowControl(bool countReset) {               // 一秒毎に呼び出される。
  static int countTime = 0;                         // 自動制御時の巻動作、休止時間（秒）
  static bool inResetMode = false;                  // 巻上カウンタリセット中のフラグ
  static bool resetFinished = false;                // 巻上カウンタリセット済みのフラグ
  static windingMode_t windingMode = W_STOP_MODE;   // 窓開閉動作モード
  static bool powerOn = true;
  static int dehumidTime;                           // 除湿累計時間
  static int prevTemp[2] = {-1000, -1000};
  int diffTemp;                                     // 設定温度に近づいていたらマイナス、離れていたらプラス

  if (powerOn) {
    switch(gControlMode) {
    case C_STOP_MODE:                               // 停電復旧時に停止モードだったらリセット省略
      resetFinished = true;                         // 巻上カウンタ復旧するのでリセット済とする。
      break;
    case C_OPEN_MODE:                               // 手動開モード時は停電復旧時に巻上カウンタリセットされるので、開度が足りなくなることはない。
      break;
    case C_CLOSE_MODE:
      gTotalUpWindingTime = gUpWindingTimeLimit;    // 手動閉モード時は停電復旧時に巻上カウンタを最大にして確実に閉まるようにする。
      break;
    }
    powerOn = false;
  }

  if (countReset && ((windingMode == W_OPEN_MODE) || (windingMode == W_CLOSE_MODE) || (windingMode == W_PAUSE_MODE))) { // カウンターリセットするのは自動制御時のみ
    countTime = 0;                                  // カウンターリセット時は強制的にカウンターをゼロにする。
    windingMode = W_PAUSE_MODE;                     
    gMotorMode = MOTOR_STOP_MODE;
    return;
  }

  if (countTime > 0) countTime--;                   // 毎秒毎に動作カウンターを減らす

  switch(gControlMode) {                            // 手動で設定した制御モードに応じて開閉動作モードを選択
  case C_AUTO_MODE:
    if ((windingMode != W_OPEN_MODE) && (windingMode != W_CLOSE_MODE) && (windingMode != W_PAUSE_MODE)) { // 既に自動制御中は変更しない。
      windingMode = W_PAUSE_MODE;                   // 新たに自動制御に入ったときは休止モード終了からスタート
      countTime = 0;                                // 
    }
    if (digitalRead(RAIN_SENSOR_PIN) == LOW) {      // 降雨検出時は閉める。
      windingMode = W_FULL_CLOSE_MODE;
    }
    if (!resetFinished) {                           // リセット未実施ならリセット動作
      windingMode = W_RESET_MODE;
    }
    break;
  case C_OPEN_MODE:
    windingMode = W_FULL_OPEN_MODE;
    break;
  case C_CLOSE_MODE:
    windingMode = W_FULL_CLOSE_MODE;
    break;
  case C_STOP_MODE:
    windingMode = W_STOP_MODE;
    break;
  case C_RESET_MODE:
    windingMode = W_RESET_MODE;
    break;
  case C_DEHUMID_MODE:
    if (windingMode != W_DEHUMID_MODE) {    // 　除湿モードに入る際は除湿時間リセット
      windingMode = W_DEHUMID_MODE;
      dehumidTime = 0;
    }
    break;
  default:
    windingMode = W_STOP_MODE;
  }

  switch(windingMode) {                     // 各動作モードでの開閉動作の設定
  case W_OPEN_MODE:
  case W_CLOSE_MODE:
    if (countTime == 0) {
      windingMode = W_PAUSE_MODE;
      countTime = gPauseTime;
    }
    break;
  case W_PAUSE_MODE:                                           // 自動運転中の一時休止
    if (countTime == 0) {                                      // 休止モード終了なら
      countTime = gPauseTime;                                  // どれにも該当しなかったら休止モード継続のため、最初に設定しておく
      prevTemp[1] = prevTemp[0];
      prevTemp[0] = (gActualTemp + 0.05) * 10;                 // 実数から整数変換時に小数点が切り捨てられるので、+0.05して四捨五入にする
      if (prevTemp[1] == -1000) prevTemp[1] = prevTemp[0];     // 初回は温度変化無しで処理
      if (prevTemp[0] >= (gSetPoint * 10)) {                   // 設定温度よりも高かったら
        diffTemp = prevTemp[0] - prevTemp[1];                  // 設定温度から離れるとプラス、近づくとマイナスになるようにする。
      } else {
        diffTemp = prevTemp[1] - prevTemp[0];                  // 設定温度から離れるとプラス、近づくとマイナスになるようにする。
      }
      if ((prevTemp[0] + gSensitivity) < (gSetPoint * 10)) {   // 設定温度よりも低い場合
        if (diffTemp >= 0) {                                   // 変動無、もしくは設定温度から離れている場合
          windingMode = W_CLOSE_MODE;                          // 側窓を閉める。
          countTime = gWindingTime;                            // カウンタを巻き下時間に設定
          if (diffTemp > 5) {                                  // 前回測定よりも0.5℃以上離れつつある場合は
            countTime *= 2;                                    // 巻下時間を倍にする。
          }
        }
      } else if ((prevTemp[0] - gSensitivity) > (gSetPoint * 10)) {  //設定温度より高い場合
        if (diffTemp >= 0) {                                   // 変動無 、もしくは設定温度から離れている場合
          windingMode = W_OPEN_MODE;                           // 側窓を開ける。
          countTime = gWindingTime;                            // カウンタを巻き上げ時間に設定
          if (diffTemp > 5) {                                  // 前回測定よりも0.5℃以上離れている場合は
            countTime *= 2;                                    // 巻上時間を倍にする。
          }
        }
        if (gTotalUpWindingTime < gAdditionalDownWindingTime) {            // 側窓が閉じている場合は、
          countTime += (gAdditionalDownWindingTime - gTotalUpWindingTime); // 開くところまで時間追加
        }
      }
    }
    break;
  case W_BREAK_MODE:                              // 自動処理中に強制的に休止モードにする。今のところ使っていない。
    windingMode = W_PAUSE_MODE;
    countTime = 0;
    break;
  case W_STOP_MODE:
//    windingMode = W_STOP_MODE;
    countTime = 0;
    break;
  case W_RESET_MODE:
    if (!inResetMode) {                           // 既にリセットモードに入っているかどうかの判定
      gTotalUpWindingTime = gUpWindingTimeLimit;  // 入っていなかったら巻上カウンタを最大にする。
      gMotorMode = MOTOR_STOP_MODE;               // 下限に達していると強制的にリセットできないので停止モードにする。
      inResetMode = true;
      resetFinished = false;
    } 
    if (gMotorMode == LOWER_LIMIT_MODE) {         // リセット完了
      windingMode = W_STOP_MODE;                  // リセットが終わったら停止モードにする。これで設定した制御モードに移行できるようになる。
      if (gControlMode == C_RESET_MODE) {         // 手動リセットモードのままなら自動制御モードに移行
        gControlMode = C_AUTO_MODE;
        writeParam();                             // 停電時は自動制御モードで復旧するようにしておく。
      }
      inResetMode = false;
      resetFinished = true;
      countTime = 0;
    }
    break;
  case W_DEHUMID_MODE:
    if (gMotorMode == UPPER_LIMIT_MODE) {         // 全開だったら
      dehumidTime++;                              // 除湿累計時間加算
      if ((gDehumidTime * 60) <= dehumidTime) {   // 除湿時間終了なら；gDehumidTimeは分、dehumidTimeは秒なので秒に合わせる。
        gControlMode = C_AUTO_MODE;               // 自動制御モードに移行
        windingMode = W_PAUSE_MODE;
        countTime = 0;
        writeParam();                             // 停電時に自動制御モードで復旧するようにしておく。
      }
    }
    break;
  }

  switch(windingMode) {                           // 選択した開閉モードによりモーターの動作モードを設定
  case W_FULL_OPEN_MODE:
  case W_OPEN_MODE:
  case W_DEHUMID_MODE:
    if (gMotorMode != UPPER_LIMIT_MODE) {
      gMotorMode = UPWINDING_MODE;
    }
    break;
  case W_FULL_CLOSE_MODE:
  case W_CLOSE_MODE:
  case W_RESET_MODE:
    if (gMotorMode != LOWER_LIMIT_MODE) {
      gMotorMode = DOWNWINDING_MODE;
    }
    break;
  case W_PAUSE_MODE:
  case W_BREAK_MODE:              // これは実行されない。
  case W_STOP_MODE:
    if ((gMotorMode != UPPER_LIMIT_MODE) && (gMotorMode != LOWER_LIMIT_MODE)) {
      gMotorMode = MOTOR_STOP_MODE;
    }
    break;
  }

  switch(gMotorMode) {             // モーターの巻上位置を確認；上限，下限かどうか確認し、巻上カウンタを更新；停止、再起動後のカウンタがずれるので処理場所を変更
  case UPWINDING_MODE:
    if (gTotalUpWindingTime < 0) gTotalUpWindingTime = 0;
    gTotalUpWindingTime++;
    if (gTotalUpWindingTime >= gUpWindingTimeLimit) {
      gMotorMode = UPPER_LIMIT_MODE;
    }
    break;
  case DOWNWINDING_MODE:
    gTotalUpWindingTime--;
    if (gTotalUpWindingTime <= -gAdditionalDownWindingTime) {
      gTotalUpWindingTime = 0;
      gMotorMode = LOWER_LIMIT_MODE;
    }
    break;
  }

  int motorValue;
  switch(gMotorMode) {          // 設定したモーターモードに従い，モーターを動作
  case UPWINDING_MODE:
    motorValue = 255;
    break;
  case DOWNWINDING_MODE:
    motorValue = -255;
    break;
  default:
    motorValue = 0;
  }
#ifdef ELECTRICITY_LIMIT
  if (motorValue != 0) {        // モーター動作時は電力制限のため加温停止
    digitalWrite(HEATER_PIN, LOW);
    gHeaterPower = POWER_OFF;
  }
#endif
  motor.setSpeed(motorValue);   // モータードライバの出力設定; リレーを使う場合はここを変更
}


///////////////////////////////////////////////////////////
//              加温器の制御コード（単なるON/OFF制御）
///////////////////////////////////////////////////////////

#ifdef HEATER

void heatControl() {
  static bool tHeaterOn = false;
  static heaterPower_t tHeaterPower = POWER_OFF;
  int lowLimit;
#ifdef RTC_SD
  tmElements_t tm;
#endif

  if (gHeaterPower != POWER_OFF) {
    lowLimit = gHeatTemp + HYSTERESIS;
  } else {
    lowLimit = gHeatTemp;
  }

  if ((gControlMode == C_AUTO_MODE) && ((int)gActualTemp < lowLimit)) {
    if (digitalRead(HEATER_LIMIT_PIN) == HIGH) {
      gHeaterPower = POWER_HIGH;
    } else {
      gHeaterPower = POWER_LOW;
    }
  } else {
    gHeaterPower = POWER_OFF;
  }

#ifdef ELECTRICITY_LIMIT
  if ((gMotorMode == UPWINDING_MODE) || (gMotorMode == DOWNWINDING_MODE)) { // 側窓モーターが動作中は電力制限のため加温停止
    gHeaterPower = POWER_OFF;
  }
#endif

  switch (gHeaterPower) {
    case POWER_HIGH: {
      digitalWrite(HEATER_PIN, HIGH);
      break;
    }
    case POWER_LOW: {
      analogWrite(HEATER_PIN, 255 * POWER_RATE / 100);
      break;
    }
    case POWER_OFF: {
      digitalWrite(HEATER_PIN, LOW);
      break;
    }
  } 


#ifdef RTC_SD
  if (gIsSD) {
    if (gHeaterPower != tHeaterPower) {
      myRTC.read(tm);
      writeLog(tm, 0, true);  // SDに未書き込みのデータを書き込む
      logfile.print(tmYearToCalendar(tm.Year));
      logfile.print(F("/"));
      logfile.print(tm.Month);
      logfile.print(F("/"));
      logfile.print(tm.Day);
      logfile.print(F(","));
      logfile.print(tm.Hour);
      logfile.print(F(":"));
      logfile.print(tm.Minute);
      logfile.print(F(","));
      logfile.print(gActualTemp, 1);
      logfile.print(F(","));
      if (gHeaterPower == POWER_HIGH) {
        logfile.println(F("HEATER HIGH"));
      } else if (gHeaterPower == POWER_LOW) {
        logfile.println(F("HEATER LOW"));
      } else {
        logfile.println(F("HEATER OFF"));
      }
    }
  }
  tHeaterPower = gHeaterPower;
#endif
}
#endif

/////////////////////////////////////////////////////////////
//                    キー操作関係
/////////////////////////////////////////////////////////////
boolean read_keypad() {
  btnCODE_t lcd_key;
  static char dayBefore = -1; // -1は最高最低ではなく基本画面

  lcd_key = keypad.read_buttons(); // read the buttons
  if ((lcd_key  == btnVOID) || (lcd_key  == btnNONE)) { // キー入力がない場合はさっさと終了
    return false;
  }
  switch (lcd_key) {                // 押されたキーに応じた共通処理（画面遷移、設定値の変更）
    case btnRIGHT: {
#ifdef RTC_SD
      if (gDispMode == MIX_MODE) {
        gDispMode = MES_MODE;        // 基本画面に戻るだけ
        dayBefore = -1;
        break;
      }
#endif
      gDispMode = (dispMode_t)static_cast<int>(gDispMode + 1);
      if (gDispMode == END_MODE) {
        gDispMode = (dispMode_t)(0);  // 最終画面を過ぎたら最初に戻る。
      }
      gEditing = false;               // 画面を移動したら編集モードをリセット
      break;
    }
    case btnLEFT: {
#ifdef RTC_SD
      if (gDispMode == MIX_MODE) {
        gDispMode = MES_MODE;        // 基本画面に戻るだけ
        dayBefore = -1;
        break;
      }
#endif
      if (gDispMode == 0) {
        gDispMode = (dispMode_t)static_cast<int>(END_MODE - 1);  // 最初を過ぎたら最終画面に移行
      } else {
        gDispMode = (dispMode_t)static_cast<int>(gDispMode - 1);
      }
      gEditing = false;               // 画面を移動したら編集モードをリセット
      break;
    }
    case btnUP: {
      if (gDispMode < PARAM_NUM) { // パラメータ変更画面のgDispMODE番号はctlParamと同じ番号で、gDispMODEのはじめの方にまとめている。
        gCtlParam[MANU_MODE].pValue += gCtlParam[gDispMode].pInc;
        if (gCtlParam[MANU_MODE].pValue > gCtlParam[gDispMode].pMax) {
          gCtlParam[MANU_MODE].pValue = gCtlParam[gDispMode].pMax;
        }
        break;
#ifdef RTC_SD
      } else if ((gDispMode == MES_MODE) || (gDispMode == MIX_MODE)){
        gDispMode = MIX_MODE;
        if (dayBefore < (MIN_MAX - 1)) {
          if (gMinMax[gMinMaxPt[dayBefore + 1]].Day > 0) {
            dayBefore++;
          }
        }
        break;
#endif
      }
      return false;
    }
    case btnDOWN: {
      if (gDispMode < PARAM_NUM) { // パラメータ変更画面のgDispMODE番号はctlParamと同じ番号で、gDispMODEのはじめの方にまとめている。
        gCtlParam[MANU_MODE].pValue -= gCtlParam[gDispMode].pInc;
        if (gCtlParam[MANU_MODE].pValue < gCtlParam[gDispMode].pMin) {
          gCtlParam[MANU_MODE].pValue = gCtlParam[gDispMode].pMin;
        }
        break;
#ifdef RTC_SD
      } else if (gDispMode == MIX_MODE){
        gDispMode = MIX_MODE;
        if (dayBefore >= 0) dayBefore--;
        if  (dayBefore == -1) gDispMode = MES_MODE; 
        break;
#endif
      }
      return false;
    }
    case btnSELECT: { // SELECTの処理は共通化できないので次の画面表示処理内で行う。
      break;
    }
    case btnVOID: {   // 画面表示変更しないのでそのまま戻る。
      return false;
    }
    case btnNONE: {   // 画面表示変更しないのでそのまま戻る。
      return false;
    }
  }


// ここから表示処理とselectキーでの切り替え処理
  lcd.clear();
  lcd.setCursor(0, 0);
  switch (gDispMode) {
    case MANU_MODE:
      dispManualMode(lcd_key);
      break;
    case TEMP_MODE:
      dispTempMode(lcd_key);
      break;
#ifdef HEATER
    case HEAT_MODE:
      dispHeatMode(lcd_key);
      break;
#endif
    case SENS_MODE:
      dispSensitivityMode(lcd_key);
      break;
    case LENG_MODE:
      dispWindLengthMode(lcd_key);
      break;
    case INTV_MODE:
      dispPauseTimeMode(lcd_key);
      break;
    case WIND_MODE:
      dispWindTimeMode(lcd_key);
      break;
    case REWD_MODE:
      dispExtraReWindTimeMode(lcd_key);
      break;
    case DEHM_MODE:
      dispDehumidTimeMode(lcd_key);
      break;
    case OFS_MODE:
      dispOffsetMode(lcd_key);
      break;
#ifdef RTC_SD
    case LOG_MODE:
      dispLogMode(lcd_key);
      break;
    case CLK_MODE:
      dispClockMode(lcd_key);
      break;
    case MIX_MODE:
      dispMinMax(dayBefore);
      break;
#endif
//    case RST_MODE:
//      dispResetMode(lcd_key);
//      break;
    case MES_MODE:
      dayBefore = -1;     // メインモードに戻ったら、最低最高の日付も戻す。
      dispMainMode();
      break;
    case END_MODE:
      break;
  }
  return true;
}


///////////////////////////////////////////////////////////
//                 画面表示コード
///////////////////////////////////////////////////////////

void dispManualMode(btnCODE_t lcd_key) {
  if (!gEditing) {
    gCtlParam[MANU_MODE].pValue = (int)gControlMode;
    gEditing = true;
  }
  lcd.print(F("MODE:"));
  switch(gCtlParam[MANU_MODE].pValue) {
    case C_AUTO_MODE:
      lcd.print(F("AUTO"));
      break;
    case C_STOP_MODE:
      lcd.print(F("STOP"));
      break;
    case C_CLOSE_MODE:
      lcd.print(F("CLOSE"));
      break;
    case C_OPEN_MODE:
      lcd.print(F("OPEN"));
      break;
    case C_DEHUMID_MODE:
      lcd.print(F("DEHUMID"));
      break;
    case C_RESET_MODE:
      lcd.print(F("RESET"));
      break;
  }
  dispSelect();
  if (lcd_key == btnSELECT) {
    gControlMode = (controlMode_t)gCtlParam[MANU_MODE].pValue;
    writeParam();
    gDispMode = MES_MODE;
    lcd.clear();
  }
}
//0123456789ABCDEF
//AUTO
//OPEN
//CLOSE
//STOP
//DEHUMID
//SELECT TO SET

void dispTempMode(btnCODE_t lcd_key) {
  if (!gEditing) {
    gCtlParam[MANU_MODE].pValue = gSetPoint;
    gEditing = true;
  }
  lcd.print(F("SP:  "));
  if (gCtlParam[MANU_MODE].pValue < 10) lcd.print(F(" "));
  lcd.print(gCtlParam[MANU_MODE].pValue);
  lcd.write(0xdf);
  lcd.print(F("C"));
  dispSelect();
  if (lcd_key == btnSELECT) {
    gSetPoint = gCtlParam[MANU_MODE].pValue;
    writeParam();
    gDispMode = MES_MODE;
    windowControl(true);  // カウンターリセット
    lcd.clear();
  }
}
//0123456789ABCDEF
//SP:25C
//SELECT TO SET

void dispSensitivityMode(btnCODE_t lcd_key) {
  if (!gEditing) {
    gCtlParam[MANU_MODE].pValue = gSensitivity;
    gEditing = true;
  }
  lcd.print(F("SENS: "));
  lcd.print((float)gCtlParam[MANU_MODE].pValue / 10, 1);
  lcd.write(0xdf);
  lcd.print(F("C"));
  dispSelect();
  if (lcd_key == btnSELECT) {
    gSensitivity = gCtlParam[MANU_MODE].pValue;
    writeParam();
    gDispMode = MES_MODE;
    windowControl(true);  // カウンターリセット
    lcd.clear();
  }
}
//0123456789ABCDEF
//SENS:1.0ﾟC
//SELECT TO SET

#ifdef HEATER

void dispHeatMode(btnCODE_t lcd_key) {
 if (!gEditing) {
    gCtlParam[MANU_MODE].pValue = gHeatTemp;
    gEditing = true;
  }
  lcd.print(F("HEAT:"));
  if (gCtlParam[MANU_MODE].pValue < 10) lcd.print(F(" "));
  lcd.print(gCtlParam[MANU_MODE].pValue);
  lcd.write(0xdf);
  lcd.print(F("C"));
  dispSelect();
  if (lcd_key == btnSELECT) {
    gHeatTemp = gCtlParam[MANU_MODE].pValue;
    writeParam();
    gDispMode = MES_MODE;
    lcd.clear();
  }
}

#endif

void dispWindLengthMode(btnCODE_t lcd_key) {
  if (!gEditing) {
    gCtlParam[MANU_MODE].pValue = gUpWindingTimeLimit;
    gEditing = true;
  }
  lcd.print(F("MAX WIND T: "));
  if (gCtlParam[MANU_MODE].pValue < 100) lcd.print(F(" "));
  if (gCtlParam[MANU_MODE].pValue <  10) lcd.print(F(" "));
  lcd.print(gCtlParam[MANU_MODE].pValue);
  lcd.print(F("s"));
  dispSelect();
  if (lcd_key == btnSELECT) {
    gUpWindingTimeLimit = gCtlParam[MANU_MODE].pValue;
    writeParam();
    gDispMode = MES_MODE;
    lcd.clear();
  }
}
//0123456789ABCDEF
//MAX WIND T: 600s
//SELECT TO SET

void dispPauseTimeMode(btnCODE_t lcd_key) {
  if (!gEditing) {
    gCtlParam[MANU_MODE].pValue = gPauseTime;
    gEditing = true;
  }
  lcd.print(F("PAUSE TIME:  "));
  if (gCtlParam[MANU_MODE].pValue <  10) lcd.print(F(" "));
  lcd.print(gCtlParam[MANU_MODE].pValue);
  lcd.print(F("s"));
  dispSelect();
  if (lcd_key == btnSELECT) {
    gPauseTime = gCtlParam[MANU_MODE].pValue;
    writeParam();
    gDispMode = MES_MODE;
    windowControl(true);  // カウンターリセット
    lcd.clear();
  }
}
//0123456789ABCDEF
//PAUSE TIME:  60s
//SELECT TO SET

void dispWindTimeMode(btnCODE_t lcd_key) {
  if (!gEditing) {
    gCtlParam[MANU_MODE].pValue = gWindingTime;
    gEditing = true;
  }
  lcd.print(F("WINDING TIME:"));
  if (gCtlParam[MANU_MODE].pValue <  10) lcd.print(F(" "));
  lcd.print(gCtlParam[MANU_MODE].pValue);
  lcd.print(F("s"));
  dispSelect();
  if (lcd_key == btnSELECT) {
    gWindingTime = gCtlParam[MANU_MODE].pValue;
    writeParam();
    gDispMode = MES_MODE;
    windowControl(true);  // カウンターリセット
    lcd.clear();
  }
}
//0123456789ABCDEF
//WINDING TIME:60s
//SELECT TO SET

void dispExtraReWindTimeMode(btnCODE_t lcd_key) {
  if (!gEditing) {
    gCtlParam[MANU_MODE].pValue = gAdditionalDownWindingTime;
    gEditing = true;
  }
  lcd.print(F("EXTRA REWIND:"));
  if (gCtlParam[MANU_MODE].pValue <  10) lcd.print(F(" "));
  lcd.print(gCtlParam[MANU_MODE].pValue);
  lcd.print(F("s"));
  dispSelect();
  if (lcd_key == btnSELECT) {
    gAdditionalDownWindingTime = gCtlParam[MANU_MODE].pValue;
    writeParam();
    gDispMode = MES_MODE;
    lcd.clear();
  }
}
//0123456789ABCDEF
//EXTRA REWD: 060s
//SELECT TO SET

void dispDehumidTimeMode(btnCODE_t lcd_key) {
  if (!gEditing) {
    gCtlParam[MANU_MODE].pValue = gDehumidTime;
    gEditing = true;
  }
  lcd.print(F("DEHUMID TIME:"));
  if (gCtlParam[MANU_MODE].pValue <  10) lcd.print(F(" "));
  lcd.print(gCtlParam[MANU_MODE].pValue);
  lcd.print(F("m"));
  dispSelect();
  if (lcd_key == btnSELECT) {
    gDehumidTime = gCtlParam[MANU_MODE].pValue;
    writeParam();
    gDispMode = MES_MODE;
    lcd.clear();
  }
}
//0123456789ABCDEF
//DEHUMID TIME:60m
//SELECT TO SET


void dispOffsetMode(btnCODE_t lcd_key) {
  if (!gEditing) {
    gCtlParam[MANU_MODE].pValue = gOffsetTemp;
    gEditing = true;
  }
  lcd.print(F("OFFSET TMP:"));
  if (gCtlParam[MANU_MODE].pValue >= 0) lcd.print(F("+"));
  lcd.print((float)gCtlParam[MANU_MODE].pValue / 10, 1);
  lcd.write(0xdf);
//  lcd.print(F("C"));
  dispSelect();
  if (lcd_key == btnSELECT) {
    gOffsetTemp = gCtlParam[MANU_MODE].pValue;
    writeParam();
    gDispMode = MES_MODE;
  }
}
//0123456789ABCDEF
//OFFSET TMP:+0.0C
//SELECT TO SET


#ifdef RTC_SD
void dispLogMode(btnCODE_t lcd_key) {
  tmElements_t tm;

  if (!gEditing) {
    gCtlParam[MANU_MODE].pValue = gLogMode;
    gEditing = true;
  }
  lcd.print(F("LOG MODE: "));
  if (gLogInt[gCtlParam[MANU_MODE].pValue] <  10) lcd.print(F(" "));
  lcd.print(gLogInt[gCtlParam[MANU_MODE].pValue]);
  lcd.print(F("min."));
  dispSelect();
  if (lcd_key == btnSELECT) {
    if (gCtlParam[MANU_MODE].pValue == 0) {
      if (gIsSD) {
        writeLog(tm, 0, true);
        logfile.close();
        gIsSD = false;
      }
    } else {
      if (!gIsSD) {
        if (setupSD()) {
          gIsSD = true;
        } else {
          gCtlParam[MANU_MODE].pValue = 0;
        }
      }
    }
    gLogMode = gCtlParam[MANU_MODE].pValue;
    writeParam();
    gDispMode = MES_MODE;
    lcd.clear();
  }
}
//0123456789ABCDEF
//LOG MODE: 10min.
//SELECT TO SET

void dispClockMode(btnCODE_t lcd_key) {
  tmElements_t tm;
  if (lcd_key == btnSELECT) {
    setRTCwKeypad(); // キーパッドで時計の設定
  } else {
    myRTC.read(tm);
    lcdDayTime(tm, 0, -1);
    lcd.setCursor(0, 1);
    lcd.print(F("SELECT=SET TIME"));
  }
}


void dispMinMax(char dayBefore) {
  lcd.setCursor(0, 0);
  if (gMinMax[gMinMaxPt[dayBefore]].Day == 0) {
    lcd.print(F("NO DATA"));
    return;
  }
  lcdDay(gMinMax[gMinMaxPt[dayBefore]].Month, gMinMax[gMinMaxPt[dayBefore]].Day);
  lcd.print(F(" "));
  lcdTime(gMinMax[gMinMaxPt[dayBefore]].maxHour, gMinMax[gMinMaxPt[dayBefore]].maxMinute);
  lcd.print(F(" "));
  if (gMinMax[gMinMaxPt[dayBefore]].maxTenFold < 100) lcd.print(F(" "));
  lcd.print((float)gMinMax[gMinMaxPt[dayBefore]].maxTenFold / 10, 1);
  lcd.setCursor(0, 1);
  lcd.print(dayBefore + 0x30, 0);
  lcd.setCursor(6, 1);
  lcdTime(gMinMax[gMinMaxPt[dayBefore]].minHour, gMinMax[gMinMaxPt[dayBefore]].minMinute);
  lcd.print(F(" "));
  if ((gMinMax[gMinMaxPt[dayBefore]].minTenFold < 100) && (gMinMax[gMinMaxPt[dayBefore]].minTenFold >= 0)) {
    lcd.print(F(" "));
  }
  lcd.print((float)gMinMax[gMinMaxPt[dayBefore]].minTenFold / 10, 1);
}
//0123456789ABCDEF
//10/25 13:00 25.2
//      06:00 18.5

#endif


void dispMainMode() {
#ifdef HEATER
  static bool heaterLcd = false;
#endif
  lcd.setCursor(0, 0);    // これは他からも呼ばれるので必要
  lcd.print(F("SP:"));
  if (gSetPoint < 10) lcd.print(F(" "));
  lcd.print((float)gSetPoint, 1);
  lcd.write(0xdf);
//  lcd.print(F("C "));
  switch(gControlMode) {
    case C_AUTO_MODE:
      lcd.print(F("AUTO "));
      break;
    case C_OPEN_MODE:
      lcd.print(F("OPEN "));
      break;
    case C_CLOSE_MODE:
      lcd.print(F("CLOSE"));
      break;
    case C_STOP_MODE:
      lcd.print(F("STOP "));
      break;
    case C_RESET_MODE:
      lcd.print(F("RESET"));
      break;
    case C_DEHUMID_MODE:
      lcd.print(F("DEHUM"));
      break;
  }
  char ch;
  switch(gMotorMode) {
    case UPWINDING_MODE:
      ch = '^';
      break;
    case DOWNWINDING_MODE:
      ch = 'v';
      break;
    case UPPER_LIMIT_MODE:
      ch = 0x7e;
//      ch = '>';
      break;
    case LOWER_LIMIT_MODE:
      ch = 0x7f;
//      ch = '<';
      break;
    case MOTOR_STOP_MODE:
      ch = '-';
      break;
  }
  lcd.setCursor(15, 0);
  lcd.print(ch);

  lcd.setCursor(0, 1);
  lcd.print(F("TP:"));
  if (gActualTemp < 10) lcd.print(F(" "));
  lcd.print(gActualTemp, 1);
  lcd.write(0xdf);
//  lcd.print(F("C "));
  if (gTotalUpWindingTime >= 0)  {
    if (gTotalUpWindingTime < 100) lcd.print(F(" "));
    if (gTotalUpWindingTime <  10) lcd.print(F(" "));
  } else {
    if (gTotalUpWindingTime > -10) lcd.print(F(" "));
  }
  lcd.print(gTotalUpWindingTime);
  lcd.print(F("s "));
#ifdef HEATER
  if (gHeaterPower != POWER_OFF) {
    heaterLcd = !heaterLcd;
  } else{
    heaterLcd = false;
  }
  if (heaterLcd) {
    lcd.print(F("  "));
  } else {
    if (gHeatTemp < 10) lcd.print(F(" "));
    lcd.print(gHeatTemp);
    lcd.write(0xdf);
  }
#endif
}
//0123456789ABCDEF
//S:25+1.0ﾟAUTO  ^v-_
//T:22.0C -80s 10ﾟ

//SP:25<0C OPEN
//SP:25<0C CLOSE
//SP:25<0C STOP
//TP:25.0C -30s HT

void dispSelect() {
  lcd.setCursor(0, 1);
  lcd.print(F("SELECT TO SET"));
}



/////////////////////////////////////////////////////////////
//                動作設定保存、読み出し
/////////////////////////////////////////////////////////////
void readParam() {
  int i;
  if (EEPROM.read(PARAM_RSRV_ADDR) == PARAM_RSRV_MARK) {
    for (i = 0; i < PARAM_NUM; i++) {
      EEPROM.get(PARAM_RSRV_ADDR +  1 + (i * sizeof(gCtlParam[0].pValue)), gCtlParam[i].pValue);
    }
    gControlMode =(controlMode_t)gCtlParam[MANU_MODE].pValue;
    gSetPoint =                  gCtlParam[TEMP_MODE].pValue;
#ifdef HEATER
    gHeatTemp =                  gCtlParam[HEAT_MODE].pValue;
#endif
    gSensitivity =               gCtlParam[SENS_MODE].pValue;
    gUpWindingTimeLimit =        gCtlParam[LENG_MODE].pValue;
    gWindingTime =               gCtlParam[WIND_MODE].pValue;
    gPauseTime =                 gCtlParam[INTV_MODE].pValue;
    gAdditionalDownWindingTime = gCtlParam[REWD_MODE].pValue;
    gOffsetTemp =                gCtlParam[OFS_MODE].pValue;
#ifdef RTC_SD
    gLogMode =                   gCtlParam[LOG_MODE].pValue;
#endif
    if (gControlMode == C_STOP_MODE) {        // 電源OFF時に手動停止モードの時は、巻上カウンタ値を元に戻す。
      EEPROM.get(PARAM_RSRV_ADDR + 1 + (PARAM_NUM * sizeof(gCtlParam[0].pValue)), gTotalUpWindingTime);
    }
  }
}


void writeParam() {
  int i;
  gCtlParam[MANU_MODE].pValue = gControlMode;
  gCtlParam[TEMP_MODE].pValue = gSetPoint;
#ifdef HEATER
  gCtlParam[HEAT_MODE].pValue = gHeatTemp;
#endif
  gCtlParam[SENS_MODE].pValue = gSensitivity;
  gCtlParam[LENG_MODE].pValue = gUpWindingTimeLimit;
  gCtlParam[WIND_MODE].pValue = gWindingTime;
  gCtlParam[INTV_MODE].pValue = gPauseTime;
  gCtlParam[REWD_MODE].pValue = gAdditionalDownWindingTime;
  gCtlParam[OFS_MODE].pValue = gOffsetTemp;
#ifdef RTC_SD
  gCtlParam[LOG_MODE].pValue = gLogMode;
#endif
  EEPROM.write(PARAM_RSRV_ADDR, PARAM_RSRV_MARK);
  for (i = 0; i < PARAM_NUM; i++) {
    EEPROM.put(PARAM_RSRV_ADDR + 1 + (i * sizeof(gCtlParam[0].pValue)), gCtlParam[i].pValue);
  }
  if (gControlMode == C_STOP_MODE) {        // 電源OFF時に手動停止モードの時は、巻上カウンタ値も保存する。
    EEPROM.put(PARAM_RSRV_ADDR + 1 + (PARAM_NUM * sizeof(gCtlParam[0].pValue)), gTotalUpWindingTime);
  }
}


/////////////////////////////////////////////////////////////
//           ログ記録処理関係
/////////////////////////////////////////////////////////////


#ifdef RTC_SD

bool isLogTime(tmElements_t tm) {
  byte tmm = tm.Minute;
  static byte loggedTime = 99;
  int logInt = gLogInt[gLogMode];     // ログ間隔（分）

  if (!gIsSD) return false;
  if (logInt == 0) return false;
  if ((tmm % logInt) == 0) {         // ログ時間の場合
    if (tmm != loggedTime) {         // ログ済みでない場合は
      loggedTime = tmm;;             // ログ時間を保存
      return true;                   // ログ時間としてリターン
    }
  }
  return false;
}


void writeLog(tmElements_t tm, int tmpTenFold, bool isFlush) {
  static byte bufNo = 0;

  if (!gIsSD) return;
  if (!isFlush) {
    gLogData[bufNo].Year        = tm.Year;
    gLogData[bufNo].Month       = tm.Month;
    gLogData[bufNo].Day         = tm.Day;
    gLogData[bufNo].Hour        = tm.Hour;
    gLogData[bufNo].Minute      = tm.Minute;
    gLogData[bufNo].TempTenFold = tmpTenFold;     // 気温×10
    bufNo++;
    if (bufNo >= BUFF_NO) {  // バッファの上限になったら
      writeBuf(BUFF_NO);
      bufNo = 0;
    }
  } else {
    if (bufNo > 0) {
      writeBuf(bufNo);
      bufNo = 0;
    }
  }
}


void writeBuf(byte bufNo) {
  for (byte i = 0; i < bufNo; i++) {  
    logfile.print(tmYearToCalendar(gLogData[i].Year));
    logfile.print(F("/"));
    logfile.print(gLogData[i].Month);
    logfile.print(F("/"));
    logfile.print(gLogData[i].Day);
    logfile.print(F(","));
    logfile.print(gLogData[i].Hour);
    logfile.print(F(":"));
    logfile.print(gLogData[i].Minute);
    logfile.print(F(","));
    logfile.println((float)gLogData[i].TempTenFold / 10, 1);
  }
}

/////////////////////////////////////////////////////////////
//                    SDカードの設定
/////////////////////////////////////////////////////////////
// SDカードのイニシャライズ
// https://github.com/adafruit/Light-and-Temp-logger/blob/master/lighttemplogger.ino
bool setupSD() {
  // initialize the SD card
#ifdef SERIAL_MONITOR
  Serial.print(F("Initializing SD card..."));
#endif
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(chipSelect, OUTPUT);

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
#ifdef SERIAL_MONITOR
    Serial.println(F("Card failed, or not present"));
#endif
    // don't do anything more:
    return false;
  }
#ifdef SERIAL_MONITOR
  Serial.println(F("card initialized."));
#endif
  // create a new file
  bool isFileOpen;
  char filename[] = "LOGGER00.CSV";
  SdFile::dateTimeCallback( &dateTime );
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i / 10 + '0';
    filename[7] = i % 10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
#ifdef SdFatLite
      isFileOpen = logfile.open(filename, O_WRONLY | O_CREAT | O_EXCL);
#else
      logfile = SD.open(filename, FILE_WRITE);
#endif
      break;  // leave the loop!
    }
  }
#ifdef SdFatLite
  if (!isFileOpen) {
#else
  if (!logfile) {
#endif
#ifdef SERIAL_MONITOR
    Serial.println(F("couldnt create file"));
#endif
    return false;
  }
#ifdef SERIAL_MONITOR
  Serial.print(F("Logging to: "));
  Serial.println(filename);
#endif
  //  logfile.close();
  return true;
}


// SDカードのファイル作成日付設定用
// 色んな所で使われているコードで、どれがオリジナルかは不明
void dateTime(uint16_t* date, uint16_t* time) {

  //setSyncProvider(myRTC.get()); //sync time with RTC
  tmElements_t tm;
  myRTC.read(tm);          // RTCから時刻を読む

  // FAT_DATEマクロでフィールドを埋めて日付を返す
  // fill date to FAT_DATE macro
  *date = FAT_DATE(tmYearToCalendar(tm.Year), tm.Month, tm.Day);

  // FAT_TIMEマクロでフィールドを埋めて時間を返す
  // fill time to FAT_TIME macro
  *time = FAT_TIME(tm.Hour, tm.Minute, tm.Second);
}


/////////////////////////////////////////////////////////////
//            LCD、キーパッドを使った時計の表示、設定
/////////////////////////////////////////////////////////////
//////////////////// RTC SET ////////////////////////
// set RTC clock using LCD keypad shield           //
/////////////////////////////////////////////////////
void setRTCwKeypad() {
  //  int i;
  byte tStart[5] = {CalendarYrToTm(2022), 1, 1, 0, 0}; // Year member of tmElements_t is an offset from 1970
  byte tValue[5];
  byte tLimit[5] = {CalendarYrToTm(2099), 12, 31, 23, 59};
  byte item = 0; // year=0, month=1, day=2, hour=3, minute=4
  bool isChanged = false;        //日時の修正有無
  byte lcd_key;                  // キー入力コード
  unsigned long waitTime = millis() + TIME_LIMIT;  // TIME_LIMITms経過したら時計設定完了していなくても終了
  tmElements_t tm;

  myRTC.read(tm);          // RTCから時刻を読む
  tValue[0] = tm.Year;
  tValue[1] = tm.Month;
  tValue[2] = tm.Day;
  tValue[3] = tm.Hour;
  tValue[4] = tm.Minute;

  byte cursorColumn[5] = {0, 5, 8, 0x0b, 0x0e};
  // |    |  |  |  |
  // 0123456789abcdef
  // 2019/07/01 16:15

  lcd.clear();
  lcdDayTime(tm, 0, cursorColumn[item]);
  do {
    if (item == 2) tLimit[2] = daysInMonth(tm); // last day number in the month
    lcd_key = keypad.read_buttons(); // read the buttons
    switch (lcd_key) {               // depending on which button was pushed, we perform an action
      case btnRIGHT: {
        if (item < 4) {item++;}
        break;
      }
      case btnLEFT: {
        if (item > 0) {item--;}
        break;
      }
      case btnUP: {
        if (tValue[item] >= tLimit[item]) {
          tValue[item] = tStart[item]; // value will roll over from the start point if reaches the limit
        } else {
          tValue[item]++;              // increase the value if button is pressed
        }
        isChanged = true;
        break;
      }
      case btnDOWN: {
        if (tValue[item] <= tStart[item]) {
          tValue[item] = tLimit[item]; // value will roll over to the limit point if reaches the start point.
        } else {
          tValue[item]--;              // if button is pressed increase the value
        }
        isChanged = true;
        break;
      }
      case btnSELECT: {
        break;
      }
      case btnNONE: {
        break;
      }
    }
    if (lcd_key != btnNONE) {
      tm = {0, tValue[4], tValue[3], 0, tValue[2], tValue[1], tValue[0]}; //設定値をtmに代入したら whileに戻って表示
      lcdDayTime(tm, 0, cursorColumn[item]);
    }
    if (waitTime < millis()) {             // 指定時間を超えたら自動的に設定終了
      lcd_key = btnSELECT;
#ifdef SERIAL_MONITOR
      Serial.println(F("Time Up"));
#endif
    }
  } while (lcd_key != btnSELECT);
  if (isChanged) myRTC.write(tm);            // 設定値をRTCに設定。ゼロ秒で押せば秒まで正確に設定できる。
  lcd.noBlink();
}


// うるう年を考慮して各月の日数を計算
byte daysInMonth(tmElements_t tm) {
  byte days[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}; //days of eath month
  if (!(tmYearToCalendar(tm.Year) % 4)) {   // leap year
    days[1] = 29;
  }
  return days[tm.Month - 1];
}


// LCDに日時を表示
void lcdDayTime(tmElements_t tm, byte row, char cursorColumn) {
  lcd.setCursor(0, row);
  lcd.print(tmYearToCalendar(tm.Year));
  lcd.print(F("/"));
  lcdDay(tm.Month, tm.Day);
  lcd.print(F(" "));
  lcd.setCursor(11, row);
  lcdTime(tm.Hour, tm.Minute);
//  if (tm.Hour < 10) lcd.print(F("0"));
//  lcd.print(tm.Hour, DEC);
//  lcd.print(F(":"));
//  if (tm.Minute < 10) lcd.print(F("0"));
//  lcd.print(tm.Minute, DEC);
  if (cursorColumn >= 0) {
    lcd.setCursor(cursorColumn, 0);
    lcd.blink();
  }
}


void lcdDay(byte mt, byte dy) {
  if (mt < 10) lcd.print(F("0"));
  lcd.print(mt, DEC);
  lcd.print(F("/"));
  if (dy < 10) lcd.print(F("0"));
  lcd.print(dy, DEC);
}


void lcdTime(byte hr, byte mn) {
  if (hr < 10) lcd.print(F("0"));
  lcd.print(hr, DEC);
  lcd.print(F(":"));
  if (mn < 10) lcd.print(F("0"));
  lcd.print(mn, DEC);
}

#endif
