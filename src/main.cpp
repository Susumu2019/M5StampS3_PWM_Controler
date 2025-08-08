/*
20250803 1710 更新
*/
#include <M5Unified.h>
#include <FastLED.h>
#include <ESP32Servo.h> // by Kevin Harrington
#include <Wire.h>

#define onBordLED_PIN 21 // オンボードフルカラーLEDの使用端子
#define onBordSW_PIN 0  // オンボードスイッチの使用端子
#define PWM1_PIN 8
#define PWM2_PIN 9
#define PWM3_PIN 10
#define PWM4_PIN 11
#define FREQUENCY 50
#define RESOLUTION 12

#define Analog1_PIN 1
#define Analog2_PIN 2
#define Analog3_PIN 3
#define Analog4_PIN 4
//ダメなピン 5,6,7,8,10,

// WS2812テープLED関連
#define WS2812_PIN 39    // WS2812 LEDテープ用の信号ピン
#define WS2812_NUM 5    // テープのLED数
CRGB tape_leds[WS2812_NUM];

hw_timer_t *publicTimer = NULL;

uint8_t count_timer_10 = 0;
uint8_t count_timer_10_one = 0;
uint8_t count_timer_100 = 0;
uint8_t count_timer_100_one = 0;
uint8_t count_timer_500 = 0;
uint8_t count_timer_500_one = 0;
int16_t count_timer_1000 = 0;
int16_t count_timer_1000_one = 0;
int16_t count_flicker_100 = 0;
uint16_t count_flicker_500 = 0;
int16_t count_flicker_1000 = 0;

// オンボードLED関連
const int num_leds = 1;
CRGB leds[num_leds];

void ARDUINO_ISR_ATTR onpublicTimer() {
  count_timer_10_one = 1;

  if (count_timer_100 == 9) {
    count_timer_100 = 0;
    count_timer_100_one = 1;
    count_flicker_100 = !count_flicker_100;
  } else {
    count_timer_100++;
  }

  if (count_timer_500 == 49) {
    count_timer_500 = 0;
    count_timer_500_one = 1;
    count_flicker_500 = !count_flicker_500;
  } else {
    count_timer_500++;
  }

  if (count_timer_1000 == 99) {
    count_timer_1000 = 0;
    count_timer_1000_one = 1;
    count_flicker_1000 = !count_flicker_1000;
  } else {
    count_timer_1000++;
  }
}

// サーボ関係
const int servo_min_us = 600;
const int servo_max_us = 2400;
int angleToDuty(int angle, int min_us, int max_us, int resolution, int freq) {
  int duty_max = (1 << resolution) - 1;
  float period_us = 1000000.0 / freq;
  float pulse_us = min_us + (max_us - min_us) * (angle / 180.0);
  return (int)((pulse_us / period_us) * duty_max);
}
void setServoAngle(int channel, int angle) {//サーボに角度指定で制御
  int duty = angleToDuty(angle, servo_min_us, servo_max_us, RESOLUTION, FREQUENCY);
  ledcWrite(channel, duty);
}
int adcToDegree(int adcValue) {//取得したアナログ値を角度に変換
  return map(adcValue, 0, 4095, 0, 180);
}

void setup() {
  USBSerial.begin(115200);
  auto cfg = M5.config();
  M5.begin(cfg);

  //ピンモード設定
  pinMode(onBordSW_PIN, INPUT_PULLUP);//オンボードスイッチ
  pinMode(onBordLED_PIN,OUTPUT);//オンボードLED
  pinMode(Analog1_PIN, INPUT);//
  pinMode(Analog2_PIN, INPUT);//
  pinMode(Analog3_PIN, INPUT);//
  pinMode(Analog4_PIN, INPUT);//
  pinMode(PWM1_PIN, OUTPUT);//
  pinMode(PWM2_PIN, OUTPUT);//
  pinMode(PWM3_PIN, OUTPUT);//
  pinMode(PWM4_PIN, OUTPUT);//

  // オンボードLED設定
  FastLED.addLeds<WS2812B, onBordLED_PIN, GRB>(leds, num_leds);

  // WS2812 LEDテープ設定
  FastLED.addLeds<WS2812B, WS2812_PIN, GRB>(tape_leds, WS2812_NUM);

  publicTimer = timerBegin(0, 80, true);
  timerAttachInterrupt(publicTimer, &onpublicTimer, true);
  timerAlarmWrite(publicTimer, 10000, true);
  timerAlarmEnable(publicTimer);

  // PWM設定
  ledcSetup(1, FREQUENCY, RESOLUTION);
  ledcSetup(2, FREQUENCY, RESOLUTION);
  ledcSetup(3, FREQUENCY, RESOLUTION);
  ledcSetup(4, FREQUENCY, RESOLUTION);
  ledcAttachPin(PWM1_PIN, 1);//ピン番号とPWMチャンネルの割り当て
  ledcAttachPin(PWM2_PIN, 2);
  ledcAttachPin(PWM3_PIN, 3);
  ledcAttachPin(PWM4_PIN, 4);

  // ledcDetachPin(PWM1_PIN);//念のため無効とする
  // ledcDetachPin(PWM2_PIN);//念のため無効とする
  // ledcDetachPin(PWM3_PIN);//念のため無効とする
  // ledcDetachPin(PWM4_PIN);//念のため無効とする

  // LED初期点灯色
  leds[0] = CRGB(0, 255, 0);//RGB CRGB::White;
  // WS2812 テープを黄色点灯
  for (int i = 0; i < WS2812_NUM; i++) {
    tape_leds[i] = CRGB(0, 255, 0);//RGB
  }
  FastLED.show();
  delay(1000);

  // delay(2000);
  // USBSerial.println("Start");
}

void loop() {
  M5.update();

  if (digitalRead(onBordSW_PIN) == 1) {
    setServoAngle(1, adcToDegree(analogRead(Analog1_PIN)));
    setServoAngle(2, adcToDegree(analogRead(Analog2_PIN)));
    setServoAngle(3, adcToDegree(analogRead(Analog3_PIN)));
    setServoAngle(4, 180-adcToDegree(analogRead(Analog4_PIN)));//出力反転

    //setServoAngle(1, 90);
    //setServoAngle(2, 90);
    //setServoAngle(3, 90);
    //setServoAngle(4, 90);

    leds[0] = CRGB(0, 100, 0);

    // WS2812 テープを黄色点灯
    for (int i = 0; i < WS2812_NUM; i++) {
      tape_leds[i] = CRGB(255, 255, 255);//RGB
    }
  } else {
    // setServoAngle(1, 90);
    // setServoAngle(2, 90);
    // setServoAngle(3, 90);
    // setServoAngle(4, 90);

    leds[0] = CRGB(0, 0, 255);

    // WS2812 テープを青色点灯
    for (int i = 0; i < WS2812_NUM; i++) {
      tape_leds[i] = CRGB(0, 0, 255);
    }
  }

  if(count_timer_100_one != 0){count_timer_100_one = 0;
    // USBSerial.printf("%04d,",adcToDegree(analogRead(Analog1_PIN)));
    // USBSerial.printf("%04d,",adcToDegree(analogRead(Analog2_PIN)));
    // USBSerial.printf("%04d,",adcToDegree(analogRead(Analog3_PIN)));
    // USBSerial.printf("%04d",adcToDegree(analogRead(Analog4_PIN)));
  }

  FastLED.show();
}



