/*
20250803 1705 更新

※ESP32-C3上のエラーがあるためアナログ入力としては使えない
ESP32-C3 Series Datasheet Version 2.1 P.49より下記の記述あり
ADC2 of some chip revisions is not operable. For details, please refer to ESP32-C3 Series SoC Errata.
訳：一部のチップリビジョンのADC2は動作しません。詳細はESP32-C3シリーズSoCのエラッタを参照してください。
*/

#include <M5Unified.h>
#include <FastLED.h>
#include <ESP32Servo.h> // by Kevin Harrington
#include <Wire.h>

#define onBordLED_PIN 2 // オンボードフルカラーLEDの使用端子
#define onBordSW_PIN 3  // オンボードスイッチの使用端子
// #define PWM1_PIN 4
// #define PWM2_PIN 5
// #define PWM3_PIN 8
// #define PWM4_PIN 10
#define FREQUENCY 50
#define RESOLUTION 12

#define Analog1_PIN 0
#define Analog2_PIN 1
#define Analog3_PIN 9//4
#define Analog4_PIN 18
//ダメなピン 5,6,7,8,10,


// WS2812テープLED関連
#define WS2812_PIN 19    // WS2812 LEDテープ用の信号ピン
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
const int servo_min_us = 1000;
const int servo_max_us = 2000;
int angleToDuty(int angle, int min_us, int max_us, int resolution, int freq) {
  int duty_max = (1 << resolution) - 1;
  float period_us = 1000000.0 / freq;
  float pulse_us = min_us + (max_us - min_us) * (angle / 180.0);
  return (int)((pulse_us / period_us) * duty_max);
}
void setServoAngle(int channel, int angle) {
  int duty = angleToDuty(angle, servo_min_us, servo_max_us, RESOLUTION, FREQUENCY);
  ledcWrite(channel, duty);
}

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  Serial.begin(115200);

  //ピンモード設定
  pinMode(onBordSW_PIN, INPUT_PULLUP);//オンボードスイッチ
  pinMode(onBordLED_PIN,OUTPUT);//オンボードLED
  pinMode(Analog1_PIN, INPUT);//
  pinMode(Analog2_PIN, INPUT);//
  pinMode(Analog3_PIN, INPUT);//
  pinMode(Analog4_PIN, INPUT);//

  // オンボードLED設定
  FastLED.addLeds<WS2812B, onBordLED_PIN, GRB>(leds, num_leds);

  // WS2812 LEDテープ設定
  FastLED.addLeds<WS2812B, WS2812_PIN, GRB>(tape_leds, WS2812_NUM);

  publicTimer = timerBegin(0, 80, true);
  timerAttachInterrupt(publicTimer, &onpublicTimer, true);
  timerAlarmWrite(publicTimer, 10000, true);
  timerAlarmEnable(publicTimer);

  // LED初期点灯色
  leds[0] = CRGB::White;
  FastLED.show();

  // PWM設定
  ledcSetup(0, FREQUENCY, RESOLUTION);
  ledcSetup(1, FREQUENCY, RESOLUTION);
  ledcSetup(2, FREQUENCY, RESOLUTION);
  ledcSetup(3, FREQUENCY, RESOLUTION);
  // ledcAttachPin(PWM1_PIN, 1);//ピン番号とPWMチャンネルの割り当て
  // ledcAttachPin(PWM2_PIN, 2);
  // ledcAttachPin(PWM3_PIN, 3);
  // ledcAttachPin(PWM4_PIN, 4);
}

void loop() {
  M5.update();

  if (digitalRead(onBordSW_PIN) == 1) {
    setServoAngle(1, 90);
    setServoAngle(2, 90);
    setServoAngle(3, 90);
    setServoAngle(4, 90);
    leds[0] = CRGB(100, 100, 0);

    // WS2812 テープを黄色点灯
    for (int i = 0; i < WS2812_NUM; i++) {
      tape_leds[i] = CRGB(100, 100, 0);
    }
  } else {
    setServoAngle(1, 95);
    setServoAngle(2, 95);
    setServoAngle(3, 95);
    setServoAngle(4, 95);
    leds[0] = CRGB(0, 0, 100);

    // WS2812 テープを青色点灯
    for (int i = 0; i < WS2812_NUM; i++) {
      tape_leds[i] = CRGB(0, 0, 100);
    }
  }

  if(count_timer_500_one != 0){count_timer_500_one == 0;
    Serial.printf("%04d,",analogRead(Analog1_PIN));
    Serial.printf("%04d,",analogRead(Analog2_PIN));
    Serial.printf("%04d,",analogRead(Analog3_PIN));
    Serial.printf("%04d",analogRead(Analog4_PIN));

    Serial.println();
  }

  FastLED.show();
}
