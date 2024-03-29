//#define SHINSUNG
//https://github.com/espressif/arduino-esp32/releases/download/1.0.0/package_esp32_dev_index.json
#define VERSION "2.04"
#define DEFAULT_DROP 0.05        // rdrop 한방울의 무게
#define DEFAULT_ZERO_SPEED 0.017 // 3cc/hr 이하이면 0
#define DROP_TIME 10000          // 1분(60000)이상 떨어지지 않으면 0

#define HOST_URL_LENGTH 150

#define DEFAULT_AP "jong"
#define DEFAULT_PW "sstk0824"

#define SSID_SIZE 30
#define PW_SIZE 20

#define MAX_INDEX 17 //24LC64

#define SN_SIZE 20
#define VER_SIZE 20

//기본용량
#define ML_SIZE 20

#include <ArduinoJson.h>
#include <Arduino.h>
#include <Wire.h>
#include <Ticker.h>
#include <WiFi.h>
#define I2C_ADDR 0x70
#include <WiFiMulti.h>
#include <HTTPClient.h>
#include "EEPROM.h"
#define EEPROM_SIZE 512

#define INDEX_ADDR (MAX_INDEX * 50)

#define HOST_ADDR (INDEX_ADDR + 10)
#define VER_ADDR (HOST_ADDR + HOST_URL_LENGTH)
#define SN_ADDR (VER_ADDR + VER_SIZE)
#define AP_ADDR (SN_ADDR + SN_SIZE)
#define PW_ADDR (AP_ADDR + SSID_SIZE)
#define ML_ADDR (PW_ADDR + PW_SIZE)

#define MAX_VOL 4200
#define MIN_VOL 3100

#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>

//define pins of TFT screen
#define TFT_CS 12
#define TFT_RST 14
#define TFT_DC 13
#define TFT_SCLK 22
#define TFT_MOSI 21
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
float p = 3.1415926;

#define SDA1 18
#define SCL1 19

char host_url[HOST_URL_LENGTH] = "https://iringer.kr:8443/";
// char ip[25]="http://192.168.0.100/";
char base64[65] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
char default_ap[SSID_SIZE + 1] = {DEFAULT_AP};
char default_pw[PW_SIZE + 1] = {DEFAULT_PW};
char serial_num[SN_SIZE + 1] = {0, };//21 자 

//인터넷연결체크
int SIGNAL_CHK = 0; // 0 Wifi 연결 안됨 1 Wifi 연결됨 2 서버 연결 됨
int SIGNAL_CHK_OLD = -1;
int thirty_counter = 0; // 30회에 한 번 실행하기 위한 플래그

byte bat_init_flag = 0;

uint32_t voltage, current, max_voltage = 0;
#define VoltageFactor 9011  /* LSB=2.20mV ~9011/4096 - convert to mV          */
#define CurrentFactor 24084 /* LSB=5.88uV/R= ~24084/R/4096 - convert to mA    */
#define RSENSE 10           /* sense resistor in mOhms            */

WiFiMulti wifiMulti;
HTTPClient http;
uint8_t ap_list_count = 0;
byte ap_list[MAX_INDEX][SSID_SIZE + PW_SIZE];
uint32_t tmr = 0, err_cnt = 0;
u8_t upload_counter = 0;
bool show_serial_log = false;
bool server_connected = false;
bool print_debug = false;

#define PING_PERIOD 1000
#define PING_TIMEOUT 900
#define PING_TOTAL_COUNT 3
#define ADC_PERIOD 5000

#define MEAS_INTERVAL 5

#define MIN_DELAY 1
#define MAX_DELAY 30

#define MAX_ERROR 5

uint16_t old_max = 0;
float old_rest = 0;
uint16_t old_mlph = 0;
int fRate_old = 0;

String time_rest_old = ""; //"00:00";
String time_rest = "";
String time_progress_old = "00:00";
String time_progress = "";

typedef struct ringer
{
  uint16_t r_volume_max;// 설정용량
  float r_volume_now;//남은용량
  float r_adrop;
  float drop_per_sec;
  float last_drop_per_sec;
  uint32_t nBat;
  uint32_t drop_cnt;
  uint16_t ml_per_hour;
  uint16_t rest_min;
  uint16_t ordered_gtt;
  uint16_t min_gtt;
  uint16_t max_gtt;
  bool is_monitoring;
} ringerData_t;

uint32_t old_bat = 0;
float old_rate;

static ringerData_t g_RingerData = {
  0,
};

struct Button
{
  const uint8_t PIN;
  uint32_t numberKeyPresses;
  bool pressed;
};

Button wps_key = {25, 0, false};
//Button bat_alarm = {27, 0, false};
Button pwr_chg = {12, 0, false};




Button ir_rx = {17, 0, false};
int pnIrRx = 17;

#define boot 0

#define ir_tx 16
#define led_power 23
#define usb_detect 33

uint64_t chipid;
Ticker timer;

#include <stdio.h>
#include <stdarg.h>



void IRAM_ATTR isr1()
{
  wps_key.numberKeyPresses += 1;
  wps_key.pressed = true;
  //printf("Key Pressed\n");
}

 
void IRAM_ATTR isr3()
{
  pwr_chg.numberKeyPresses += 1;
  pwr_chg.pressed = true;
  //printf("pwr_chg Pressed\n");
}

char bIsDrop = 0;
uint32_t v1 = 0;
uint32_t drop_tmr = 0, old_tmr = 0;

void IRAM_ATTR isr4()
{
  uint32_t now;
  uint32_t elaps;

  char bOn = digitalRead(pnIrRx);
  float a;

  ir_rx.numberKeyPresses += 1;
  ir_rx.pressed = true;

  now = tmr;
  elaps = now - v1;

  // _printf("Drop %d\n", bOn);

  if (bOn == 1) //감지
  {
    if (elaps >= 70) //이전에 감지와 일정 시간 이상
    {
      bIsDrop = 1;
      g_RingerData.drop_cnt++;
      old_tmr = drop_tmr;
      drop_tmr = 0;
      v1 = now;
    }
  }
  else //해지
  {
    if ((elaps >= MIN_DELAY) && (bIsDrop == 1)) //일정 시간 이상의 방울 길이일 때
    {
      bIsDrop = 0;
    }
  }
}

u16_t sec_tmr = 0;
word bat_tmr = 0;
u8_t sec_flag = 0;

void timer_isr()
{

  tmr++;
  if (++sec_tmr == 1000)
  {
    sec_tmr = 0;
    sec_flag = 1;
  }

  if (++bat_tmr == 5000)
  {
    bat_tmr = 0;
    bat_init_flag = 1;
  }

  if (++drop_tmr >= DROP_TIME)
  {
    drop_tmr = DROP_TIME;
  }
}

#define FIRST_X 4
#define FIRST_Y 2

#include <Fonts/FreeSerifItalic9pt7b.h>
#include <Fonts/FreeSerifItalic12pt7b.h>

void print_num4(uint16_t n)
{
  if (n < 1000)
    space(1);
  if (n < 100)
    space(1);
  if (n < 10)
    space(1);

  tft.setTextColor(ST7735_BLUE);
  tft.print(n);
}

//하단 남은용량
void print_fnum4(float n)
{
  if (n < 1000)
    space(1);
  if (n < 100)
    space(1);
  if (n < 10)
    space(1);

  tft.setTextColor(ST7735_RED);
  char strdisp[6] = {
    0,
  };
  sprintf(strdisp, "%.1f", n);
  //printf("print_fnum4() strdisp=%f\n",strdisp);
  tft.print(strdisp);
  //tft.print( "ml");
}

void print_num3(uint16_t n)
{
  if (n < 1000)
    space(1);
  if (n < 100)
    space(1);
  if (n < 10)
    space(1);
  tft.setTextColor(ST7735_WHITE);
  tft.print(n);
}

//수액센서 데이터수신시 표출하는곳
void print_num3_2(uint16_t n)
{
  if (n < 1000)
    space2(1);
  if (n < 100)
    space2(1);
  if (n < 10)
    space2(1);
  //tft.print(n);
  tft.print(n);
}
void print_num4x(uint16_t n)
{
  if (n < 1000)
    spacex(1);
  if (n < 100)
    spacex(1);
  if (n < 10)
    spacex(1);
  tft.print(n);
}

void print_fnum4x(float n)
{
  if (n < 1000)
    spacex(1);
  if (n < 100)
    spacex(1);
  if (n < 10)
    spacex(1);

  //printf("print_fnum4x() n:%f \n",n);
  tft.print(n);
}

void print_num5x_2(uint16_t n)
{
  if (n < 1000)
    space5(1);
  if (n < 100)
    space5(1);
  if (n < 10)
    space5(1);
  //tft.print(n);
  tft.print(n);
}

void print_num5x(uint16_t n)
{
  if (n < 1000)
    spacex(1);
  if (n < 100)
    spacex(1);
  if (n < 10)
    spacex(1);
  tft.print(n);
}

void print_num3x(uint16_t n)
{
  if (n < 1000)
    spacex(1);
  if (n < 100)
    spacex(1);
  if (n < 10)
    spacex(1);
  tft.print(n);
}

void print_fnum3x(float n)
{
  if (n < 100)
    spacex(1);
  if (n < 10)
    spacex(1);
  tft.print(n);
}

void space(uint8_t n)
{
  uint8_t i;
  tft.setTextColor(ST7735_BLACK);
  for (i = 0; i < n; i++)
    tft.print(0);
  //tft.setTextColor(ST7735_WHITE);
}

void spacex(uint8_t n)
{
  uint8_t i;
  for (i = 0; i < n; i++)
    tft.print(0);
}
//수액센서데이터 폰트색 지정
void space2(uint8_t n)
{
  uint8_t i;
  tft.setTextColor(ST7735_BLACK);
  for (i = 0; i < n; i++)
    tft.print(0);
  tft.setTextColor(ST7735_YELLOW); //속도 체크시 데이터 들어가는 곳 폰트 셋팅
}

//센서데이터 폰트색 지정
void space5(uint8_t n)
{
  uint8_t i;
  tft.setTextColor(ST7735_BLACK);
  for (i = 0; i < n; i++)
    tft.print(0);
  tft.setTextColor(ST7735_GREEN); //속도 체크시 데이터 들어가는 곳 폰트 셋팅
}

void init_tft()
{
  tft.initR(INITR_144GREENTAB); //initialize a ST7735S chip, black tab
  tft.setRotation(3);
  tft.fillScreen(ST7735_BLACK); //large block of text
  tft.setTextWrap(false);
  tft.setFont(&FreeSerifItalic9pt7b);
  tft.setTextColor(ST7735_BLUE);
  tft.setTextSize(1);
}

//초기 한번만 설정 
void display_main()
{
  uint16_t i;

 // old_max = read_ml();
 // if (old_max == 0) old_max = 1;
  //printf("display_main() old_max 2 : %lu \n", old_max);

  //총수액용량
  tft.setFont(&FreeSerifItalic9pt7b);
  tft.setCursor(FIRST_X + 70, FIRST_Y + 90);
  space(3);
  tft.setTextColor(ST7735_BLUE);
  tft.print("ml");

  //배터리용량
  tft.setCursor(FIRST_X + 65, FIRST_Y + 12);
  space(4);
  tft.setTextColor(ST7735_WHITE);
  tft.print("%");

  // gtt
  tft.setFont(&FreeSerifItalic12pt7b);
  tft.setCursor(FIRST_X + 65, FIRST_Y + 40);
  tft.setTextColor(ST7735_WHITE);
  tft.print("gtt");

  // cc/hr 부분
  tft.setFont(&FreeSerifItalic9pt7b);
  tft.setCursor(FIRST_X + 65, FIRST_Y + 65);
  tft.setTextColor(ST7735_WHITE);
  tft.print("cc/hr");

  //하단남은용량 첫실행시 기본셋팅
  old_rest = old_max;

  tft.setFont(&FreeSerifItalic9pt7b);
  tft.setCursor(FIRST_X + 96, FIRST_Y + 112); //하단 ml위치
  tft.setTextColor(ST7735_RED);
  tft.print("ml"); //하단 용량

  //하단 현재시간
  tft.setCursor(FIRST_X + 2, FIRST_Y + 90);
  tft.setTextColor(ST7735_WHITE);

  //현재 진행시간
  tft.setCursor(FIRST_X + 2, FIRST_Y + 112);
  tft.setTextColor(ST7735_YELLOW);
}

int old_rest_chk = 0;

void print_data() // 화면 표시
{
  if (show_serial_log) {
    printf("ml_per_hour %4.2f %4.2f\n", old_mlph, g_RingerData.ml_per_hour);
    Serial.print("time_progress ");
    Serial.print(time_progress_old);
    Serial.print(" ");
    Serial.println(time_progress);
    Serial.print("time_rest ");
    Serial.print(time_rest_old);
    Serial.print(" ");
    Serial.println(time_rest);
  }

  if (SIGNAL_CHK != SIGNAL_CHK_OLD) {//0 Wifi 연결 안됨 1 Wifi 연결됨 2 서버 연결 됨
    //연결상태
    tft.setCursor(FIRST_X + 2, FIRST_Y + 12);
    //spacex(2);
    tft.setTextColor(ST7735_BLACK);
    if (SIGNAL_CHK_OLD == 0) 
    {
      tft.print("Wifi off");
    }
    else if (SIGNAL_CHK_OLD == 1)
    {
      tft.print("Wifi on");
    }
    else
    {
      tft.print("connected");
    }

    tft.setCursor(FIRST_X + 2, FIRST_Y + 12);
    if (SIGNAL_CHK == 0) //0 Wifi 연결 안됨 1 Wifi 연결됨 2 서버 연결 됨
    {
      tft.setTextColor(ST7735_RED);
      tft.print("Wifi off");
    } 
    else if (SIGNAL_CHK == 1)
    {
      tft.setTextColor(ST7735_YELLOW);
      tft.print("Wifi on");
    }
    else if (SIGNAL_CHK == 2)
    {
      tft.setTextColor(ST7735_GREEN);
      tft.print("connected");
    }
    SIGNAL_CHK_OLD = SIGNAL_CHK;
  }


  if (time_progress_old != time_progress)
  {
    //현재 시간
    tft.setFont(&FreeSerifItalic9pt7b);
    tft.setCursor(FIRST_X + 2, FIRST_Y + 90);
    tft.setTextColor(ST7735_BLACK);
    tft.print(time_progress_old);

    tft.setCursor(FIRST_X + 2, FIRST_Y + 90);
    tft.setTextColor(ST7735_WHITE);
    tft.print(time_progress);

    time_progress_old = time_progress;
  }

  if (time_rest_old != time_rest)
  {
    // 남은 시간
    tft.setFont(&FreeSerifItalic9pt7b);
    tft.setCursor(FIRST_X + 2, FIRST_Y + 112);
    tft.setTextColor(ST7735_BLACK);
    tft.print(time_rest_old);

    tft.setCursor(FIRST_X + 2, FIRST_Y + 112);
    tft.setTextColor(ST7735_YELLOW);
    tft.print(time_rest);

    time_rest_old = time_rest;
  }

  //  while(1);
  //수액기준용량 표시
  if (old_max != g_RingerData.r_volume_max)
  {
    tft.setFont(&FreeSerifItalic9pt7b);
    
    tft.setTextColor(ST7735_BLACK);
    tft.setCursor(FIRST_X + 58, FIRST_Y + 90);
    print_num4x(old_max);
    old_max = g_RingerData.r_volume_max;

    tft.setTextColor(ST7735_BLUE);
    tft.setCursor(FIRST_X + 58, FIRST_Y + 90);
//    EEPROM.put(ML_ADDR, old_max);
//    EEPROM.commit();
    print_num4(old_max);
    
  }

  if (old_bat != g_RingerData.nBat)
  {
    printf("old_bat %d: new_bat %d\n", old_bat, g_RingerData.nBat);
    tft.setFont(&FreeSerifItalic9pt7b);
    tft.setTextColor(ST7735_BLACK);
    tft.setCursor(FIRST_X + 62, FIRST_Y + 12);
    print_num3x(old_bat);
    old_bat = g_RingerData.nBat;
    //printf("print_data() old_bat2:%lu === g_RingerData.nBat2:%lu\n", old_bat, g_RingerData.nBat);
    tft.setTextColor(ST7735_WHITE);
    tft.setCursor(FIRST_X + 62, FIRST_Y + 12);
    print_num3(old_bat);
  }

  int fRate_new = g_RingerData.drop_per_sec * 60;
  /*
    gtt=1~10 d =0.05
    gtt=11~20 d =0.0507
    gtt=21~30 d =0.0513
    gtt=31~40 d =0.0523
    gtt=41~50 d =0.0528
    gtt=51~60 d =0.0538
    gtt=61~70 d =0.054
    gtt >70 일 때 D=0.056
  */

  if (fRate_old != fRate_new)
  {
    tft.setFont(&FreeSerifItalic12pt7b);
    tft.setCursor(FIRST_X + 15, FIRST_Y + 40);
    //spacex(2);
    tft.setTextColor(ST7735_BLACK);
    print_num5x(fRate_old);
    //tft.print(fRate_old);

    fRate_old = fRate_new;

    tft.setCursor(FIRST_X + 15, FIRST_Y + 40);
    //spacex(2);
    tft.setTextColor(ST7735_GREEN);
    //tft.print(fRate_old);
    print_num5x_2(fRate_old);
    tft.setFont(&FreeSerifItalic9pt7b);
  }

  //cc/hr
  if (old_mlph != g_RingerData.ml_per_hour)
  {
    tft.setFont(&FreeSerifItalic12pt7b);
    tft.setTextColor(ST7735_BLACK);
    tft.setCursor(FIRST_X + 15, FIRST_Y + 65);
    print_num3x(old_mlph);
    old_mlph = g_RingerData.ml_per_hour;

    tft.setTextColor(ST7735_YELLOW);
    tft.setCursor(FIRST_X + 15, FIRST_Y + 65);
    print_num3_2(old_mlph);
    tft.setFont(&FreeSerifItalic9pt7b);
  }

  //하단수액용량
  if (old_rest != roundf(g_RingerData.r_volume_now * 10) / 10)
  {
    if (old_rest_chk > 0)
      old_rest_chk = 0;

    tft.setFont(&FreeSerifItalic9pt7b);
    tft.setTextColor(ST7735_BLACK);
    tft.setCursor(FIRST_X + 42, FIRST_Y + 84 + 28);
    // spacex(1);
    print_fnum4x(old_rest);

    old_rest = roundf(g_RingerData.r_volume_now * 10) / 10;

    tft.setCursor(FIRST_X + 42, FIRST_Y + 84 + 28);

    //spacex(1);
    print_fnum4(old_rest);
  }
}



byte search_n_connect()
{
  int k;
  char searched_ap[SSID_SIZE + 1] = {0}; //검색된 SSID   SSID SIZE=30
  char c2[SSID_SIZE + 1] = {0}; //저장된 SSID
  char c3[PW_SIZE + 1] = {0}; //저장된 PW

  WiFi.disconnect();  delay(100);

  Serial.println("scan start");

  // WiFi.scanNetworks will return the number of networks found
  int n = 0;
  int i;
  int try_count = 3;
  char fSuccess = 0;

  while (n == 0 && 0 < try_count--)
  {
    n = WiFi.scanNetworks();
    if (n == 0)
    {
      Serial.println("no networks found");
      return 0;
    }
    delay(1000);
  }
  Serial.println("scan done");

  Serial.print(n);
  Serial.println(" networks found");

#ifndef LOW_SIGNAL
  for (i = 0; i < n; ++i)
#else
  for (i = n - 1; i >= 0; --i)
#endif
  {
    // Print SSID and RSSI for each network found
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(WiFi.SSID(i));
    Serial.print(" (");
    Serial.print(WiFi.RSSI(i));
    Serial.print(")");
    Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? " " : "*");
    delay(10);
    WiFi.SSID(i).toCharArray(searched_ap, sizeof(searched_ap));
    if (strlen(searched_ap) <= SSID_SIZE)
    {
      for (int j = 0; j < ap_list_count; j++)
      {
        memcpy(c2, &ap_list[j][0], SSID_SIZE);
        if (strcmp(searched_ap, c2) == 0)
        {
          memcpy(c3, &ap_list[j][SSID_SIZE], PW_SIZE);
          wifiMulti.addAP(searched_ap, c3);
          for (k = 0; k < 5; k++)
          {
            delay(1000);
            Serial.print("Try... "); Serial.print(searched_ap); Serial.print(":"); Serial.println(c3);
            if ((wifiMulti.run() == WL_CONNECTED))
            {
              Serial.print("Connected=>"); Serial.print(searched_ap); Serial.print(":"); Serial.println(c3);
              return 1;
            }
          }
          if (k == 5) Serial.println("Fail");
        }
      }
    }
  }
  return 0;
}

void setup() //su
{
  delay(1000);
  Serial.begin(115200);
  Serial.println("setup() Start");

  pinMode(wps_key.PIN, INPUT_PULLUP);
  attachInterrupt(wps_key.PIN, isr1, FALLING);
 
  pinMode(pwr_chg.PIN, INPUT_PULLUP);
  attachInterrupt(pwr_chg.PIN, isr3, FALLING);
  pinMode(boot, INPUT_PULLUP);
  pinMode(ir_tx, OUTPUT);
  digitalWrite(ir_tx, 1);
  timer.attach(0.001, timer_isr);
  pinMode(ir_rx.PIN, INPUT);
  pinMode(led_power, OUTPUT);
  pinMode(usb_detect, INPUT);
  attachInterrupt(ir_rx.PIN, isr4, CHANGE);

  if (digitalRead(usb_detect)) //  power_led by usb_detect
  {
    digitalWrite(led_power, HIGH);
  }
  else
  {
    digitalWrite(led_power, LOW);
  }

  g_RingerData.drop_per_sec = 0.0f;
  g_RingerData.r_volume_now = 0.0f;
  g_RingerData.nBat = 0;

  Wire.begin(SDA1, SCL1, 100000); //STC3115

  STC3115_get_id();
  STC3115_init();
  get_battery();
  // Set WiFi to station mode and disconnect from an AP if it was previously connected
  WiFi.mode(WIFI_STA);//클라이언트 모드 공유기(AP)
  read_sn();
  printf("host_url %s\n",host_url);
  init_tft();
  display_main();//초기 디스플레이 설정
  
  print_data();//show_serial_log  에의해서 일부 출력 for debug
 
 // read_ap_list();

//  if (ap_list_count > 0 && ap_list_count < MAX_INDEX) {
    SIGNAL_CHK = search_n_connect();// wifi 검색
//  }

  if (SIGNAL_CHK == 0) {//0 Wifi 연결 안됨 1 Wifi 연결됨 2 서버 연결 됨
    connect_ap(default_ap, default_pw);
    delay(1000);
  }

  if (wifiMulti.run() == WL_CONNECTED) {
    server_connected = send_data();;// tjio 수정 
    
//    get_list();//tjio remove 
  }
  // memset(ap_list, 0, sizeof(ap_list));
}//setup end 


byte connect_ap(char* ap, char* pw)
{
  printf("func connect_ap connect to %s\n", ap);
  wifiMulti.addAP(ap, pw);
  Serial.print("Try to connect:tjio ");
  Serial.println(ap);
  return 0;
}

bool check_gtt_out_of_range(float gtt) {
  if (gtt > g_RingerData.ordered_gtt * 1.5 || gtt < g_RingerData.ordered_gtt * 0.5) return true;
  if (gtt > g_RingerData.max_gtt || gtt < g_RingerData.min_gtt ) return true;
}

bool check_need_to_update(float drop_speed) {
  if (print_debug)
    printf("drop speed %f %d\n", drop_speed*60, g_RingerData.drop_cnt);
  float gtt = drop_speed * 60;
  float prev_gtt = g_RingerData.last_drop_per_sec * 60;

  if (gtt < prev_gtt * 0.5 || gtt > prev_gtt * 1.5) return true;

  return check_gtt_out_of_range(gtt) != check_gtt_out_of_range(prev_gtt);
}

/**
   디바이스에서 배터리 정보 및 속도 계산
   return 바로 서버에 정보를 보내야할지 여부
 **/
bool calc_drop()
{
  float a;
  float drop_speed = 0;
  bool need_to_update = false;
  //g_RingerData.nBat=900;

  if (g_RingerData.drop_cnt > 1)
  {
    a = (float)drop_tmr;
    if (drop_tmr > old_tmr + 1)
      drop_speed = 1000. / a;
    else
      drop_speed = 1000. / old_tmr;
    if (drop_speed < DEFAULT_ZERO_SPEED)
      drop_speed = 0; // 3cc/hr
    if (drop_tmr >= DROP_TIME)
      drop_speed = 0; // 3cc/hr
  }
  
  need_to_update = check_need_to_update(drop_speed);
  if (g_RingerData.drop_per_sec != drop_speed) 
  {
    g_RingerData.last_drop_per_sec = g_RingerData.drop_per_sec;
    g_RingerData.drop_per_sec = drop_speed;
  }
  if (g_RingerData.drop_per_sec > 10)
  {
    g_RingerData.drop_per_sec = 10;
  }
  old_rate = g_RingerData.drop_per_sec;
  //g_RingerData.ml_per_hour = old_rate * g_RingerData.r_adrop * 60 * 60; // ml / hr
  g_RingerData.ml_per_hour = old_rate * 60 * 3;
  g_RingerData.r_volume_now = g_RingerData.r_volume_max - (g_RingerData.drop_cnt * g_RingerData.r_adrop);

  g_RingerData.rest_min = g_RingerData.r_volume_now / g_RingerData.ml_per_hour * 60;
  uint16_t rest_hour = g_RingerData.rest_min / 60;
  if (rest_hour > 100) {
    time_rest = "--:--";  
  } else {
    time_rest = (rest_hour < 10 ? "0" : "") + String(rest_hour) + ':' +  (g_RingerData.rest_min % 60 < 10 ? "0" : "") + String(g_RingerData.rest_min % 60);
  }

  // time check
  uint16_t time_min = (tmr / 1000 / 60);
  uint16_t time_hour = time_min / 60;
  time_min = time_min % 60;
  time_progress = (time_hour < 10 ? "0" : "") + String(time_hour) + ':' +  (time_min < 10 ? "0" : "") + String(time_min);

  if (g_RingerData.r_volume_now <= 0.0f)
  {
    g_RingerData.r_volume_now = 0.0f;
    need_to_update = true;
  }
  return need_to_update;
}

void read_sn()
{
    byte mac[6];
    WiFi.macAddress(mac);
    Serial.print(mac[3],HEX);
    Serial.print(":");
    Serial.print(mac[4],HEX);
    Serial.print(":");
    Serial.println(mac[5],HEX);
    int num = int((unsigned char)(mac[3]) << 16 |
      (unsigned char)(mac[4]) << 8 |
      (unsigned char)(mac[5]));
    char new_serial[13] = "iRinger-1234";
    new_serial[8] = base64[(num >> 18) % 64];
    new_serial[9] = base64[(num >> 12) % 64];
    new_serial[10] = base64[(num >> 6) % 64];
    new_serial[11] = base64[num % 64];
    memcpy(serial_num, new_serial, 13);//new_serial 값 13 바이트를 serial_num 복사한다.
    Serial.println(serial_num);
    return;
 }






void STC3115_write(byte addr, byte d)
{
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(addr);
  Wire.write(d);
  Wire.endTransmission(true);
}

void STC3115_writeWord(byte addr, word d)
{
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(addr);
  Wire.write(d);
  Wire.write(d >> 8);
  Wire.endTransmission(true);
}

word STC3115_readWord(byte addr)
{
  byte error;
  word a, b, d;

  Wire.beginTransmission(I2C_ADDR);
  Wire.write(addr);
  error = Wire.endTransmission(true);
  Wire.requestFrom(I2C_ADDR, 2, true);
  a = Wire.read();
  b = Wire.read();
  d = b << 8 + a;
  ///printf("STC3115 read=0x%02X\n",d);
  return d;
}

void STC3115_init()
{
  word ocv;
  byte i;
  //  STC3115_get_id();
  STC3115_write(0, 0x09);
  STC3115_write(1, 0x05);
  /*while(STC3115_readWord(0x04)<3)
    {
    delay(300);
    }*/
  ocv = STC3115_readWord(0x0d);

  for (i = 0x30; i <= 0x3f; i++)
  {
    STC3115_write(i, 0);
  }
  STC3115_write(0x0f, 0x95);
  STC3115_write(0x10, 0x03);
  STC3115_writeWord(0x0d, ocv);
  STC3115_write(0, 0x18);
}

static int STC3115_conv(short value, unsigned short factor)
{
  int v;

  v = ((long)value * factor) >> 11;
  v = (v + 1) / 2;

  return (v);
}

void get_battery()  
{
  byte data[25], i, error;
  int value;
  word bat_cnt;
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(0);
  error = Wire.endTransmission(true);
  Wire.requestFrom(I2C_ADDR, 25, true);
  for (i = 0; i < 25; i++)
  {
    data[i] = Wire.read();
  }
  //printf("\n");
  value = data[5];
  value = (value << 8) + data[4];
    /* current */
  value = data[7];
  value = (value << 8) + data[6];
  value &= 0x3fff; /* mask unused bits */
  if (value >= 0x2000)
    value = value - 0x4000;                              /* convert to signed value */
  current = STC3115_conv(value, CurrentFactor / RSENSE); /* result in mA */
   /* voltage */
  value = data[9];
  value = (value << 8) + data[8];
  value &= 0x0fff; /* mask unused bits */
   if (value >= 0x0800)
    value -= 0x1000;                          /* convert to signed value */
  value = STC3115_conv(value, VoltageFactor); /* result in mV */
  if (value < 0)
    value = 0;
  voltage = value; /* result in mV */
  if (voltage > max_voltage)
    max_voltage = voltage;
  if (voltage > MAX_VOL)
    voltage = MAX_VOL;
  else if (voltage < MIN_VOL)
    voltage = MIN_VOL;
  g_RingerData.nBat = (voltage - MIN_VOL) * 100 / (MAX_VOL - MIN_VOL);

}

void STC3115_get_id()
{
  byte error;

  Wire.beginTransmission(I2C_ADDR);
  Wire.write(24);
  //error=Wire.endTransmission(false);
  error = Wire.endTransmission(true);
  printf("error=%u\n", error);
  //printf("Wire.endTransmission(false)=%u\n",Wire.endTransmission(false));//send and restart
  //  Wire.requestFrom(I2C_ADDR, 14, true);  // request a total of 14 registers
  Wire.requestFrom(I2C_ADDR, 1, true);
  printf("STC3115 ID=0x%X\n", Wire.read());
}

int post_graphql_query(char* body, char* query) {
  char graphql_url[HOST_URL_LENGTH + 8] = { 0, };
  sprintf(graphql_url, "%smonitoring", host_url);//23.1.17 수정
  
  http.begin(graphql_url);
  http.addHeader("Content-Type", "application/json");

  sprintf(body, "{ \"query\": \"%s\" }", query);

  printf("POST %s\n %s\n", graphql_url, body);
  return http.POST(body);
}


//  uint32_t new_drop_cnt;// for test

byte send_data() //sd
{
  Serial.println("Send data  함수를 실행 하였습니다.");
  byte x = 1;
  char query[200] = { 0,};
  char body[220] = { 0, };

  String test;

  if (g_RingerData.nBat == 0)
  {
    g_RingerData.drop_per_sec = 0;
  }
/*
  sprintf(
    query,
    "mutation { v1Monitoring ( sensing: { sn: \\\"%s\\\", injectedAmount: %f, gtt: %f, battery: %d, restMinute: %d } ) { r_volume_max, r_volume_now } }",
    serial_num, g_RingerData.drop_cnt * g_RingerData.r_adrop, g_RingerData.drop_per_sec * 60.0, g_RingerData.nBat, g_RingerData.rest_min
  );//Mutation: 저장된 데이터 수정하기 Create: 새로운 데이터 생성 Update: 기존의 데이터 수정 Delete: 기존의 데이터 삭제
 */
 g_RingerData.r_adrop=0.05;
     Serial.print("g_RingerData.drop_cnt:");
     Serial.println(g_RingerData.drop_cnt);
     Serial.print("g_RingerData.r_adrop :");
     Serial.println(g_RingerData.r_adrop);
//     new_drop_cnt=g_RingerData.drop_cnt-new_drop_cnt;// tjio 수정 
  sprintf(
    query,
    "{ \\\"v1Monitoring\\\" : { \\\"sn\\\": \\\"%s\\\", \\\"injectedAmount\\\": %f, \\\"gtt\\\": %f, \\\"battery\\\": %d, \\\"restMinute\\\": %d } }",
    serial_num, g_RingerData.drop_cnt * g_RingerData.r_adrop, g_RingerData.drop_per_sec * 60.0, g_RingerData.nBat, g_RingerData.rest_min
  );//Mutation: 저장된 데이터 수정하기 Create: 새로운 데이터 생성 Update: 기존의 데이터 수정 Delete: 기존의 데이터 삭제
 
 int httpCode = post_graphql_query(body, query);

  if (httpCode == 200)//400 잘못요청 401 권한없음,403 금지됨,404 찾을수 없음,500 서버오류/*  { 불가
  {
/*  {
    StaticJsonDocument<256>  doc;// tjio check
    String str = http.getString();//HTTP 요청의 응답 본문을 str 변수에 저장합니다.
    auto error = deserializeJson(doc, str);// 성공 ==0, 구문 분석 실패 -1 저장 
    if (error == DeserializationError::Ok) {//deserializeJson(doc, str);// str 즉 읽은값을 doc  스택에 집어넣는다.  
      String r_vol_max = doc["data"]["v1Monitoring"]["r_volume_max"];
      String r_vol_now = doc["data"]["v1Monitoring"]["r_volume_now"];
      printf("v1Monitoring volume max : %d\n", r_vol_max.toInt());
      g_RingerData.r_volume_max = r_vol_max.toInt();
      x = 1;
    } else {
      Serial.print("Error");
      Serial.print(str);
      x = 0;
 {    } 
  }*/
    server_connected=1;
    StaticJsonBuffer<200> jsonBuffer;
    String str = http.getString();
     Serial.print("수신  query :");
     Serial.println(str);
     Serial.print("수신  body  :");
     Serial.println(body);
    JsonObject &root = jsonBuffer.parseObject(str);
    if  (root.success() && !root["errors"].is<JsonObject>()) {
//      String r_vol_max = root["data"]["v1Monitoring"]["r_volume_max"];
//      String r_vol_now = root["data"]["v1Monitoring"]["r_volume__now"];
      String r_vol_max = root["v1Monitoring"]["r_volume_max"];//23.1.17 수정
      String r_vol_now = root["v1Monitoring"]["r_volume_now"];
      printf("v1Monitoring volume max : %d\n", r_vol_max.toInt());
      printf("v1Monitoring volume now : %f\n", r_vol_now.toFloat());
      
      if(r_vol_max.toInt()){
        g_RingerData.r_volume_max = r_vol_max.toInt();
      }
//      new_drop_cnt=g_RingerData.drop_cnt;//갱신
      x = 1;
    } else {
      Serial.print("Error");
      Serial.print(str);
      x = 0;
    } 
 }
  
  else
  {
//    new_drop_cnt=g_RingerData.drop_cnt;//복귀
    
    Serial.print("send_data error with code ");
    Serial.println(httpCode);
    
    x = 0;
  }
  http.end();
    Serial.println("Send data  함수를 종료  하였습니다.");
  return x;
}

byte get_info()
{
  Serial.println("get_info 함수를 실행 하였습니다.");
  byte x = 0;
  char query[200] = { 0, };// Your GraphQL query
  char body[220] = {0, };//Your body data for the POST request
 
//  sprintf(query, "query { v1Device(sn:\\\"%s\\\", battery: %d){ r_volume_max, r_volume_now, r_adrop, ordered_gtt, min_gtt, max_gtt }}", serial_num, g_RingerData.nBat);
// new_drop_cnt=g_RingerData.drop_cnt-new_drop_cnt;// tjio 수정 
  sprintf(query, "{ \\\"v1Monitoring\\\" : { \\\"sn\\\": \\\"%s\\\", \\\"injectedAmount\\\": %f, \\\"gtt\\\": %f, \\\"battery\\\": %d, \\\"restMinute\\\": %d } }",
    serial_num, g_RingerData.drop_cnt * g_RingerData.r_adrop, g_RingerData.drop_per_sec * 60.0, g_RingerData.nBat, g_RingerData.rest_min
  );//Mutation: 저장된 데이터 수정하기 Create: 새로운 데이터 생성 Update: 기존의 데이터 수정 Delete: 기존의 데이터 삭제
 
  
  int httpCode = post_graphql_query(body, query);//body 쿼리를 포함하는 문자열 배열 query는 쿼리이름 

     Serial.print("g_RingerData.drop_cnt : ");//TJIO 수정 
     Serial.println(g_RingerData.drop_cnt); 
     
     Serial.print("g_RingerData.drop_cnt*g_RingerData.r_adrop(injectedAmount)  : ");//TJIO 수정 
     Serial.println(g_RingerData.drop_cnt*g_RingerData.r_adrop); 
         
     Serial.print("GET_INFO()실행후:");//TJIO 수정 
     Serial.println(body);//TJIO 수정 
     
  if (httpCode == 200)
  {
    StaticJsonBuffer<200> jsonBuffer;
    String str = http.getString();
    JsonObject &root = jsonBuffer.parseObject(str);
    Serial.print("get_info rcv value:");
    Serial.println(str);
    if  (root.success() && !root["errors"].is<JsonObject>()) {
      String r_vol_max = root["v1Monitoring"]["r_volume_max"];
      String r_vol_now = root["v1Monitoring"]["r_volume_now"];
      
      printf("v1Monitoring volume max : %d\n", r_vol_max.toInt());
      
      if(r_vol_max.toInt()){
        g_RingerData.r_volume_max = r_vol_max.toInt();
      }
      
 //    new_drop_cnt=g_RingerData.drop_cnt;//갱신
     Serial.print("get_info save value:");
     Serial.println(g_RingerData.r_volume_max);     
      x = 1;
    } else {
      Serial.print("Error");

      x = 0;
    } 
  }
  else
  {
//    new_drop_cnt=g_RingerData.drop_cnt;//복귀
    
    Serial.print("HTTP Error ");
    Serial.println(httpCode);
    x = 0;
  }
  http.end();
  //print_data();

  if (x == 0) {
    Serial.println("Fail to get info");
  }
    Serial.println("get_info 함수를 종료 하였습니다.");
  return x;
}


void loop()
{
  if (server_connected) {
    SIGNAL_CHK = 2;//0 Wifi 연결 안됨 1 Wifi 연결됨 2 서버 연결 됨
  }
  
  if (digitalRead(usb_detect)) //  power_led by usb_detect
  {
    digitalWrite(led_power, HIGH);
  }
  else
  {
    digitalWrite(led_power, LOW);
  }

  if (wps_key.pressed == 1)
  {
    get_info();//r_vol_max  저장  tjio 수정
    wps_key.pressed = 0;
  }

  if (bat_init_flag)
  {
    bat_init_flag = 0;
    STC3115_init();
  }


  if (sec_flag) // 1초마다 실행
  {
    get_battery();

    
    bool is_alert_report = calc_drop();
    if (g_RingerData.drop_cnt < 50 && g_RingerData.drop_cnt % 5 == 0) {
      is_alert_report = true;
    }
    
    if (!g_RingerData.is_monitoring) {
      is_alert_report = false;
    }
    
    print_data();
    
    upload_counter++;// 0부터 증가 

    
    if (upload_counter > 29 || (is_alert_report && upload_counter > 4))//29
    {
      upload_counter = 0;
    }
    
    sec_flag = 0;
    
     if (upload_counter == 0 && server_connected)
//    if (upload_counter == 0)
    {
        
     Serial.print("loop에서 g_RingerData.drop_cnt : ");//TJIO 수정 
     Serial.println(g_RingerData.drop_cnt);
            

     if (send_data() == 1) {
         err_cnt = 0;
      } 
      else {
          err_cnt++;
          printf("err_cnt=%lu\n", err_cnt);
          if (err_cnt == 5)// 5번 전송 실패면 재접속 
          {
            err_cnt = 0;
            server_connected = false;
            thirty_counter = 0;
          }
        }
      }
      
       if(server_connected){
             if(!(upload_counter%5)){
                Serial.println("<--upload_counter-->");    
                Serial.print(upload_counter);
              }
              else Serial.print(upload_counter);
      }

    //30초마다 와이파이 체크   

    if (!server_connected && thirty_counter == 0)// 서버 연결이 끊어졌고 0초 일때마다 진행 
    {
      Serial.println("서버가 끊어져서 30초후 재접속 .");
      
      if ((wifiMulti.run() == WL_CONNECTED))
      {
        if (SIGNAL_CHK < 1)//0 Wifi 연결 안됨 1 Wifi 연결됨 2 서버 연결 됨
        {
          SIGNAL_CHK = 1;
          Serial.println("WiFi 가 연결 되었습니다.");
        }
          server_connected =send_data(); 
      }
      else // 연결 안된 경우
      {
        
        SIGNAL_CHK = 0;
        Serial.println("WiFi 가 끊어 졌습니다.");
        connect_ap(default_ap, default_pw);
    
      }  
    }   
    

    if (!server_connected && (thirty_counter < 30))//30 초 마다 와이파이 검색을 위해 
    {
      thirty_counter++;

      Serial.print("WiFi Wating : ");
      Serial.println(thirty_counter);
    }
    else
    {
      thirty_counter = 0;
    }

    
  } // set_flag
}
