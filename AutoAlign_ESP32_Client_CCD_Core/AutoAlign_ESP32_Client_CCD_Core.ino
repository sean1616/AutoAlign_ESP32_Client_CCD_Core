#include <EEPROM.h>
// #include <avr/wdt.h> //使用看門狗計時器的含括檔
// #include <esp_task_wdt.h>
// #include <FLASHLED.h>
// #include <ArduinoSort.h>
#include <curveFitting.h>
// #include <U8glib.h>
// #include <U8g2lib.h>
// #include <BluetoothSerial.h>
#include <WiFi.h>
#include <esp_now.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

#include <ESPAsyncWebServer.h>

TaskHandle_t Task_1;
#define WDT_TIMEOUT 3

// Set your access point network credentials
const char *ssid = "ESP32-Access-Point";
const char *password = "123456789";
String server_ID, server_Password;

bool isWiFiConnected = false;

String ID = "003";
String Station_ID = "A00";

const byte X_STP_Pin = 15; //x軸 步進控制
const byte X_DIR_Pin = 2;  //X軸 步進馬達方向控制
const byte Y_STP_Pin = 0;  //y軸 步進控制 0
const byte Y_DIR_Pin = 4;  //y軸 步進馬達方向控制 4
const byte Z_STP_Pin = 16; //z軸 步進控制
const byte Z_DIR_Pin = 17; //z軸 步進馬達方向控制

int ButtonSelected = 0;

int LCD_Encoder_A_pin = 35; //22
int LCD_Encoder_B_pin = 23; //23
uint8_t LCD_Select_pin = 34;  //21

bool LCD_Encoder_State = false;
bool LCD_Encoder_LastState = false;
int LCD_en_count = 0, idx = 0;
int LCD_sub_count = 0, idx_sub = 0;
int current_selection = 0;

int LCD_Update_Mode = 0;
uint8_t LCD_PageNow = 1;

const byte R_0 = 12;

/* Keyboard Pin Setting */
const byte R_1 = 14;
const byte R_2 = 27;
const byte R_3 = 26;
const byte C_1 = 25;
const byte C_2 = 33;
const byte C_3 = 32;

const byte PD_Pin = 34;
int Tablet_PD_mode_Trigger_Pin = 13;

Adafruit_ADS1115 ads;
TwoWire I2CADS = TwoWire(1);

#define I2C_SDA 21  //21
#define I2C_SCL 22  //22

const byte LED_Align = 5;

int MotorDir_Pin = 0;
int MotorSTP_Pin = 0;
bool MotorCC = false;
bool MotorCC_X = false;
bool MotorCC_Y = false;
bool MotorCC_Z = false;

double MotorStepRatio = 1;
int delayBetweenStep = 600;
int delayBetweenStep_X = 8;
int delayBetweenStep_Y = 8;
int delayBetweenStep_Z = 8;
int MinMotorDelayTime = 320;
long MinMotroStep = 20;
int M_Level = 10;

int xyz = 0;

long X_Pos_Record = 0;
long Y_Pos_Record = 0;
long Z_Pos_Record = 0;
long X_Pos_Now = 0;
long Y_Pos_Now = 0;
long Z_Pos_Now = 0;
long Z_Pos_reLoad = 0;

// int X_rotator_steps = 2;
// int Y_rotator_steps = 2;
// int Z_rotator_steps = 20;

int X_backlash = 0;
int Y_backlash = 0;
int Z_backlash = 0;

int X_ScanSTP = 12;
int Y_ScanSTP = 10;
int Z_ScanSTP = 200;

int X_ScanStable = 25;
int Y_ScanStable = 50;
int Z_ScanStable = 80;

//Intention _ Region _ MotionType _ ParaType _ Axis _ Rank = Value
int AA_SpiralRough_Feed_Steps_Z_A = 25000;
int AA_SpiralRough_Spiral_Steps_XY_A = 2000;
int AA_SpiralFine_Spiral_Steps_XY_A = 1500;
int AA_SpiralFine_Scan_Steps_X_A = 25;
int AA_SpiralFine_Scan_Steps_X_B = 30;
int AA_SpiralFine_Scan_Steps_X_C = 40;
int AA_SpiralFine_Scan_Steps_X_D = 50;
int AA_SpiralFine_Scan_Steps_Y_A = 30;
int AA_SpiralFine_Scan_Steps_Y_B = 40;
int AA_SpiralFine_Scan_Steps_Y_C = 60;
int AA_SpiralFine_Scan_Steps_Y_D = 80;
int AA_SpiralFine_Scan_Steps_Y_E = 140;
int AA_ScanRough_Feed_Steps_Z_A = 10000;
int AA_ScanRough_Feed_Steps_Z_B = 1000;
double AA_ScanRough_Feed_Ratio_Z_A = 2.8;
double AA_ScanRough_Feed_Ratio_Z_B = 2.5;
double AA_ScanRough_Feed_Ratio_Z_C = 2.0;
double AA_ScanRough_Feed_Ratio_Z_D = 1.5;
int AA_ScanRough_Scan_Steps_Y_A = 25;
int AA_ScanRough_Scan_Steps_Y_B = 30;
int AA_ScanRough_Scan_Steps_Y_C = 40;
int AA_ScanRough_Scan_Steps_Y_D = 70;
int AA_ScanRough_Scan_Steps_X_A = 25;
int AA_ScanRough_Scan_Steps_X_B = 30;
int AA_ScanRough_Scan_Steps_X_C = 80;
int AA_ScanRough_Scan_Steps_X_D = 100;
int AA_ScanRough_Scan_Steps_X_E = 120;
int AA_ScanFine_Scan_Steps_Z_A = 200;
int AA_ScanFine_Scan_Steps_Y_A = 20;
int AA_ScanFine_Scan_Steps_X_A = 20;
int AA_ScanFinal_Scan_Steps_Z_A = 125;
int AA_ScanFinal_Scan_Steps_Y_A = 20;
int AA_ScanFinal_Scan_Steps_X_A = 20;

int AQ_Scan_Compensation_Steps_Z_A = 12;
int AQ_Scan_Steps_Z_A = 40;  //125, 30
int AQ_Scan_Steps_Z_B = 40;  //120, 30
int AQ_Scan_Steps_Z_C = 45;   //70, 35
int AQ_Scan_Steps_Z_D = 50;   //50, 50

int AA_ScanFinal_Scan_Delay_X_A = 100;
int AA_ScanFinal_Scan_Delay_Y_A = 60;

int AQ_Total_TimeSpan = 840;

uint16_t FS_Count_X = 7;
uint16_t FS_Steps_X = 25;
uint16_t FS_Stable_X = 0;
uint16_t FS_DelaySteps_X = 50;
uint16_t FS_Avg_X = 600;
uint16_t FS_Count_Y = 8;
uint16_t FS_Steps_Y = 20;
uint16_t FS_Stable_Y = 0;
uint16_t FS_DelaySteps_Y = 120;
uint16_t FS_Avg_Y = 600;
uint16_t FS_Count_Z = 7;
uint16_t FS_Steps_Z = 80;
uint16_t FS_Stable_Z = 0;
uint16_t FS_DelaySteps_Z = 80;
uint16_t FS_Avg_Z = 800;
uint16_t FS_Trips_X = 0;
uint16_t FS_Trips_Y = 0;
uint16_t FS_Trips_Z = 2;

uint16_t EP_PD_Ref = 0;
uint16_t EP_Board_ID = 8;
uint16_t EP_Station_ID = 16;
uint16_t EP_X_backlash = 24;
uint16_t EP_Y_backlash = 32;
uint16_t EP_Z_backlash = 40;
uint16_t EP_delayBetweenStep_X = 48;
uint16_t EP_delayBetweenStep_Y = 56;
uint16_t EP_delayBetweenStep_Z = 64;
uint16_t EP_Target_IL = 72;
uint16_t EP_AA_ScanFinal_Scan_Delay_X_A = 80;
uint16_t EP_Server_ID = 88;  //88~119
uint16_t EP_Server_Password = 120;  //120~151
uint16_t EP_AQ_Scan_Compensation_Steps_Z_A = 160;
uint16_t EP_AQ_Total_TimeSpan = 168;
uint16_t EP_AQ_Scan_Steps_Z_A = 176;
uint16_t EP_AQ_Scan_Steps_Z_B = 184;
uint16_t EP_AQ_Scan_Steps_Z_C = 192;
uint16_t EP_AQ_Scan_Steps_Z_D = 200;
uint16_t EP_FS_Count_X = 240;
uint16_t EP_FS_Steps_X = 248;
uint16_t EP_FS_Stable_X = 256;
uint16_t EP_FS_DelaySteps_X = 264;
uint16_t EP_FS_Avg_X = 272;
uint16_t EP_FS_Count_Y = 280;
uint16_t EP_FS_Steps_Y = 288;
uint16_t EP_FS_Stable_Y = 296;
uint16_t EP_FS_DelaySteps_Y = 304;
uint16_t EP_FS_Avg_Y = 312;
uint16_t EP_FS_Count_Z = 320;
uint16_t EP_FS_Steps_Z = 328;
uint16_t EP_FS_Stable_Z = 336;
uint16_t EP_FS_DelaySteps_Z = 344;
uint16_t EP_FS_Avg_Z = 352;
uint16_t EP_FS_Trips_X = 360;
uint16_t EP_FS_Trips_Y = 368;
uint16_t EP_FS_Trips_Z = 376;

int PD_Ref_Array[15][2] = 
  {
    {24260, -3},
    {23644, -4},
    {23282, -5},
    {22571, -7},
    {21910, -9},
    {21182, -11},
    {20525, -13},
    {19854, -15},
    {19127, -17},
    {18460, -19},
    {17739, -21},
    {16354, -25},
    {14663, -30},
    {11299, -40},
    {8903, -50},
  };


double averagePDInput = 0;

double ref_Dac = 0; //PD reference
double ref_IL = 0;  //PD reference

double PDValue_Best = 0;
double AutoCuring_Best_IL = 0, PD_Now = 0, PD_Before = 0;

unsigned long time_curing_0, time_curing_1, time_curing_2, time_curing_3;
unsigned long timer_Get_IL_1 = 0, timer_Get_IL_2;

bool btn_isTrigger = false;
int Threshold;
int stableDelay = 0;
bool key_ctrl = false;

double Motor_Unit_Idx = 0.01953125; /* (1/51.2) um/pulse */

int Get_PD_Points = 1;
double Target_IL = 0; //0 dB
double StopValue = 0; //0 dB
int cmd_No = 0;

bool isStop = false, isGetPower = true, isILStable = false;
bool sprial_JumpToBest = true;
int Q_State = 0;
unsigned long Q_Time = 0;
unsigned long LCD_Auto_Update_TimeCount = 0;
byte GetPower_Mode = 1;
bool is_Scan_V2_ReWork = false;

bool isWatchDog_Flag = false;
bool isLCD = true;
bool isLCD_Auto_Update = false;

void Task_1_sendData(void *pvParameters)
{
  while (true)
  {
    if (!digitalRead(LCD_Select_pin))
    {
      LCD_Encoder_Selected();
    }

    int idx = LCD_en_count / 2;
    // Serial.println(String(idx));
    updateUI(idx);

    if (isLCD_Auto_Update)
      if (millis() - LCD_Auto_Update_TimeCount > 5000)
      {
        LCD_Auto_Update_TimeCount = millis();
        isLCD = true;
      }

    delay(150);
    // lcd.clearDisplay();
    // delay(150);

    //Task1休息，delay(1)不可省略
    delay(1);
  }
}

void step(byte stepperPin, long steps, int delayTime)
{
  steps = abs(steps);

  for (long i = 0; i < steps; i++)
  {
    digitalWrite(stepperPin, HIGH);
    delayMicroseconds(delayTime);
    digitalWrite(stepperPin, LOW);
    delayMicroseconds(delayTime);
  }

  //Position Record
  if (MotorCC)
  {
    switch (stepperPin)
    {
    case X_STP_Pin:
      X_Pos_Now += steps;
      MotorCC_X = true;
      break;
    case Y_STP_Pin:
      Y_Pos_Now += steps;
      MotorCC_Y = true;
      break;
    case Z_STP_Pin:
      Z_Pos_Now += steps;
      MotorCC_Z = true;
      break;
    }
  }
  else
  {
    switch (stepperPin)
    {
    case X_STP_Pin:
      X_Pos_Now -= steps;
      MotorCC_X = false;
      break;
    case Y_STP_Pin:
      Y_Pos_Now -= steps;
      MotorCC_Y = false;
      break;
    case Z_STP_Pin:
      Z_Pos_Now -= steps;
      MotorCC_Z = false;
      break;
    }
  }
}

void step(byte stepperPin, long steps, int delayTime, byte dirPin, bool dir)
{
  // steps = abs(steps);
  digitalWrite(dirPin, dir);

  for (long i = 0; i < steps; i++)
  {
    digitalWrite(stepperPin, HIGH);
    delayMicroseconds(delayTime);
    digitalWrite(stepperPin, LOW);
    delayMicroseconds(delayTime);
  }

  //Position Record
  if (MotorCC == true)
  {
    switch (stepperPin)
    {
    case X_STP_Pin:
      X_Pos_Now += steps;
      MotorCC_X = true;
      break;
    case Y_STP_Pin:
      Y_Pos_Now += steps;
      MotorCC_Y = true;
      break;
    case Z_STP_Pin:
      Z_Pos_Now += steps;
      MotorCC_Z = true;
      break;
    }
  }
  else
  {
    switch (stepperPin)
    {
    case X_STP_Pin:
      X_Pos_Now -= steps;
      MotorCC_X = false;
      break;
    case Y_STP_Pin:
      Y_Pos_Now -= steps;
      MotorCC_Y = false;
      break;
    case Z_STP_Pin:
      Z_Pos_Now -= steps;
      MotorCC_Z = false;
      break;
    }
  }
}

int KeyValueConverter()
{
  int keyNo = -1;
  bool isKeyPressed = false;
  int keyValueSum = 0;

  if (!digitalRead(R_1))
  {
    isKeyPressed = true;
    keyValueSum += 10;
  }
  else if (!digitalRead(R_2))
  {
    isKeyPressed = true;
    keyValueSum += 5;
  }
  else if (!digitalRead(R_3))
  {
    isKeyPressed = true;
    keyValueSum += 0;
  }

  if (isKeyPressed)
  {
    pinMode(C_1, INPUT_PULLUP);
    pinMode(C_2, INPUT_PULLUP);
    pinMode(C_3, INPUT_PULLUP);

    pinMode(R_1, OUTPUT);
    pinMode(R_2, OUTPUT);
    pinMode(R_3, OUTPUT);

    delay(2);

    if (!digitalRead(C_1))
    {
      keyValueSum += 1;
    }
    else if (!digitalRead(C_2))
      keyValueSum += 2;
    else if (!digitalRead(C_3))
      keyValueSum += 3;
    else
      keyValueSum = 0;

    pinMode(R_1, INPUT_PULLUP);
    pinMode(R_2, INPUT_PULLUP);
    pinMode(R_3, INPUT_PULLUP);

    pinMode(C_1, OUTPUT);
    pinMode(C_2, OUTPUT);
    pinMode(C_3, OUTPUT);

    delay(2);
  }

  if (keyValueSum != 0)
  {

    switch (keyValueSum)
    {
    case 1:
      keyNo = 101; /* Z- */
      break;
    case 2:
      keyNo = 102; /* X+ */
      break;
    case 3:
      keyNo = 103; /* Z+ */
      break;
    case 6:
      keyNo = 104; /* Y+ */
      break;
    case 7:
      keyNo = 105; /* X- */
      break;
    case 8:
      keyNo = 106; /* Y- */
      break;
    case 11:
      keyNo = 7;
      break;
    case 12:
      keyNo = 8;
      break;
    case 13:
      keyNo = 9;
      break;
    default:
      keyNo = -1;
      break;
    }

    isKeyPressed = false;
  }

  return keyNo;
}

double a1 = 0.0374, a2 = -65.561;
double b1 = 0.0394, b2 = -67.778;

//Dac to dBm
double ILConverter(double pdDac)
{
  double IL = 0;

  if (pdDac >= 1200) /* >20 dBm */
    IL = a1 * pdDac + a2;
  else
    IL = b1 * pdDac + b2;

  // if(pdDac >= PD_Ref_Array[0][0])
  //   return -3;
  // else if (pdDac < PD_Ref_Array[14][0])
  //   return -50;

  // for (size_t i = 1; i < 15; i++)
  // {        
  //   if(pdDac >= PD_Ref_Array[i][0])
  //   {
  //     IL = ((pdDac - PD_Ref_Array[i][0])/(PD_Ref_Array[i-1][0] - PD_Ref_Array[i][0]) * (PD_Ref_Array[i - 1][1] - PD_Ref_Array[i][1])) + PD_Ref_Array[i][1];
  //     break;
  //   }
  // }
  
  return IL;
}

//Calculate PD input value, Return Dac
double Cal_PD_Input_Dac(int averageCount)
{
  digitalWrite(X_DIR_Pin, false);
  digitalWrite(Y_DIR_Pin, false);
  digitalWrite(Z_DIR_Pin, false);
  delay(1);

  double averagePDInput = 0;
  double PDAvgInput = 0;
  for (int i = 0; i < averageCount; i++)
  {
    PDAvgInput += analogRead(PD_Pin);
    // PDAvgInput += ads.readADC_SingleEnded(0);
  }

  //Function: (PD Value) - (reference) + 300
  averagePDInput = (PDAvgInput / averageCount);

  digitalWrite(X_DIR_Pin, MotorCC_X);
  digitalWrite(Y_DIR_Pin, MotorCC_Y);
  digitalWrite(Z_DIR_Pin, MotorCC_Z);
  delay(1);

  return (averagePDInput - ref_IL);
}

//Calculate PD input value, Return IL
double Cal_PD_Input_IL(int averageCount)
{
  digitalWrite(X_DIR_Pin, false);
  digitalWrite(Y_DIR_Pin, false);
  digitalWrite(Z_DIR_Pin, false);
  delay(1);

  double averagePDInput = 0;
  double PDAvgInput = 0;

  for (int i = 0; i < averageCount; i++)
  {
    PDAvgInput += analogRead(PD_Pin);
    // PDAvgInput += ads.readADC_SingleEnded(0);
  }

  averagePDInput = (PDAvgInput / averageCount);

  digitalWrite(X_DIR_Pin, MotorCC_X);
  digitalWrite(Y_DIR_Pin, MotorCC_Y);
  digitalWrite(Z_DIR_Pin, MotorCC_Z);
  delay(1);

  double IL = ILConverter(averagePDInput) - ref_IL;

  return IL;
}

//Calculate PD input value, Return Row Dac
double Cal_PD_Input_Row_IL(int averageCount)
{
  digitalWrite(X_DIR_Pin, false);
  digitalWrite(Y_DIR_Pin, false);
  digitalWrite(Z_DIR_Pin, false);
  delay(1);

  double averagePDInput = 0;
  double PDAvgInput = 0;
  for (int i = 0; i < averageCount; i++)
  {
    PDAvgInput += analogRead(PD_Pin);
    // PDAvgInput += ads.readADC_SingleEnded(0);
  }
  //Function: (PD Value)
  averagePDInput = (PDAvgInput / averageCount);

  digitalWrite(X_DIR_Pin, MotorCC_X);
  digitalWrite(Y_DIR_Pin, MotorCC_Y);
  digitalWrite(Z_DIR_Pin, MotorCC_Z);
  delay(1);

  double IL = ILConverter(averagePDInput);

  return IL;
}

//Calculate PD input value, Return Row Dac
double Cal_PD_Input_Row_Dac(int averageCount)
{
  digitalWrite(X_DIR_Pin, false);
  digitalWrite(Y_DIR_Pin, false);
  digitalWrite(Z_DIR_Pin, false);
  delay(1);

  double averagePDInput = 0;
  double PDAvgInput = 0;
  for (int i = 0; i < averageCount; i++)
  {
    PDAvgInput += analogRead(PD_Pin);
    // PDAvgInput += ads.readADC_SingleEnded(0);
  }
  averagePDInput = (PDAvgInput / averageCount);

  digitalWrite(X_DIR_Pin, MotorCC_X);
  digitalWrite(Y_DIR_Pin, MotorCC_Y);
  digitalWrite(Z_DIR_Pin, MotorCC_Z);
  delay(1);

  return averagePDInput;
}

void CleanEEPROM(int startPosition, int datalength)
{
  for (size_t i = startPosition; i < (startPosition + datalength); i++)
  {
    EEPROM.write(i, ' ');
  }
  Serial.println("Clean EEPROM");
}

void WriteInfoEEPROM(String data, int start_position)
{
  for (int i = 0; i < data.length(); ++i)
  {
    EEPROM.write(i + start_position, data[i]);
  }
}

String ReadInfoEEPROM(int start_position, int data_length)
{
  String EEPROM_String = "";
  for (int i = 0; i < data_length; i++)
  {
    uint8_t a = EEPROM.read(i + start_position);
    if (a != 255)
      EEPROM_String += char(EEPROM.read(i + start_position));
  }
  EEPROM_String.trim();
  return EEPROM_String;
}

String WR_EEPROM(int start_position, String data)
{
  CleanEEPROM(start_position, 8); //Clean EEPROM(int startPosition, int datalength)

  WriteInfoEEPROM(String(data), start_position); //Write Data to EEPROM (data, start_position)
  EEPROM.commit();

  String s = ReadInfoEEPROM(start_position, 8);
  return s;
}

String WR_EEPROM(int start_position, int data_length, String data)
{
  CleanEEPROM(start_position, data_length); //Clean EEPROM(int startPosition, int datalength)

  WriteInfoEEPROM(String(data), start_position); //Write Data to EEPROM (data, start_position)
  EEPROM.commit();

  String s = ReadInfoEEPROM(start_position, data_length);
  return s;
}

bool Contains(String text, String search)
{
  if (text.indexOf(search) == -1)
    return false;
  else
    return true;
}


#define ITEMS_COUNT 100
char *UI_Items[ITEMS_COUNT] =
    {" "};

#define MENU_ITEMS 6
char *UI_Menu_Items[MENU_ITEMS] =
    {"1. Status",
     "2. Target IL",
     "3. StableDelay",
     "4. Q Z-offset",
     "5. Speed",
     "6. Get Ref"};

#define Speed_Page_ITEMS 4
char *UI_Speed_Page_Items[MENU_ITEMS] =
    {"1. X Speed",
     "2. Y Speed",
     "3. Z Speed",
     "<<"};

uint8_t i, h, w, title_h, H;

int PageLevel = 0;
int PageItemsCount = 1;

int Top_Item_Index = 0;
int Bottom_Item_Index = 3;
bool ui_YesNo_Selection = false;

int mainpageIndex = 0;

int subpageIndex = 0;
int subpage_itemsCount = 1;
bool item_is_selected = false;
bool plus_minus = false;

//Full Page method
void updateUI(int pageIndex)
{

}

void Draw_ALL_UI_Items(int LCD_Update_Mode, int pageIndex)
{

}

int pre_LCD_Page_index = 0;
void LCD_Encoder_Rise()
{
  
}

void LCD_Encoder_Selected()
{

}

void EmergencyStop()
{
  isStop = true;

  Serial.println("EmergencyStop");
  digitalWrite(Tablet_PD_mode_Trigger_Pin, true); //false is PD mode, true is Servo mode

  isLCD = true;
  PageLevel = 0;
}
//------------------------------------------------------------------------------------------------------------------------------------------
String httpGETRequest(const char *serverName)
{
  WiFiClient client;
  HTTPClient http;

  // Your Domain name with URL path or IP address with path
  http.begin(client, serverName);

  // Send HTTP POST request
  int httpResponseCode = http.GET();

  String payload = "--";

  if (httpResponseCode > 0)
  {
    // Serial.print("HTTP Response code: ");
    // Serial.println(httpResponseCode);
    payload = http.getString();
  }
  else
  {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  // Free resources
  http.end();

  return payload;
}
//------------------------------------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------------------------------------

//ESP-Now Receive data function
// void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len)
// {
//   memcpy(&hallVal, incomingData, sizeof(hallVal));
//   Serial.print("hallVal: ");
//   Serial.println(hallVal);
// }

uint8_t broadcastAddress[] = {0x8C, 0x4B, 0x14, 0x16, 0x65, 0xFC}; 

String Msg;

typedef struct struct_send_message {
    String msg;
} struct_send_message;

typedef struct struct_message {
    String contr_name;
    String msg;
} struct_message;

// Create a struct_message called BME280Readings to hold sensor readings
struct_send_message sendmsg;

// Create a struct_message to hold incoming sensor readings
struct_message incomingReadings;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  status == ESP_NOW_SEND_SUCCESS ;
  //
  if(status == 0)
  {
    // Serial.println("OK");
  }
}

String cmd_from_contr = "";

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  incomingReadings.msg = "";
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  // Serial.print("Bytes received: ");
  // Serial.println(len);
  Msg = incomingReadings.msg;
  Serial.println(incomingReadings.contr_name + Msg);

  if(Msg != "")
  {
    cmd_from_contr = Msg;
  }

  sendmsg.msg = "Get Data!";
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &sendmsg, sizeof(sendmsg));
}

//------------------------------------------------------------------------------------------------------------------------------------------

void setup()
{
  Serial.begin(115200);
  Serial.setTimeout(20); //設定序列埠接收資料時的最大等待時間

  //宣告使用EEPROM 512 個位置
  EEPROM.begin(512);
  
  #pragma region WiFi Server Setting
  
  server_ID = ReadInfoEEPROM(88, 32);
  server_Password = ReadInfoEEPROM(120, 32);

  if (Contains(server_ID, "??") || server_ID == "")
  {
    server_ID = "GFI-ESP32-Access-Point";
  }

  if (Contains(server_Password, "??"))
  {
    server_Password = "22101782";
  }

  Serial.println("Server ID: " + server_ID);
  Serial.println("Server Password: " + server_Password);

  // 取得本機的MACAddress
  // WiFi.mode(WIFI_MODE_STA);
  // Serial.print("ESP32 Board MAC Address:  ");
  // Serial.println(WiFi.macAddress());

  // 初始化 ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  else
    Serial.println("Initializing ESP-NOW");

  // 設置發送數據回傳函數
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // 绑定數據接收端
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6); // Register peer
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }


  // WiFi.begin(server_ID.c_str(), server_Password.c_str());
  // // WiFi.begin(ssid, password);
  // Serial.println("Connecting");

  // int wifiConnectTime = 0;
  // while (WiFi.status() != WL_CONNECTED)
  // {
  //   delay(300);
  //   Serial.print(".");

  //   wifiConnectTime += 300;
  //   if (wifiConnectTime > 2400)
  //     break;
  // }

  // if (wifiConnectTime <= 2400)
  // {
  //   Serial.println("");
  //   Serial.print("Connected to WiFi network with IP Address:");
  //   Serial.println(WiFi.localIP());
  //   isWiFiConnected = true;
  // }
  // else
  // {
  //   Serial.println("Connected to WiFi network failed");
  // }

  #pragma endregion

#pragma region pinMode Setting

  pinMode(LED_Align, OUTPUT);
  pinMode(X_STP_Pin, OUTPUT);
  pinMode(X_DIR_Pin, OUTPUT);
  pinMode(Y_STP_Pin, OUTPUT);
  pinMode(Y_DIR_Pin, OUTPUT);
  pinMode(Z_STP_Pin, OUTPUT);
  pinMode(Z_DIR_Pin, OUTPUT);

  pinMode(R_0, INPUT_PULLUP);
  attachInterrupt(R_0, EmergencyStop, FALLING);
  // keyValueConverter()
    
  pinMode(R_2, INPUT_PULLUP); // /keyValue:5
  pinMode(R_3, INPUT_PULLUP); // /keyValue:0

  pinMode(C_1, OUTPUT); ///keyValue:1
  pinMode(C_2, OUTPUT); ///keyValue:2
  pinMode(C_3, OUTPUT); ///keyValue:3

  // pinMode(LCD_Encoder_A_pin, INPUT_PULLUP);                     // /keyValue:0
  // pinMode(LCD_Encoder_B_pin, INPUT_PULLUP);                     // /keyValue:0
  // pinMode(LCD_Select_pin, INPUT_PULLUP);                        //Encoder switch
  // attachInterrupt(LCD_Encoder_A_pin, LCD_Encoder_Rise, CHANGE); //啟用中斷函式(中斷0，函式，模式)
  // attachInterrupt(LCD_Select_pin, LCD_Encoder_Selected, FALLING); //啟用中斷函式(中斷0，函式，模式)

  digitalWrite(C_1, false);
  digitalWrite(C_2, false);
  digitalWrite(C_3, false);

#pragma endregion

  Serial.println("~~ Auto-Align System - CCD ~~");

 #pragma region EEPROM Setting
  String eepromString;

  for (int i = 0; i < 511; i = i + 8)
  {
    eepromString = ReadInfoEEPROM(i, 8); //Reading EEPROM(int start_position, int data_length)
    MSGOutput("EEPROM(" + String(i) + ") - " + eepromString);
  }

  // FS_Trips_Z = ReadInfoEEPROM(EP_FS_Trips_Z, 8).toInt();
  // MSGOutput("FS_Trips_Z: " + String(FS_Trips_Z));

  #pragma endregion
  
  // timer_Get_IL_1 = millis();

  // timer_Get_IL_2 = millis();
 
  // Serial.println("Timespan of Get PD IL:" + String(timer_Get_IL_2 - timer_Get_IL_1));
}

bool isMsgShow = false;
unsigned long previousMillis = 0;
const long interval = 2000;
String Data;

void loop()
{
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval)
  {    
    //Call ESP-Now receive data function
    // esp_now_register_recv_cb(OnDataRecv);

    //Re-Initialize
    isStop = false;
    ButtonSelected = -1;

    //Keyboard Detect
    // ButtonSelected = KeyValueConverter();

    String rsMsg = "";
    if (Serial.available())
      rsMsg = Serial.readString();

    String cmd = rsMsg;

    // if(cmd_from_contr != "")
    // {
    //   cmd = cmd_from_contr;
    // }

    cmd_No = Function_Classification(cmd, ButtonSelected);

    // BLE_Function(cmd);

    cmd_No = Function_Excecutation(cmd, cmd_No);

    // cmd_from_contr = "";
  }
}

//------------------------------------------------------------------------------------------------------------------------------------------

void Move_Motor_abs(int xyz, long Target)
{
  String axis = "";
  long Pos_Now = 0;
  switch (xyz)
  {
  case 0:
    MotorDir_Pin = X_DIR_Pin;
    MotorSTP_Pin = X_STP_Pin;
    Pos_Now = X_Pos_Now;
    axis = "X";
    break;
  case 1:
    MotorDir_Pin = Y_DIR_Pin;
    MotorSTP_Pin = Y_STP_Pin;
    Pos_Now = Y_Pos_Now;
    axis = "Y";
    break;
  case 2:
    MotorDir_Pin = Z_DIR_Pin;
    MotorSTP_Pin = Z_STP_Pin;
    Pos_Now = Z_Pos_Now;
    axis = "Z";
    break;
  }

  if (Target - Pos_Now < 0)
    MotorCC = false;
  else if (Target - Pos_Now > 0)
    MotorCC = true;
  else
    return;

  delayBetweenStep = 20;
  MinMotroStep = abs(Target - Pos_Now);

  MSGOutput(axis + " Go to position: " + String(Target) + ", origin position: " + String(Pos_Now));

  //byte dir_pin, byte stp_pin, bool dirt, long moveSteps, int delayStep, int stableDelay
  Move_Motor(MotorDir_Pin, MotorSTP_Pin, MotorCC, MinMotroStep, delayBetweenStep, 0);
}

void Move_Motor_abs_all(int x, int y, int z)
{
  Move_Motor_abs(0, x);
  Move_Motor_abs(1, y);
  Move_Motor_abs(2, z);
}

void Move_Motor(byte dir_pin, byte stp_pin, bool dirt, long moveSteps, int delayStep, int stableDelay)
{
  MotorCC = dirt;
  digitalWrite(dir_pin, dirt);
  delay(2);
  if (moveSteps > 0)
  {
    step(stp_pin, moveSteps, delayStep);
    delay(stableDelay);
    DataOutput(false);
  }
}

void Move_Motor(byte dir_pin, byte stp_pin, bool dirt, long moveSteps, int delayStep, int stableDelay, bool isOutputPosition)
{
  MotorCC = dirt;
  digitalWrite(dir_pin, dirt);
  delay(5);
  if (moveSteps > 0)
  {
    step(stp_pin, moveSteps, delayStep);
    delay(stableDelay);

    if (isOutputPosition)
      DataOutput(false);
  }
}

void Move_Motor_Cont(byte dir_pin, byte stp_pin, bool dirt, long moveSteps, int delayStep)
{
  MotorSTP_Pin = dir_pin;

  if (MotorDir_Pin != dir_pin || MotorCC != dirt)
  {
    MotorCC = dirt;
    MotorDir_Pin = dir_pin;
    digitalWrite(MotorDir_Pin, MotorCC); //步進馬達方向控制, false為負方向
    delay(3);
  }

  step(stp_pin, moveSteps, delayStep);
}

//------------------------------------------------------------------------------------------------------------------------------------------

String Region, msg;
bool Fine_Scan(int axis, bool Trip2Stop)
{
  isLCD = true;
  LCD_Update_Mode = 1;

  MSGOutput("");
  MSGOutput("Fine Scan ");

  MSGOutput("Stop Value: " + String(StopValue));

  digitalWrite(Tablet_PD_mode_Trigger_Pin, false); //false is PD mode, true is Servo mode
  delay(5);

  Threshold = -37.3;
  // delayBetweenStep = 100;

  double pdBest = Cal_PD_Input_IL(Get_PD_Points);

  // Region = Region + "_Fine_Scan";
  String msg;

  bool K_OK = true;

  if (axis < 4)
  {
    switch (axis)
    {
    case 1:

      PD_Now = Cal_PD_Input_IL(Get_PD_Points);

      MotorCC_X = digitalRead(X_DIR_Pin);

      CMDOutput("AS");
      // K_OK = Scan_AllRange_TwoWay(0, 8, 22, 0, 0, 120, StopValue, 500, 2, "X Scan,Trip_");
      // K_OK = Scan_AllRange_TwoWay(0, 7, 25, stableDelay, 0, 50, StopValue, 600, 2, "X Scan,Trip_");
      K_OK = Scan_AllRange_TwoWay(0, FS_Count_X, FS_Steps_X * MotorStepRatio, FS_Stable_X, MotorCC_X, FS_DelaySteps_X, StopValue, FS_Avg_X, FS_Trips_X, "X Scan,Trip_");
      CMDOutput("%:");

      if (!K_OK)
      {
        MotorCC_X = digitalRead(X_DIR_Pin);

        CMDOutput("AS");
        // Scan_AllRange_TwoWay(0, 7, 25, stableDelay, 0, 50, StopValue, 600, 2, "X Re-Scan,Trip_");
        Scan_AllRange_TwoWay(0, FS_Count_X, FS_Steps_X * MotorStepRatio, FS_Stable_X, MotorCC_X, FS_DelaySteps_X, StopValue, FS_Avg_X, FS_Trips_X, "X Re-Scan,Trip_");
        CMDOutput("%:");
      }

      break;

    case 2:

      PD_Now = Cal_PD_Input_IL(2 * Get_PD_Points);

      MotorCC_Y = digitalRead(Y_DIR_Pin);

      CMDOutput("AS");
      // Scan_AllRange_TwoWay(1, 6, 35, AA_ScanFinal_Scan_Delay_Y_A, 0, 100, StopValue, 600, 2, "Y Scan, Trip_");
      // K_OK = Scan_AllRange_TwoWay(1, 8, 20, 0, 0, 120, StopValue, 600, 2, "Y Scan,Trip_"); 
      K_OK = Scan_AllRange_TwoWay(1, FS_Count_Y, FS_Steps_Y * MotorStepRatio, FS_Stable_Y, MotorCC_Y, FS_DelaySteps_Y, StopValue, FS_Avg_Y, FS_Trips_Y, "Y Scan,Trip_"); 
      CMDOutput("%:");

      if (!K_OK)
      {
        MotorCC_Y = digitalRead(Y_DIR_Pin);

        CMDOutput("AS");
        // Scan_AllRange_TwoWay(1, 8, 20, 0, 0, 120, StopValue, 600, 2, "Y Re-Scan,Trip_");
        Scan_AllRange_TwoWay(1, FS_Count_Y, FS_Steps_Y * MotorStepRatio, FS_Stable_Y, MotorCC_Y, FS_DelaySteps_Y, StopValue, FS_Avg_Y, FS_Trips_Y, "Y Re-Scan,Trip_"); 
        CMDOutput("%:");
      }

      break;

    case 3:

      PD_Now = Cal_PD_Input_IL(2 * Get_PD_Points);

      // MotorCC_Z = digitalRead(Z_DIR_Pin);

      CMDOutput("AS");
      // Scan_AllRange_TwoWay(2, 6, 100, AA_ScanFinal_Scan_Delay_X_A, 0, 80, StopValue, 600, 2, "Z Scan, Trip_"); //--Z--
      // Scan_AllRange_TwoWay(2, 8, 125, 0, 0, 100, StopValue, 600, 2, "Z_Scan,Trip_");
      // K_OK = Scan_AllRange_TwoWay(2, 7, 80, 0, 0, 80, StopValue, 800, 2, "Z Scan,Trip_");
      K_OK = Scan_AllRange_TwoWay(2, FS_Count_Z, FS_Steps_Z * MotorStepRatio, FS_Stable_Z, 0, FS_DelaySteps_Z, StopValue, FS_Avg_Z, FS_Trips_Z, "Z Scan,Trip_");
      CMDOutput("%:");

      if (!K_OK)
      {
        // MotorCC_Z = digitalRead(Z_DIR_Pin);

        CMDOutput("AS");
        // Scan_AllRange_TwoWay(2, 7, 80, 0, 0, 80, StopValue, 800, 2, "Z Re-Scan,Trip_");
        Scan_AllRange_TwoWay(2, FS_Count_Z, FS_Steps_Z * MotorStepRatio, FS_Stable_Z, 0, FS_DelaySteps_Z, StopValue, FS_Avg_Z, FS_Trips_Z, "Z Re-Scan,Trip_");
        CMDOutput("%:");
      }

      break;
    }

    if(Q_Time !=0)
      MSGOutput("Scan at QTime:" + String(Q_Time));
  }
  //Case 4: all actions should be excuted
  else if (axis == 4)
  {
    // Region = Region + "_Fine_Scan (All Range)";
    CMDOutput("AS");
    msg = Region + "_Fine_Scan (All Range)" + ", Z Scan, Trip_";
    Scan_AllRange_TwoWay(2, 8, 125, 0, 0, 100, StopValue, 500, 2, "Z Scan, Trip_");
    CMDOutput("%:");

    if (isStop)
    {
      return true;
    }

    CMDOutput("AS");
    msg = Region + ", Y Scan, Trip_";
    Scan_AllRange_TwoWay(1, 7, 20, 0, 0, 120, StopValue, 500, 2, "Y Scan, Trip_");
    CMDOutput("%:");

    if (isStop)
    {
      return true;
    }

    CMDOutput("AS");
    msg = Region + ", X Scan, Trip_";
    Scan_AllRange_TwoWay(0, 8, 22, 0, 0, 120, StopValue, 500, 2, "X Scan, Trip_");
    CMDOutput("%:");
  }

  digitalWrite(Tablet_PD_mode_Trigger_Pin, true); //false is PD mode, true is Servo mode
  delay(5);
  MSGOutput("Fine Scan End");

  isLCD = true;
  LCD_Update_Mode = 0;
  LCD_PageNow = 100;
}

//------------------------------------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------------------------------------

int matrix_edge;
int x = 0, y = 0;
double AutoAlign_Result[3] = {0, 0, 0};

bool AutoAlign_Spiral(int M, double StopValue, int stableDelay)
{
  // Serial.println("Position : " + String(X_Pos_Now) + ", " + String(Y_Pos_Now) + ", " + String(Z_Pos_Now));
  DataOutput(false);
  // Serial.println("Set_Steps:" + String(MinMotroStep));
  CMDOutput("ST" + String(MinMotroStep));
  // CMDOutput("M:" + String(M));
  // Serial.println("M:" + String(M));
  Serial.println("StopValue:" + String(StopValue));
  Serial.println("stableDelay:" + String(stableDelay));

  double ts = 0;
  unsigned long timer_1 = 0, timer_2 = 0;
  timer_1 = millis();

  double PD_BestIL = -100, PD_Now = -100;
  int PD_BestIL_Position[2];
  int PD_Best_Pos_Abs[2];

  double SpiralStop_Threshold = StopValue; //Default : 198
  bool isFindThreshold = false;

  AutoAlign_Result[0] = 0;
  AutoAlign_Result[1] = 0;
  AutoAlign_Result[2] = 0;

  matrix_edge = 2 * M + 1;
  x = 0;
  y = 0;

  PD_BestIL = -100;
  PD_BestIL_Position[0] = x;
  PD_BestIL_Position[1] = y;

  PD_Now = Cal_PD_Input_IL(Get_PD_Points);

  int m = 1;

  CMDOutput("$[" + String(x) + "," + String(y) + "]=" + PD_Now); //[0,0]
  // Serial.println("$[" + String(x) + "," + String(y) + "]=" + PD_Now);
  PD_BestIL = PD_Now;
  PD_BestIL_Position[0] = x;
  PD_BestIL_Position[1] = y;
  PD_Best_Pos_Abs[0] = X_Pos_Now;
  PD_Best_Pos_Abs[1] = Y_Pos_Now;
  Serial.println("Inital:(" + String(X_Pos_Now) + "," + String(Y_Pos_Now) + ") = " + String(PD_BestIL));

  if (PD_Now >= SpiralStop_Threshold)
  {
    Serial.println("Over Threshold: " + String(PD_Now) + ", Threshold: " + String(SpiralStop_Threshold));
    return true;
  }

  for (int n = 1; abs(n) < (M + 1); n++)
  {
    if (isStop)
      return true;

    CMDOutput("ML");
    Serial.println("Matrix Layers: " + String(n));

    if (n > m)
      m++;

    y--;
    MotorCC = false;
    Move_Motor(Y_DIR_Pin, Y_STP_Pin, MotorCC, MinMotroStep, delayBetweenStep, stableDelay, true);

    PD_Now = Cal_PD_Input_IL(Get_PD_Points);
    // Serial.println("$[" + String(x) + "," + String(y) + "," + String(Z_Pos_Now) + "]=" + PD_Now);  //[0,-1]
    CMDOutput("$[" + String(x) + "," + String(y) + "]=" + PD_Now); //[0,-1]

    if (PD_Now > PD_BestIL)
    {
      PD_BestIL = PD_Now;
      PD_BestIL_Position[0] = x;
      PD_BestIL_Position[1] = y;
      PD_Best_Pos_Abs[0] = X_Pos_Now;
      PD_Best_Pos_Abs[1] = Y_Pos_Now;

      if (PD_Now >= SpiralStop_Threshold)
      {
        Serial.println("Over Threshold: " + String(PD_Now) + ", Threshold: " + String(SpiralStop_Threshold));

        timer_2 = millis();
        ts = (timer_2 - timer_1) * 0.001;
        CMDOutput("t:" + String(ts, 2));

        return true;
      }
    }

    x--;

    if (isStop)
      return true;

    //To Left

    MotorCC = false;

    while (x >= (-n))
    {
      Move_Motor(X_DIR_Pin, X_STP_Pin, MotorCC, MinMotroStep, delayBetweenStep, stableDelay);

      PD_Now = Cal_PD_Input_IL(Get_PD_Points);

      CMDOutput("$[" + String(x) + "," + String(y) + "]=" + PD_Now);

      if (PD_Now > PD_BestIL)
      {
        PD_BestIL = PD_Now;
        PD_BestIL_Position[0] = x;
        PD_BestIL_Position[1] = y;
        Serial.println("Best IL Update : (" + String(PD_BestIL_Position[0]) + "," + String(PD_BestIL_Position[1]) + ") = " + String(PD_BestIL));
        Serial.println("(" + String(X_Pos_Now) + "," + String(Y_Pos_Now) + ") = " + String(PD_BestIL));
        PD_Best_Pos_Abs[0] = X_Pos_Now;
        PD_Best_Pos_Abs[1] = Y_Pos_Now;

        if (abs(PD_BestIL_Position[0]) >= 100 || abs(PD_BestIL_Position[1]) >= 100)
        {
          Serial.println(String(PD_BestIL_Position[0]) + ", " + String(PD_BestIL_Position[1]));
          return true;
        }
      }
      x--;

      if (PD_BestIL >= SpiralStop_Threshold)
      {
        Serial.println("Over Threshold: " + String(PD_BestIL) + ", Threshold: " + String(SpiralStop_Threshold));

        timer_2 = millis();
        ts = (timer_2 - timer_1) * 0.001;
        CMDOutput("t:" + String(ts, 2));

        return true;
      }

      if (isStop)
        return true;
    }

    x++;
    y++;

    if (isStop)
      return true;

    //Up

    MotorCC = true;

    int nM = n;
    while (y <= (nM))
    {
      Move_Motor(Y_DIR_Pin, Y_STP_Pin, MotorCC, MinMotroStep, delayBetweenStep, stableDelay);

      PD_Now = Cal_PD_Input_IL(Get_PD_Points);
      CMDOutput("$[" + String(x) + "," + String(y) + "]=" + PD_Now);

      if (PD_Now > PD_BestIL)
      {
        PD_BestIL = PD_Now;
        PD_BestIL_Position[0] = x;
        PD_BestIL_Position[1] = y;
        Serial.println("Best IL Update : (" + String(PD_BestIL_Position[0]) + "," + String(PD_BestIL_Position[1]) + ") = " + String(PD_BestIL));
        Serial.println("(" + String(X_Pos_Now) + "," + String(Y_Pos_Now) + ") = " + String(PD_BestIL));
        PD_Best_Pos_Abs[0] = X_Pos_Now;
        PD_Best_Pos_Abs[1] = Y_Pos_Now;

        if (abs(PD_BestIL_Position[0]) >= 100 || abs(PD_BestIL_Position[1]) >= 100)
        {
          return true;
        }
      }
      y++;

      if (PD_BestIL >= SpiralStop_Threshold)
      {
        Serial.println("Over Threshold: " + String(PD_BestIL));

        timer_2 = millis();
        ts = (timer_2 - timer_1) * 0.001;
        CMDOutput("t:" + String(ts, 2));

        return true;
      }

      if (isStop)
        return true;
    }

    y--;
    x++;

    if (isStop)
      return true;

    //To Right

    MotorCC = true;

    while (x <= (n))
    {
      Move_Motor(X_DIR_Pin, X_STP_Pin, MotorCC, MinMotroStep, delayBetweenStep, stableDelay);

      PD_Now = Cal_PD_Input_IL(Get_PD_Points);
      CMDOutput("$[" + String(x) + "," + String(y) + "]=" + PD_Now);

      if (PD_Now > PD_BestIL)
      {
        PD_BestIL = PD_Now;
        PD_BestIL_Position[0] = x;
        PD_BestIL_Position[1] = y;
        Serial.println("Best IL Update : (" + String(PD_BestIL_Position[0]) + "," + String(PD_BestIL_Position[1]) + ") = " + String(PD_BestIL));
        Serial.println("(" + String(X_Pos_Now) + "," + String(Y_Pos_Now) + ") = " + String(PD_BestIL));
        PD_Best_Pos_Abs[0] = X_Pos_Now;
        PD_Best_Pos_Abs[1] = Y_Pos_Now;

        if (abs(PD_BestIL_Position[0]) >= 100 || abs(PD_BestIL_Position[1]) >= 100)
        {
          return true;
        }
      }
      x++;

      if (PD_BestIL >= SpiralStop_Threshold)
      {
        Serial.println("Over Threshold: " + String(PD_BestIL));

        timer_2 = millis();
        ts = (timer_2 - timer_1) * 0.001;
        CMDOutput("t:" + String(ts, 2));

        return true;
      }

      if (isStop)
        return true;
    }

    x--;
    y--;

    if (isStop)
      return true;

    //Down

    MotorCC = false;

    while (y >= (-n))
    {
      Move_Motor(Y_DIR_Pin, Y_STP_Pin, MotorCC, MinMotroStep, delayBetweenStep, stableDelay);

      PD_Now = Cal_PD_Input_IL(Get_PD_Points);
      CMDOutput("$[" + String(x) + "," + String(y) + "]=" + PD_Now);

      if (PD_Now > PD_BestIL)
      {
        PD_BestIL = PD_Now;
        PD_BestIL_Position[0] = x;
        PD_BestIL_Position[1] = y;
        Serial.println("Best IL Update : (" + String(PD_BestIL_Position[0]) + "," + String(PD_BestIL_Position[1]) + ") = " + String(PD_BestIL));
        Serial.println("(" + String(X_Pos_Now) + "," + String(Y_Pos_Now) + ") = " + String(PD_BestIL));
        PD_Best_Pos_Abs[0] = X_Pos_Now;
        PD_Best_Pos_Abs[1] = Y_Pos_Now;

        if (abs(PD_BestIL_Position[0]) >= 100 || abs(PD_BestIL_Position[1]) >= 100)
        {
          return true;
        }
      }
      y--;

      if (PD_BestIL >= SpiralStop_Threshold)
      {
        Serial.println("Over Threshold: " + String(PD_BestIL));

        timer_2 = millis();
        ts = (timer_2 - timer_1) * 0.001;
        CMDOutput("t:" + String(ts, 2));

        return true;
      }

      if (isStop)
        return true;
    }

    y++;
  }

  CMDOutput("ML");
  Serial.println("Matrix Layers: Max");

  if (isStop)
    return true;

  int delta_X = 0, delta_Y = 0;

  if (!sprial_JumpToBest)
  {
    PD_BestIL_Position[0] = 0;
    PD_BestIL_Position[1] = 0; //Jump to (0,0)
  }

  if (PD_BestIL_Position[0] <= 2 * M && PD_BestIL_Position[1] <= 2 * M)
  {
    Move_Motor_abs(0, PD_Best_Pos_Abs[0]);
    Move_Motor_abs(1, PD_Best_Pos_Abs[1]);

    delay(200);

    double finalIL = Cal_PD_Input_IL(Get_PD_Points);
    Serial.println("Final IL : " + String(finalIL));

    Serial.println("Position : " + String(X_Pos_Now) + ", " + String(Y_Pos_Now) + ", " + String(Z_Pos_Now));

    AutoAlign_Result[0] = PD_BestIL_Position[0];
    AutoAlign_Result[1] = PD_BestIL_Position[1];
    AutoAlign_Result[2] = finalIL;

    timer_2 = millis();
    ts = (timer_2 - timer_1) * 0.001;
    CMDOutput("t:" + String(ts, 2));
  }
  else
  {
    Serial.println("Delta step out of range.");
  }
  return false;
}

//------------------------------------------------------------------------------------------------------------------------------------------

bool Scan_Fast(int XYZ, int count, int motorStep, int stableDelay,
                          bool Direction, int delayBetweenStep, double StopPDValue, int Get_PD_Points, int Trips, String msg)
{
  int DIR_Pin = 0;
  int STP_Pin = 0;
  int backlash = 40;
  MotorCC = Direction; // initial direction
  int trip = 1;
  int dataCount = 3;
  int dataCount_ori;
  int indexofBestIL = 0;
  double PD_Value[4 * count + 1];
  long Step_Value[4 * count + 1];
  unsigned long timer_1 = 0, timer_2 = 0;
  double PD_Best = -50;
  timer_1 = millis();

  dataCount = 2 * count + 1;
  dataCount_ori = dataCount;

  switch (XYZ)
  {
  case 0:
    DIR_Pin = X_DIR_Pin;
    STP_Pin = X_STP_Pin;
    backlash = X_backlash;
    delay(5);
    break;
  case 1:
    DIR_Pin = Y_DIR_Pin;
    STP_Pin = Y_STP_Pin;
    backlash = Y_backlash;
    delay(5);
    break;
  case 2:
    DIR_Pin = Z_DIR_Pin;
    STP_Pin = Z_STP_Pin;
    backlash = Z_backlash;
    delay(5);
    break;
  }

  MSGOutput("Scan Step: " + String(motorStep));
  MSGOutput("Backlash: " + String(backlash));
  CMDOutput(">>" + msg + String(trip));

  MSGOutput("StopValue:" + String(StopPDValue));

  double PD_initial = Cal_PD_Input_IL(Get_PD_Points);
  MSGOutput("Initial PD: " + String(PD_initial));

  
  if (PD_initial >= StopPDValue)
    return true;
  else
    PD_Best = PD_initial;
  //-------------------------------------------Jump to Trip_1 initial position-------------------------------------
   
  digitalWrite(DIR_Pin, MotorCC);
  delay(5); 

  //-------------------------------------------------------Trip_1 -----------------------------------------------
 
  for (int i = 0; i < dataCount; i++)
  {
    PD_Value[i] = 0;
    Step_Value[i] = 0;
  }

  double IL_Best_Trip1 = PD_initial;
  long Pos_Best_Trip1 = Get_Position(XYZ);
  long Pos_Ini_Trip1 = Get_Position(XYZ);

  int data_plus_time = 0;

  for (int i = 0; i < dataCount; i++)
  {
    if (isStop)
      return true;

    if (i == 0)
    {
      PD_Value[i] = PD_Now;
      Step_Value[i] = Get_Position(XYZ);
      continue;
    }

    step(STP_Pin, motorStep, delayBetweenStep);    
    delay(stableDelay);

    PD_Value[i] = Cal_PD_Input_IL(Get_PD_Points);

    Step_Value[i] = Get_Position(XYZ);

    if (PD_Value[i] > IL_Best_Trip1)
    {
      indexofBestIL = i;
      IL_Best_Trip1 = PD_Value[i];
      Pos_Best_Trip1 = Get_Position(XYZ);
    }    

    DataOutput();
    DataOutput(XYZ, PD_Value[i]); //int xyz, double pdValue
   
    if (PD_Value[i] >= StopPDValue)
    {
      MSGOutput("Better than StopValue");
      return true;
    }

    if(i > 3 && PD_Value[i]<PD_Value[i-1] && PD_Value[i-1]<PD_Value[i-2] && PD_Value[i-2]<PD_Value[i-3])
    {
      MSGOutput("Over Best IL 3 points, Break trip 1.");
      break;
    }    
  }

  PD_Best = IL_Best_Trip1;

  trip++;

  double Trip2_Initial_IL = 0;
  double IL_Best_Trip2 = 0;
  long Pos_Best_Trip2 = 0;
  long Pos_Ini_Trip2 = 0;

  //------------------------------------Trip_2 ------------------------------------------------------------
  MSGOutput(" --- Trip 2 --- " );
  MSGOutput("trip: " + String(trip) );
  
  if (true)
  {
    CMDOutput("~:" + msg + String(trip));

    PD_Now = Cal_PD_Input_IL(Get_PD_Points);
    IL_Best_Trip2 = PD_Now;
    
    Pos_Best_Trip2 = Get_Position(XYZ);
    Pos_Ini_Trip2 = Get_Position(XYZ);

    for (int i = 0; i < dataCount; i++)
    {
      PD_Value[i] = 0;
      Step_Value[i] = 0;
    }

    MotorCC = !MotorCC; //Reverse direction
    digitalWrite(DIR_Pin, MotorCC);
    delay(5);

    Trip2_Initial_IL = PD_Now;
    Serial.println("Trip2_Initial_IL:" + String(Trip2_Initial_IL));

    for (int i = 0; i < dataCount; i++)
    {
      if (isStop)
      {
        Serial.println("Emergency Stop");
        return true;
      }

      if (i == 0)
      {
        PD_Value[i] = PD_Now;
        Step_Value[i] = Get_Position(XYZ);
        continue;
      }

      step(STP_Pin, motorStep, delayBetweenStep);
      delay(stableDelay);

      PD_Value[i] = Cal_PD_Input_IL(Get_PD_Points);
      Step_Value[i] = Get_Position(XYZ);

      if (PD_Value[i] > IL_Best_Trip2)
      {
        indexofBestIL = i;
        IL_Best_Trip2 = PD_Value[i];
        Pos_Best_Trip2 = Get_Position(XYZ);
      }

      DataOutput();
      DataOutput(XYZ, PD_Value[i]); //int xyz, double pdValue

      if(i>3 && PD_Value[i] >= Trip2_Initial_IL && (PD_initial - PD_Value[i]) < 0.05 && PD_Value[i]<= PD_Value[i-1] && PD_Value[i-1]<= PD_Value[i-2])
      {
        Serial.println("Best Position, IL is: " + String(PD_Value[i]));
        break;
      }

       if(i>3 
        && (PD_Value[i] >= PD_initial || abs(PD_Value[i] - PD_initial) <= 0.02)
        && abs(PD_Value[i] - PD_Value[i-1])<=0.03 
        && abs(PD_Value[i-1] - PD_Value[i-2]) <= 0.03)
      {
        Serial.println("Best Position (2), IL is: " + String(PD_Value[i]));
        break;
      }

      // if(i>10 && PD_Value[i] < PD_initial && PD_Value[i]<= PD_Value[i-1] && PD_Value[i-1]<= PD_Value[i-2] && PD_Value[i-2]<= PD_Value[i-3])
      // {
      //   Serial.println("Scan Fail");
      //   return false;
      // }

      if (PD_Value[i] >= StopPDValue)
      {
        MSGOutput("Better than StopValue");
        break;
      }
     
    }
  }
  MSGOutput("IL_Best_Trip1: " + String(IL_Best_Trip1));
  MSGOutput("IL_Best_Trip2: " + String(IL_Best_Trip2));

  if(PD_Best < IL_Best_Trip2)
    PD_Best = IL_Best_Trip2;
  
  //------------------------------------Trip_3 -------------------------------------------------------

  PD_Now = Cal_PD_Input_IL(2*Get_PD_Points);
  MSGOutput("Best IL: " + String(PD_Best));
  MSGOutput("Final IL: " + String(PD_Now));

  timer_2 = millis();
  double ts = (timer_2 - timer_1) * 0.001;
  CMDOutput("t:" + String(ts, 2));

  if (PD_Now < PD_Best - 0.2)
    return false;
  else
    return true;
}

//------------------------------------------------------------------------------------------------------------------------------------------
double maxIL_in_FineScan = 0;
double minIL_in_FineScan = -100;

bool Scan_AllRange_TwoWay(int XYZ, int count, int motorStep, int stableDelay,
                          bool Direction, int delayBetweenStep, double StopPDValue, int Get_PD_Points, int Trips, String msg)
{
  int DIR_Pin = 0;
  int STP_Pin = 0;
  int backlash = 40;
  MotorCC = Direction; // initial direction
  int trip = 1;
  int dataCount = 3;
  int dataCount_ori;
  int indexofBestIL = 0;
  double PD_Value[4 * count + 1];
  long Step_Value[4 * count + 1];
  unsigned long timer_1 = 0, timer_2 = 0;
  // delayBetweenStep = stableDelay;
  timer_1 = millis();

  dataCount = 2 * count + 1;
  dataCount_ori = dataCount;

  switch (XYZ)
  {
  case 0:
    DIR_Pin = X_DIR_Pin;
    STP_Pin = X_STP_Pin;
    backlash = X_backlash;
    delay(5);
    break;
  case 1:
    DIR_Pin = Y_DIR_Pin;
    STP_Pin = Y_STP_Pin;
    backlash = Y_backlash;
    delay(5);
    break;
  case 2:
    DIR_Pin = Z_DIR_Pin;
    STP_Pin = Z_STP_Pin;
    backlash = Z_backlash;
    delay(5);
    break;
  }

  MSGOutput("Scan Step: " + String(motorStep));
  MSGOutput("Backlash: " + String(backlash));
  CMDOutput(">>" + msg + String(trip));

  MSGOutput("StopValue:" + String(StopPDValue));

  double PD_initial = Cal_PD_Input_IL(Get_PD_Points);
  MSGOutput("Initial PD: " + String(PD_initial));

  maxIL_in_FineScan = PD_initial;
  minIL_in_FineScan = PD_initial;

  if (PD_initial >= StopPDValue)
    return true;

  //-------------------------------------------Jump to Trip_1 initial position-------------------------------------
  digitalWrite(DIR_Pin, MotorCC);
  delay(1);

  if(true)
  {
    step(STP_Pin, motorStep * count, delayBetweenStep); 

    delay(100);

    PD_Now = Cal_PD_Input_IL(Get_PD_Points);
    DataOutput();
    DataOutput(XYZ, PD_Now); //int xyz, double pdValue

    Serial.println("Jump IL: " + String(PD_Now));

    for (size_t i = 0; i < 2; i++)
    {
      if(PD_Now > PD_initial && (PD_Now - PD_initial) >= 1)
      {
        step(STP_Pin, motorStep * count, delayBetweenStep); 
        PD_Now = Cal_PD_Input_IL(Get_PD_Points);
        DataOutput();
        DataOutput(XYZ, PD_Now); //int xyz, double pdValue
        Serial.println("Jump IL: " + String(PD_Now));
      }
      else
        break;
    }
  }
  else
  {
    for (size_t i = 0; i < count; i++)
    {
      step(STP_Pin, motorStep, delayBetweenStep);
      PD_Now = Cal_PD_Input_IL(Get_PD_Points);
      // DataOutput();
      // DataOutput(XYZ, PD_Now); //int xyz, double pdValue

      if(PD_Now < (PD_initial - 3.5))
      {
        Serial.println("Jump IL < (IL - 3.5): " + String(PD_Now));
        break;
      }
    }
  }

  
  
  MotorCC = !MotorCC; //Reverse direction
  digitalWrite(DIR_Pin, MotorCC);

  delay(stableDelay + 105); 

  // CMDOutput(">>" + msg + String(trip));
  // Serial.println(">>" + msg + String(trip)); //Trip_1

  // PD_Now = Cal_PD_Input_IL(Get_PD_Points);
  // DataOutput();
  // DataOutput(XYZ, PD_Now); //int xyz, double pdValue

  //-------------------------------------------------------Trip_1 -----------------------------------------------

  if (PD_Now >= StopPDValue)
  {
    maxIL_in_FineScan = 0;
    minIL_in_FineScan = -100;
    return true;
  }

  for (int i = 0; i < dataCount; i++)
  {
    PD_Value[i] = 0;
    Step_Value[i] = 0;
  }

  double IL_Best_Trip1 = PD_Now;
  long Pos_Best_Trip1 = Get_Position(XYZ);
  long Pos_Ini_Trip1 = Get_Position(XYZ);

  int data_plus_time = 0;

  for (int i = 0; i < dataCount; i++)
  {
    if (isStop)
      return true;

    if (i == 0)
    {
      PD_Value[i] = PD_Now;
      Step_Value[i] = Get_Position(XYZ);
      continue;
    }

    step(STP_Pin, motorStep, delayBetweenStep);    
    delay(stableDelay);

    if(i>0 && PD_Value[i-1] > -2)
      PD_Value[i] = Cal_PD_Input_IL(Get_PD_Points * 3);  // 2500
    else
      PD_Value[i] = Cal_PD_Input_IL(Get_PD_Points);

    Step_Value[i] = Get_Position(XYZ);

    if (PD_Value[i] > IL_Best_Trip1)
    {
      indexofBestIL = i;
      IL_Best_Trip1 = PD_Value[i];
      Pos_Best_Trip1 = Get_Position(XYZ);
    }

    //Update Min, Max IL in Scan Process
    if(PD_Value[i]>maxIL_in_FineScan)
        maxIL_in_FineScan=PD_Value[i];
    if(PD_Value[i]<minIL_in_FineScan)
        minIL_in_FineScan=PD_Value[i];    

    DataOutput();
    DataOutput(XYZ, PD_Value[i]); //int xyz, double pdValue

    if(IL_Best_Trip1 >= -2.5 && PD_Value[i] <= (IL_Best_Trip1 - 1.5) && Trips == 1)
    {
      Serial.println("IL < (IL-1.5): " + String(PD_Value[i]));     

      //Curfit
      if (indexofBestIL != 0 && Pos_Best_Trip1 != Get_Position(XYZ))
      {
        // MSGOutput("i:" + String(i) + ", Pos_Best_Trip1:" + String(Pos_Best_Trip1));
        double x[3];
        double y[3];
        for (int k = -1; k < 2; k++)
        {
          x[k + 1] = Step_Value[indexofBestIL + k]; //idex * step = real steps
          y[k + 1] = PD_Value[indexofBestIL + k];   //fill this with your sensor data
          Serial.println("Point : " + String(x[k + 1]) + " , " + String(y[k + 1]));
        }
        Pos_Best_Trip1 = Curfit(x, y, 3);
        MSGOutput("Best IL position in Trip_1 is: " + String(Pos_Best_Trip1));
      }

      break;
    }

    if (PD_Value[i] >= StopPDValue)
    {
      MSGOutput("Better than StopValue");
      return true;
    }

    if(Trips == 0 && i > 3)
    {
      if( (PD_Value[i]<=PD_Value[i-1] || abs(PD_Value[i] - PD_Value[i-1]) <=0.02) && PD_Value[i]>=-1.8)
      {
        MSGOutput("Over best IL in trip 1");
        PD_Now = Cal_PD_Input_IL(2 * Get_PD_Points);
        MSGOutput("Final IL: " + String(PD_Now));
        timer_2 = millis();
        double ts = (timer_2 - timer_1) * 0.001;
        CMDOutput("t:" + String(ts, 2));
        return true;
      }
    }

    if (i == (dataCount - 1) && Pos_Best_Trip1 == Get_Position(XYZ))
    {
      Serial.println("Datacount+3");
      dataCount = dataCount + 3;
      data_plus_time = data_plus_time + 1;

      if (dataCount - dataCount_ori > 20 || data_plus_time > 3)
      {
        Serial.println("Data plus time: " + String(data_plus_time));
        timer_2 = millis();
        double ts = (timer_2 - timer_1) * 0.001;
        CMDOutput("t:" + String(ts, 2));

        return true;
      }
    }

    else if (indexofBestIL != 0 && i == (dataCount - 1) && Pos_Best_Trip1 != Get_Position(XYZ))
    {
      MSGOutput("i:" + String(i) + ", Pos_Best_Trip1:" + String(Pos_Best_Trip1));
      double x[3];
      double y[3];
      for (int k = -1; k < 2; k++)
      {
        x[k + 1] = Step_Value[indexofBestIL + k]; //idex * step = real steps
        y[k + 1] = PD_Value[indexofBestIL + k];   //fill this with your sensor data
        Serial.println("Point : " + String(x[k + 1]) + " , " + String(y[k + 1]));
      }
      Pos_Best_Trip1 = Curfit(x, y, 3);
      MSGOutput("Best IL position in Trip_1 is: " + String(Pos_Best_Trip1));
    }
  }

  trip++;

  double IL_Best_Trip2 = 0;
  long Pos_Best_Trip2 = 0;
  long Pos_Ini_Trip2 = 0;

  //------------------------------------Trip_2 ------------------------------------------------------------
  MSGOutput(" --- Trip 2 --- " );
  MSGOutput("trip: " + String(trip) );
  MSGOutput("Trips: " + String(Trips) );
  
  if (Trips != 1)
  {
    CMDOutput("~:" + msg + String(trip));

    IL_Best_Trip2 = PD_Now;
    Pos_Best_Trip2 = Get_Position(XYZ);
    Pos_Ini_Trip2 = Get_Position(XYZ);

    for (int i = 0; i < dataCount; i++)
    {
      PD_Value[i] = 0;
      Step_Value[i] = 0;
    }

    MotorCC = !MotorCC; //Reverse direction
    digitalWrite(DIR_Pin, MotorCC);
    delay(5);

    for (int i = 0; i < dataCount; i++)
    {
      if (isStop)
      {
        Serial.println("Emergency Stop");
        return true;
      }

      if (i == 0)
      {
        PD_Value[i] = PD_Now;
        Step_Value[i] = Get_Position(XYZ);
        continue;
      }

      step(STP_Pin, motorStep, delayBetweenStep);
      delay(stableDelay);

      PD_Value[i] = Cal_PD_Input_IL(Get_PD_Points);
      Step_Value[i] = Get_Position(XYZ);

      if (PD_Value[i] > IL_Best_Trip2)
      {
        indexofBestIL = i;
        IL_Best_Trip2 = PD_Value[i];
        Pos_Best_Trip2 = Get_Position(XYZ);
      }

      //Update Min, Max IL in Scan Process
      if(PD_Value[i]>maxIL_in_FineScan)
          maxIL_in_FineScan=PD_Value[i];
      if(PD_Value[i]<minIL_in_FineScan)
          minIL_in_FineScan=PD_Value[i];

      DataOutput();
      DataOutput(XYZ, PD_Value[i]); //int xyz, double pdValue

      if(IL_Best_Trip1 >= -2.5 && PD_Value[i] <= (IL_Best_Trip1 - 1.5) && Trips == 1)
      {
        Serial.println("IL < (IL-1.5): " + String(PD_Value[i]));
        break;
      }

      if (PD_Value[i] >= StopPDValue)
      {
        MSGOutput("Better than StopValue");
        return true;
      }

      if (indexofBestIL != 0 && i == (dataCount - 1) && Pos_Best_Trip2 != Get_Position(XYZ))
      {
        double x[3];
        double y[3];
        for (int k = -1; k < 2; k++)
        {
          x[k + 1] = Step_Value[indexofBestIL + k]; //idex * step = real steps
          y[k + 1] = PD_Value[indexofBestIL + k];   //fill this with your sensor data
          MSGOutput("Point : " + String(x[k + 1]) + " , " + String(y[k + 1]));
        }
        Pos_Best_Trip2 = Curfit(x, y, 3);
        MSGOutput("Best IL position in Trip_2 is: " + String(Pos_Best_Trip2));
      }
    }
  }
  else
    trip--;

  trip++;
  CMDOutput("~:" + msg + String(trip));

  //------------------------------------Trip_3 -------------------------------------------------------

  MSGOutput(" --- Trip 3 --- " );

  double PD_Best = IL_Best_Trip1;
  int deltaPos = 0;

  if (IL_Best_Trip2 > IL_Best_Trip1 && (IL_Best_Trip2 - IL_Best_Trip1)> 0.05 && Trips != 1)
  {
    if (isStop)
    {
      return true;
    }

    MotorCC = !MotorCC; //Reverse direction
    digitalWrite(DIR_Pin, MotorCC);
    delay(5);

    MSGOutput("Best in Trip_2 : " + String(Pos_Best_Trip2)); //------------Best in Trip_2----------------

    if (XYZ == 2)
      Pos_Best_Trip2 = Pos_Best_Trip2 - AQ_Scan_Compensation_Steps_Z_A;

    MSGOutput("Best in Trip_2 (Compensation) : " + String(Pos_Best_Trip2));

    PD_Best = IL_Best_Trip2;

    Move_Motor_abs(XYZ, Pos_Ini_Trip2); //Jump to Trip_2 start position

    delay(100); //100

    deltaPos = abs(Pos_Best_Trip2 - Get_Position(XYZ));

    if (deltaPos < backlash)
    {
      MSGOutput("Jump Backlesh 2");
      step(STP_Pin, (backlash - deltaPos), delayBetweenStep);
      delay(stableDelay + 400);

      deltaPos = abs(Pos_Best_Trip2 - Get_Position(XYZ));
    }

    delay(300);

    PD_Now = Cal_PD_Input_IL(Get_PD_Points);
    DataOutput();
    DataOutput(XYZ, PD_Now); //int xyz, double pdValue

    MotorCC = !MotorCC; //Reverse direction
    digitalWrite(DIR_Pin, MotorCC);
    delay(5);
  }

  else if (Trips == 1)
  {
    MotorCC = !MotorCC; //Reverse direction
    digitalWrite(DIR_Pin, MotorCC);
    delay(5);

    MSGOutput("Jump to Trip Initial Pos : " + String(Pos_Best_Trip2));
    Move_Motor_abs(XYZ, Pos_Ini_Trip1); //Jump to Trip_1 start position

    delay(300); //100

    PD_Now = Cal_PD_Input_IL(Get_PD_Points);
    DataOutput();
    DataOutput(XYZ, PD_Now); //int xyz, double pdValue

    MotorCC = !MotorCC; //Reverse direction
    digitalWrite(DIR_Pin, MotorCC);
    delay(5);

    deltaPos = abs(Pos_Best_Trip1 - Get_Position(XYZ));
    MSGOutput("deltaPos : " + String(deltaPos));
  }

  else //------------Best in Trip_1----------------
  {
    MSGOutput("Best in Trip_1 : " + String(Pos_Best_Trip1));
    MSGOutput("Position Now : " + String(Get_Position(XYZ)));

    if(Pos_Best_Trip1 == Get_Position(XYZ))
    {
      PD_Now = Cal_PD_Input_IL(2 * Get_PD_Points);
      if(abs(PD_Now - IL_Best_Trip1)<=0.12 || PD_Now > IL_Best_Trip1)
        return true;
      else
        return false;
    }
      
    if (XYZ == 2)
      Pos_Best_Trip1 = Pos_Best_Trip1 - AQ_Scan_Compensation_Steps_Z_A;

    MSGOutput("Best in Trip_1 (Compensation) : " + String(Pos_Best_Trip1));


    PD_Best = IL_Best_Trip1;
    deltaPos = abs(Pos_Best_Trip1 - Get_Position(XYZ));

    if (deltaPos < motorStep * 2)
    {
      MSGOutput("Jump Backlesh 1");
     
      step(STP_Pin, (backlash), delayBetweenStep+20);
      delay(stableDelay + 200);

      deltaPos = abs(Pos_Best_Trip2 - Get_Position(XYZ)); //Two curves are totally different, then back to best pos in trip 2

//      return false;
    }

    if (isStop)
    {
      return true;
    }

    MotorCC = !MotorCC; //Reverse direction
    digitalWrite(DIR_Pin, MotorCC);
    delay(2);
  }


  MSGOutput("Delta Pos : " + String(deltaPos));

  // step(STP_Pin, deltaPos, delayBetweenStep);
  // delay(stableDelay);
  // PD_Now = Cal_PD_Input_IL(Get_PD_Points);
  // DataOutput();
  // DataOutput(XYZ, PD_Now); //int xyz, double pdValue

  // if (PD_Now >= PD_Best)
  //   MSGOutput("PD_Best");

  if(true)    
    while (true)
    {
      if (isStop)
      {
        return true;
      }

      if (deltaPos >= motorStep)
      {
        deltaPos = deltaPos - motorStep;

        step(STP_Pin, motorStep, delayBetweenStep);
        delay(stableDelay);
        PD_Now = Cal_PD_Input_IL(Get_PD_Points);
        DataOutput();
        DataOutput(XYZ, PD_Now); //int xyz, double pdValue

        if (PD_Now >= StopPDValue)
        {
          MSGOutput("StopPDValue");
          break;
        }
        if (PD_Now >= PD_Best)
        {
          MSGOutput("Reach IL_Best before");
          break;
        }
      }
      else if (deltaPos > 0 && deltaPos < motorStep)
      {
        step(STP_Pin, deltaPos, delayBetweenStep);
        delay(stableDelay);
        PD_Now = Cal_PD_Input_IL(Get_PD_Points);
        DataOutput();
        DataOutput(XYZ, PD_Now); //int xyz, double pdValue

        break;
      }
      else if (deltaPos == 0)
      {
        break;
      }
      else
        break;
    }

  PD_Now = Cal_PD_Input_IL(2*Get_PD_Points);
  MSGOutput("Best IL: " + String(PD_Best));
  MSGOutput("Final IL: " + String(PD_Now));

  timer_2 = millis();
  double ts = (timer_2 - timer_1) * 0.001;
  CMDOutput("t:" + String(ts, 2));

  if (PD_Now < PD_Best - 0.5)
    return false;
  else
    return true;
}

//------------------------------------------------------------------------------------------------------------------------------------------

void BackLash_Reverse(int XYZ, bool dir, int stableDelay)
{
  int backlash = 40;
  int DIR_Pin = 0;
  int STP_Pin = 0;
  double Modify_Ratio = 1.3;

  switch (XYZ)
  {
  case 0:
    DIR_Pin = X_DIR_Pin;
    STP_Pin = X_STP_Pin;
    backlash = 60;    //default: 95
    Modify_Ratio = 1; //default:1.6
    delay(stableDelay);
    break;
  case 1:
    DIR_Pin = Y_DIR_Pin;
    STP_Pin = Y_STP_Pin;
    backlash = 40;    //default: 85
    Modify_Ratio = 1; //default:1.3
    delay(stableDelay);
    break;
  case 2:
    DIR_Pin = Z_DIR_Pin;
    STP_Pin = Z_STP_Pin;
    backlash = 500; //default: 1100
    Modify_Ratio = 1.5;
    delay(stableDelay);
    break;
  }

  MotorCC = dir;
  digitalWrite(DIR_Pin, MotorCC);
  delay(5);

  step(STP_Pin, (backlash + 20), delayBetweenStep); //Backlash about 40 pulse
  delay(stableDelay * 2);

  //Reverse
  MotorCC = !MotorCC;
  digitalWrite(DIR_Pin, MotorCC);
  delay(10);
  step(STP_Pin, (backlash * Modify_Ratio + 20), delayBetweenStep); //Backlash about 40 pulse
  delay(stableDelay * 2);

  switch (XYZ)
  {
  case 0:
    MotorCC_X = MotorCC;
    break;
  case 1:
    MotorCC_Y = MotorCC;
    break;
  case 2:
    MotorCC_Z = MotorCC;
    break;
  }
}

//------------------------------------------------------------------------------------------------------------------------------------------

long Curfit(double x1[], double y1[], int dataCount)
{
  Serial.println("Curfit Test");
  char buf[4];
  int xpower = 0;
  int order = 2;
  snprintf(buf, 4, "Fitting curve of order %i to data of power %i...\n", order, xpower);
  Serial.println(buf);

  double x[dataCount]; //idex * step = real steps
  double y[dataCount]; //fill this with your sensor data

  double center_x = x1[0];
  Serial.println(String(center_x));
  for (int i = 0; i < 3; i++)
  {
    x[i] = x1[i] - center_x;
    y[i] = y1[i];
    // Serial.println("X:" + String(x[i]));
  }

  int step_distance = abs(x[1] - x[0]);

  double coeffs[order + 1];

  int ret = fitCurve(order, sizeof(y) / sizeof(double), x, y, sizeof(coeffs) / sizeof(double), coeffs);

  if (ret == 0)
  { //Returned value is 0 if no error
    uint8_t c = 'a';
    // Serial.println("Coefficients are");

    // for (int i = 0; i < sizeof(coeffs) / sizeof(double); i++)
    // {
    //   snprintf(buf, 100, "%c=", c++);
    // Serial.print(buf);
    // Serial.print(coeffs[i]);
    // Serial.print('\t');
    // }
    // Serial.println("");

    long result_x = (-1 * coeffs[1]) / (2 * coeffs[0]);
    // Serial.println("Curfit X is : " + String(result_x));

    if (step_distance > abs(x[1] - result_x))
      return result_x + center_x;
    else
    {
      result_x = x1[1];
      // Serial.println("Final X is : " + String(result_x));
      return result_x;
    }
  }
  else
  {
    return x1[1];
  }
}

//------------------------------------------------------------------------------------------------------------------------------------------

double AutoAlign_Scan_DirectionJudge_V2(int XYZ, int count, int Threshold, int motorStep, int stableDelay,
                                        bool Direction, int delayBetweenStep, int Get_PD_Points, int StopPDValue, String msg)
{

  int DIR_Pin = 0;
  int STP_Pin = 0;
  MotorCC = Direction; // direction first
  int backlash = 40, trip = 1;
  bool isReverse = false;
  double ts = 0;
  unsigned long timer_1 = 0, timer_2 = 0;
  timer_1 = millis();

  switch (XYZ)
  {
  case 0:
    DIR_Pin = X_DIR_Pin;
    STP_Pin = X_STP_Pin;
    backlash = 40;
    delay(stableDelay);
    break;
  case 1:
    DIR_Pin = Y_DIR_Pin;
    STP_Pin = Y_STP_Pin;
    backlash = 80;
    delay(stableDelay);
    break;
  case 2:
    DIR_Pin = Z_DIR_Pin;
    STP_Pin = Z_STP_Pin;
    backlash = 40;
    delay(stableDelay);
    break;
  }

  CMDOutput(">>" + msg + String(trip)); //Trip_1------------------------------------------------------------

  double PD_Best = -64,
         PD_Trip2_Best = -64,
         PD_Now = -64;

  double PD_Value[count * 2];
  double PD_Rvrs_Value[count];
  int PD_Best_Pos = 0;
  int PD_Best_Pos_Trip2 = 0;

  double PD_initial = Cal_PD_Input_IL(Get_PD_Points);
  DataOutput(XYZ, PD_initial); //int xyz, double pdValue

  if (PD_initial >= StopPDValue)
  {
    timer_2 = millis();
    ts = (timer_2 - timer_1) * 0.001;
    CMDOutput("t:" + String(ts, 2));

    PD_Best = PD_initial;

    return PD_Best;
  }

  Move_Motor(DIR_Pin, STP_Pin, MotorCC, motorStep * 4, delayBetweenStep, 0, true);

  delay(stableDelay);

  PD_Now = Cal_PD_Input_IL(Get_PD_Points);
  DataOutput(XYZ, PD_Now); //int xyz, double pdValue

  Serial.println("Initial: " + String(PD_initial) + ", After:" + String(PD_Now));

  if (PD_Now >= StopPDValue)
  {
    timer_2 = millis();
    ts = (timer_2 - timer_1) * 0.001;
    CMDOutput("t:" + String(ts, 2));

    PD_Best = PD_Now;

    return PD_Best;
  }

  PD_Best_Pos = 0;

  for (int i = 0; i < count; i++)
  {
    PD_Value[i] = 0;
  }

  if (abs(PD_Now - PD_initial) < 0.05)
  {
    Serial.println("Delta IL < 0.05");

    timer_2 = millis();
    ts = (timer_2 - timer_1) * 0.001;
    CMDOutput("t:" + String(ts, 2));

    return true;
  }

  bool isFirstPointPassBest = false;

  if (PD_Now >= PD_initial)
  {
    PD_Best = PD_Now;
    digitalWrite(DIR_Pin, MotorCC);
    delay(10);
    Serial.println("MotorCC_Forward");

    PD_Value[0] = Cal_PD_Input_IL(Get_PD_Points);
  }
  else
  {
    PD_Best = PD_initial;
    BackLash_Reverse(XYZ, MotorCC, stableDelay);
    Serial.println("MotorCC_Reverse");
    isReverse = true;
  }

  trip++;
  CMDOutput("~:" + msg + String(trip)); //Trip_2------------------------------------------------------------

  PD_Value[0] = Cal_PD_Input_IL(Get_PD_Points);

  int Plus_Times = 0;

  // int Trip2_Start_Position = 0;
  long Pos_Ini_Trip2 = Get_Position(XYZ);

  if (!isFirstPointPassBest)
  {
    for (int i = 0; i < count; i++)
    {
      if (isStop)
        return true;

      step(STP_Pin, motorStep, delayBetweenStep);
      delay(stableDelay);

      PD_Value[i] = Cal_PD_Input_IL(Get_PD_Points);
      DataOutput(XYZ, PD_Value[i]); //int xyz, double pdValue

      if (i == 0)
      {
        PD_Trip2_Best = PD_Value[i];
        PD_Best_Pos_Trip2 = Get_Position(XYZ);
      }

      if (PD_Value[i] >= StopPDValue) //Condition 1
      {
        Serial.println("Over StopPDValue");

        timer_2 = millis();
        ts = (timer_2 - timer_1) * 0.001;
        CMDOutput("t:" + String(ts, 2));

        Serial.println("");

        PD_Best = PD_Value[i];
        return PD_Best;
      }

      if (!isReverse && i == 0 && PD_Value[0] < PD_Best) //Condition 2
      {
        Serial.println("Forward_pass_best");
        return PD_Best;
      }

      if (PD_Value[i] > PD_Best)
      {
        PD_Best = PD_Value[i];
        PD_Best_Pos = i;
      }

      if (PD_Value[i] > PD_Trip2_Best)
      {
        PD_Trip2_Best = PD_Value[i];
        PD_Best_Pos_Trip2 = Get_Position(XYZ);
      }

      if (i >= 1 && PD_Value[i] < PD_Value[i - 1]) //Condition 3
      {
        if (abs(PD_Value[i - 1] - PD_Best) < 0.8)
        {
          Serial.println("Pass best IL");
          break;
        }
      }

      if (PD_Value[i] < -53) //Condition 4
      {
        Serial.println("Miss Target");
        break;
      }

      if (i == count - 1 && PD_Value[i] != PD_Best) //Back to best position after all steps run out
      {
        trip++;
        CMDOutput("~:" + msg + String(trip)); //Trip_3------------------------------------------------------------

        Move_Motor_abs(XYZ, Pos_Ini_Trip2); //Jump to Trip_2 start position
        delay(300);

        DataOutput(XYZ, Cal_PD_Input_IL(Get_PD_Points)); //int xyz, double pdValue

        Move_Motor_abs(XYZ, PD_Best_Pos_Trip2); //Jump to Trip_2 start position
        delay(300);

        DataOutput(XYZ, Cal_PD_Input_IL(Get_PD_Points)); //int xyz, double pdValue
        MSGOutput("Back to best position");
      }
      else if (i == count - 1 && PD_Value[i] == PD_Best) //未通過最高點
      {
        if (Plus_Times < 3)
        {
          MSGOutput("Plus three points");
          count = count + 3;
          Plus_Times++;
        }
      }
    }
  }

  delay(stableDelay);

  PD_Now = Cal_PD_Input_IL(Get_PD_Points);

  if (abs(PD_Now - PD_Best) < 0.4)
  {
    timer_2 = millis();
    ts = (timer_2 - timer_1) * 0.001;
    CMDOutput("t:" + String(ts, 2));

    return PD_Best;
  }

  return PD_Best;

  //Back to best position
  // BackLash_Reverse(XYZ, MotorCC, stableDelay);
  // Serial.println("Final Reverse");

  // trip++;
  // CMDOutput("~:" + msg + String(trip));

  // double pv = 0;
  // double bv = 0;
  // for (int k = 0; k < 35; k++)
  // {
  //   if (isStop)
  //     return true;

  //   step(STP_Pin, motorStep / 4, delayBetweenStep);
  //   delay(stableDelay);

  //   pv = Cal_PD_Input_IL(Get_PD_Points);
  //   DataOutput(XYZ, pv); //int xyz, double pdValue

  //   Serial.println("Reverse PD: " + String(pv));

  //   if (pv >= StopPDValue)
  //   {
  //     Serial.println("Over StopPDValue");

  //     timer_2 = millis();
  //     ts = (timer_2 - timer_1) * 0.001;
  //     CMDOutput("t:" + String(ts, 2));

  //     PD_Best = pv;

  //     return PD_Best;
  //   }

  //   if (abs(pv - PD_Best) < 3 || pv > PD_Best || (pv < bv && pv > Threshold))
  //     break;
  //   else
  //     bv = pv;
  // }

  timer_2 = millis();
  ts = (timer_2 - timer_1) * 0.001;
  Serial.print("TS:");
  Serial.println(ts, 2);
  Serial.println(" ");
  return PD_Best;
}

//------------------------------------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------------------------------------

int Function_Classification(String cmd, int ButtonSelected)
{
  if (cmd != "" && ButtonSelected < 0)
  {
    cmd.trim();
    MSGOutput("get_cmd:" + String(cmd));

    String cmdUpper = cmd;
    cmdUpper.toUpperCase();

//Keyboard - Motor Control
#pragma region - Keyboard - Motor Control
    if (cmd == "Xp1")
    {
      cmd_No = 102;
    }
    else if (cmd == "Xm1")
    {
      cmd_No = 105;
    }
    else if (cmd == "Yp1")
    {
      cmd_No = 104;
    }
    else if (cmd == "Ym1")
    {
      cmd_No = 106;
    }
    else if (cmd == "Zp1")
    {
      cmd_No = 103;
    }
    else if (cmd == "Zm1")
    {
      cmd_No = 101;
    }

    //Jog
    else if (Contains(cmd, "Jog_"))
    {
      cmd.remove(0, 4);

      byte dirPin, stpPin;
      bool dirt;

      if (Contains(cmd, "X"))
      {
        dirPin = X_DIR_Pin;
        stpPin = X_STP_Pin;
      }
      else if (Contains(cmd, "Y"))
      {
        dirPin = Y_DIR_Pin;
        stpPin = Y_STP_Pin;
      }
      else if (Contains(cmd, "Z"))
      {
        dirPin = Z_DIR_Pin;
        stpPin = Z_STP_Pin;
      }

      cmd.remove(0, 1);

      if (Contains(cmd, "m"))
      {
        dirt = false;
      }
      else if (Contains(cmd, "p"))
      {
        dirt = true;
      }

      cmd.remove(0, 2);

      Move_Motor(dirPin, stpPin, dirt, cmd.toDouble(), delayBetweenStep_Y, 0); //(dir_pin, stp_pin, direction, steps, delaybetweensteps, stabledelay)
    }

    //Abs
    else if (Contains(cmd, "Abs_"))
    {
      cmd.remove(0, 4);

      byte dirPin, stpPin, xyz;
      bool dirt;

      if (Contains(cmd, "X"))
      {
        dirPin = X_DIR_Pin;
        stpPin = X_STP_Pin;
        xyz = 0;
      }
      else if (Contains(cmd, "Y"))
      {
        dirPin = Y_DIR_Pin;
        stpPin = Y_STP_Pin;
        xyz = 1;
      }
      else if (Contains(cmd, "Z"))
      {
        dirPin = Z_DIR_Pin;
        stpPin = Z_STP_Pin;
        xyz = 2;
      }

      cmd.remove(0, 2);

      Move_Motor_abs(xyz, cmd.toDouble());
    }

    //Abs All
    else if (Contains(cmd, "AbsAll_"))
    {
      cmd.remove(0, 7);

      int travel_x = 0, travel_y = 0, travel_z = 0;

      travel_x = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //xyz

      cmd.remove(0, cmd.indexOf('_') + 1);

      travel_y = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //xyz

      cmd.remove(0, cmd.indexOf('_') + 1);

      travel_z = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //xyz

      Move_Motor_abs_all(travel_x, travel_y, travel_z);
    }

#pragma endregion

    //(CScan) Scan Twoway Command
    else if (Contains(cmd, "CScan_"))
    {
      cmd.remove(0, 6);
      Serial.println(cmd);

      int XYZ;
      int count;
      int motorStep;
      int stableDelay;
      bool Direction;
      int delayBetweenStep;
      int StopPDValue;
      int Get_PD_Points;
      int Trips;

      XYZ = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //xyz
      cmd.remove(0, cmd.indexOf('_') + 1);

      count = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //count
      cmd.remove(0, cmd.indexOf('_') + 1);

      motorStep = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //step
      cmd.remove(0, cmd.indexOf('_') + 1);

      stableDelay = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //stable delay
      cmd.remove(0, cmd.indexOf('_') + 1);

      Direction = cmd.substring(0, cmd.indexOf('_')) == "1";
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //direction
      cmd.remove(0, cmd.indexOf('_') + 1);

      delayBetweenStep = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //delaySteps
      cmd.remove(0, cmd.indexOf('_') + 1);

      StopPDValue = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //stopValue
      cmd.remove(0, cmd.indexOf('_') + 1);

      Get_PD_Points = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //average points
      cmd.remove(0, cmd.indexOf('_') + 1);

      Trips = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd); //trips

      // int delayBetweenStep = 50;
      String msg = "Manual_Fine_Scan_Trip_";

      bool isOK = true;

      CMDOutput("AS");

      isOK = Scan_AllRange_TwoWay(XYZ, count, motorStep, stableDelay,
                                  Direction, delayBetweenStep, StopPDValue, Get_PD_Points, Trips, msg);

      CMDOutput("%:");

      if (!isOK)
      {
        CMDOutput("AS");
        Scan_AllRange_TwoWay(XYZ, count, motorStep, stableDelay,
                             Direction, delayBetweenStep, StopPDValue, Get_PD_Points, Trips, msg);
        CMDOutput("%:");
      }

      MSGOutput("Auto_Align_End");
    }

    //(SScan) Spiral Scan Command
    else if (Contains(cmd, "SScan_"))
    {
      cmd.remove(0, 6);
      Serial.println(cmd);

      int matrix;
      int motorStep;
      int stb;
      int delay_btw_steps;
      int StopPDValue;
      int Z_Layers;
      int Z_Steps;

      matrix = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //matrix
      cmd.remove(0, cmd.indexOf('_') + 1);

      motorStep = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //step
      cmd.remove(0, cmd.indexOf('_') + 1);

      stb = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //stable delay
      cmd.remove(0, cmd.indexOf('_') + 1);

      //          delay_btw_steps = cmd.substring(0, cmd.indexOf('_')) == "1";
      delay_btw_steps = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //delay_btw_steps
      cmd.remove(0, cmd.indexOf('_') + 1);

      StopPDValue = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //stopValue
      cmd.remove(0, cmd.indexOf('_') + 1);
      //          Serial.println(cmd);  //stopValue

      Z_Layers = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //Z_Layers
      cmd.remove(0, cmd.indexOf('_') + 1);

      Z_Steps = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd); //Z_Steps

      digitalWrite(Tablet_PD_mode_Trigger_Pin, false); //false is PD mode, true is Servo mode
      delay(5);

      M_Level = matrix;

      CMDOutput("AS");
      Serial.println("Auto-Align Start");
      CMDOutput("^X");
      CMDOutput("R:" + String(M_Level * 2 + 1));
      CMDOutput("C:" + String(M_Level * 2 + 1));
      // Serial.println("Rows=" + String(M_Level * 2 + 1));
      // Serial.println("Columns=" + String(M_Level * 2 + 1));
      // Serial.println("^X");

      MinMotroStep = motorStep  * MotorStepRatio; //350
      stableDelay = stb;
      delayBetweenStep = delay_btw_steps;

      if (Z_Layers > 1)
        sprial_JumpToBest = false;

      for (int ZL = 0; ZL < Z_Layers; ZL++)
      {
        if (ZL > 0)
        {
          Move_Motor(Z_DIR_Pin, Z_STP_Pin, false, Z_Steps, 8, 150); //(dir_pin, stp_pin, direction, steps, delaybetweensteps, stabledelay)
        }

        AutoAlign_Spiral(M_Level, StopPDValue, stableDelay); //Input : (Sprial Level, Threshold, stable) Threshold:128
      }

      sprial_JumpToBest = true;

      CMDOutput("X^");
      // Serial.println("X^");

      digitalWrite(Tablet_PD_mode_Trigger_Pin, true); //false is PD mode, true is Servo mode
    }

    //Set auto-align / auto-curing Parameter
    else if (Contains(cmd, "Set::"))
    {
      cmd.remove(0, 5);

      String ParaName = cmd.substring(0, cmd.indexOf('='));
      cmd.remove(0, cmd.indexOf('=') + 1);

      Serial.println("ParaName:" + ParaName + ", Value:" + String(cmd.toDouble()));

      if (ParaName == "AA_SpiralRough_Feed_Steps_Z_A")
        AA_SpiralRough_Feed_Steps_Z_A = cmd.toInt();
      else if (ParaName == "AA_SpiralRough_Spiral_Steps_XY_A")
        AA_SpiralRough_Spiral_Steps_XY_A = cmd.toInt();
      else if (ParaName == "AA_SpiralFine_Spiral_Steps_XY_A")
        AA_SpiralFine_Spiral_Steps_XY_A = cmd.toInt();
      else if (ParaName == "AA_SpiralFine_Scan_Steps_X_A")
        AA_SpiralFine_Scan_Steps_X_A = cmd.toInt();
      else if (ParaName == "AA_SpiralFine_Scan_Steps_X_B")
        AA_SpiralFine_Scan_Steps_X_B = cmd.toInt();
      else if (ParaName == "AA_SpiralFine_Scan_Steps_X_C")
        AA_SpiralFine_Scan_Steps_X_C = cmd.toInt();
      else if (ParaName == "AA_SpiralFine_Scan_Steps_X_D")
        AA_SpiralFine_Scan_Steps_X_D = cmd.toInt();
      else if (ParaName == "AA_SpiralFine_Scan_Steps_Y_A")
        AA_SpiralFine_Scan_Steps_Y_A = cmd.toInt();
      else if (ParaName == "AA_SpiralFine_Scan_Steps_Y_B")
        AA_SpiralFine_Scan_Steps_Y_B = cmd.toInt();
      else if (ParaName == "AA_SpiralFine_Scan_Steps_Y_C")
        AA_SpiralFine_Scan_Steps_Y_C = cmd.toInt();
      else if (ParaName == "AA_SpiralFine_Scan_Steps_Y_D")
        AA_SpiralFine_Scan_Steps_Y_D = cmd.toInt();
      else if (ParaName == "AA_SpiralFine_Scan_Steps_Y_E")
        AA_SpiralFine_Scan_Steps_Y_E = cmd.toInt();
      else if (ParaName == "AA_ScanRough_Feed_Steps_Z_A")
        AA_ScanRough_Feed_Steps_Z_A = cmd.toInt();
      else if (ParaName == "AA_ScanRough_Feed_Steps_Z_B")
        AA_ScanRough_Feed_Steps_Z_B = cmd.toInt();
      else if (ParaName == "AA_ScanRough_Feed_Ratio_Z_A")
        AA_ScanRough_Feed_Ratio_Z_A = cmd.toDouble();
      else if (ParaName == "AA_ScanRough_Feed_Ratio_Z_B")
        AA_ScanRough_Feed_Ratio_Z_B = cmd.toDouble();
      else if (ParaName == "AA_ScanRough_Feed_Ratio_Z_C")
        AA_ScanRough_Feed_Ratio_Z_C = cmd.toDouble();
      else if (ParaName == "AA_ScanRough_Feed_Ratio_Z_D")
        AA_ScanRough_Feed_Ratio_Z_D = cmd.toDouble();

      else if (ParaName == "AA_ScanRough_Scan_Steps_Y_A")
        AA_ScanRough_Scan_Steps_Y_A = cmd.toInt();
      else if (ParaName == "AA_ScanRough_Scan_Steps_Y_B")
        AA_ScanRough_Scan_Steps_Y_B = cmd.toInt();
      else if (ParaName == "AA_ScanRough_Scan_Steps_Y_C")
        AA_ScanRough_Scan_Steps_Y_C = cmd.toInt();
      else if (ParaName == "AA_ScanRough_Scan_Steps_Y_D")
        AA_ScanRough_Scan_Steps_Y_D = cmd.toInt();
      else if (ParaName == "AA_ScanRough_Scan_Steps_X_A")
        AA_ScanRough_Scan_Steps_X_A = cmd.toInt();
      else if (ParaName == "AA_ScanRough_Scan_Steps_X_B")
        AA_ScanRough_Scan_Steps_X_B = cmd.toInt();
      else if (ParaName == "AA_ScanRough_Scan_Steps_X_C")
        AA_ScanRough_Scan_Steps_X_C = cmd.toInt();
      else if (ParaName == "AA_ScanRough_Scan_Steps_X_D")
        AA_ScanRough_Scan_Steps_X_D = cmd.toInt();
      else if (ParaName == "AA_ScanRough_Scan_Steps_X_E")
        AA_ScanRough_Scan_Steps_X_E = cmd.toInt();
      else if (ParaName == "AA_ScanFine_Scan_Steps_Z_A")
        AA_ScanFine_Scan_Steps_Z_A = cmd.toInt();
      else if (ParaName == "AA_ScanFine_Scan_Steps_Y_A")
        AA_ScanFine_Scan_Steps_Y_A = cmd.toInt();
      else if (ParaName == "AA_ScanFine_Scan_Steps_X_A")
        AA_ScanFine_Scan_Steps_X_A = cmd.toInt();
      else if (ParaName == "AA_ScanFinal_Scan_Steps_Z_A")
        AA_ScanFinal_Scan_Steps_Z_A = cmd.toInt();
      else if (ParaName == "AA_ScanFinal_Scan_Steps_Y_A")
        AA_ScanFinal_Scan_Steps_Y_A = cmd.toInt();
      else if (ParaName == "AA_ScanFinal_Scan_Steps_X_A")
        AA_ScanFinal_Scan_Steps_X_A = cmd.toInt();

      else if (ParaName == "AQ_Scan_Compensation_Steps_Z_A")
      {
        AQ_Scan_Compensation_Steps_Z_A = cmd.toInt();
        Serial.println("Write EEPROM AQ_Scan_Compensation_Steps_Z_A: " + WR_EEPROM(160, cmd));
      }
      else if (ParaName == "AQ_Total_TimeSpan")
      {
        AQ_Total_TimeSpan = cmd.toInt();
        Serial.println("Write EEPROM AQ_Total_TimeSpan: " + WR_EEPROM(168, cmd));
      }

      else if (ParaName == "AA_ScanFinal_Scan_Delay_X_A")
      {
        AA_ScanFinal_Scan_Delay_X_A = cmd.toInt();
        Serial.println("Write EEPROM AA_ScanFinal_Scan_Delay_X_A: " + WR_EEPROM(80, cmd));
      }

      else if (ParaName == "AQ_Scan_Steps_Z_A")
      {
        AQ_Scan_Steps_Z_A = cmd.toInt();
        Serial.println("Write EEPROM AQ_Scan_Steps_Z_A: " + WR_EEPROM(176, cmd));
      }
      else if (ParaName == "AQ_Scan_Steps_Z_B")
      {
        AQ_Scan_Steps_Z_B = cmd.toInt();
        Serial.println("Write EEPROM AQ_Scan_Steps_Z_B: " + WR_EEPROM(184, cmd));
      }
      else if (ParaName == "AQ_Scan_Steps_Z_C")
      {
        AQ_Scan_Steps_Z_C = cmd.toInt();
        Serial.println("Write EEPROM AQ_Scan_Steps_Z_C: " + WR_EEPROM(192, cmd));
      }
      else if (ParaName == "AQ_Scan_Steps_Z_D")
      {
        AQ_Scan_Steps_Z_D = cmd.toInt();
        Serial.println("Write EEPROM AQ_Scan_Steps_Z_D: " + WR_EEPROM(200, cmd));
      }

      else if (ParaName == "FS_Count_X")
      {
        FS_Count_X = cmd.toInt();
        Serial.println("Write EEPROM FS_Count_X: " + WR_EEPROM(EP_FS_Count_X, cmd));
      }
      else if (ParaName == "FS_Steps_X")
      {
        FS_Steps_X = cmd.toInt();
        Serial.println("Write EEPROM FS_Steps_X: " + WR_EEPROM(EP_FS_Steps_X, cmd));
      }
      else if (ParaName == "FS_Stable_X")
      {
        FS_Stable_X = cmd.toInt();
        Serial.println("Write EEPROM FS_Stable_X: " + WR_EEPROM(EP_FS_Stable_X, cmd));
      }
      else if (ParaName == "FS_DelaySteps_X")
      {
        FS_DelaySteps_X = cmd.toInt();
        Serial.println("Write EEPROM FS_DelaySteps_X: " + WR_EEPROM(EP_FS_DelaySteps_X, cmd));
      }
      else if (ParaName == "FS_Avg_X")
      {
        FS_Avg_X = cmd.toInt();
        Serial.println("Write EEPROM FS_Avg_X: " + WR_EEPROM(EP_FS_Avg_X, cmd));
      }

      else if (ParaName == "FS_Count_Y")
      {
        FS_Count_Y = cmd.toInt();
        Serial.println("Write EEPROM FS_Count_Y: " + WR_EEPROM(EP_FS_Count_Y, cmd));
      }
      else if (ParaName == "FS_Steps_Y")
      {
        FS_Steps_Y = cmd.toInt();
        Serial.println("Write EEPROM FS_Steps_Y: " + WR_EEPROM(EP_FS_Steps_Y, cmd));
      }
      else if (ParaName == "FS_Stable_Y")
      {
        FS_Stable_Y = cmd.toInt();
        Serial.println("Write EEPROM FS_Stable_Y: " + WR_EEPROM(EP_FS_Stable_Y, cmd));
      }
      else if (ParaName == "FS_DelaySteps_Y")
      {
        FS_DelaySteps_Y = cmd.toInt();
        Serial.println("Write EEPROM FS_DelaySteps_Y: " + WR_EEPROM(EP_FS_DelaySteps_Y, cmd));
      }
      else if (ParaName == "FS_Avg_Y")
      {
        FS_Avg_Y = cmd.toInt();
        Serial.println("Write EEPROM FS_Avg_Y: " + WR_EEPROM(EP_FS_Avg_Y, cmd));
      }

      else if (ParaName == "FS_Count_Z")
      {
        FS_Count_Z = cmd.toInt();
        Serial.println("Write EEPROM FS_Count_Z: " + WR_EEPROM(EP_FS_Count_Z, cmd));
      }
      else if (ParaName == "FS_Steps_Z")
      {
        FS_Steps_Z = cmd.toInt();
        Serial.println("Write EEPROM FS_Steps_Z: " + WR_EEPROM(EP_FS_Steps_Z, cmd));
      }
      else if (ParaName == "FS_Stable_Z")
      {
        FS_Stable_Z = cmd.toInt();
        Serial.println("Write EEPROM FS_Stable_Z: " + WR_EEPROM(EP_FS_Stable_Z, cmd));
      }
      else if (ParaName == "FS_DelaySteps_Z")
      {
        FS_DelaySteps_Z = cmd.toInt();
        Serial.println("Write EEPROM FS_DelaySteps_Z: " + WR_EEPROM(EP_FS_DelaySteps_Z, cmd));
      }
      else if (ParaName == "FS_Avg_Z")
      {
        FS_Avg_Z = cmd.toInt();
        Serial.println("Write EEPROM FS_Avg_Z: " + WR_EEPROM(EP_FS_Avg_Z, cmd));
      }
      else if (ParaName == "FS_Trips_X")
      {
        FS_Trips_X = cmd.toInt();
        Serial.println("Write EEPROM FS_Trips_X: " + WR_EEPROM(EP_FS_Trips_X, cmd));
      }
      else if (ParaName == "FS_Trips_Y")
      {
        FS_Trips_Y = cmd.toInt();
        Serial.println("Write EEPROM FS_Trips_Y: " + WR_EEPROM(EP_FS_Trips_Y, cmd));
      }
      else if (ParaName == "FS_Trips_Z")
      {
        FS_Trips_Z = cmd.toInt();
        Serial.println("Write EEPROM FS_Trips_Z: " + WR_EEPROM(EP_FS_Trips_Z, cmd));
      }
    }

    //Set BackLash Command
    else if (Contains(cmd, "_BL:"))
    {
      if (Contains(cmd, "X"))
      {
        cmd.remove(0, 5);

        X_backlash = cmd.toInt();

        CleanEEPROM(24, 8); //Clean EEPROM(int startPosition, int datalength)

        WriteInfoEEPROM(String(cmd), 24); //(data, start_position)  // Write Data to EEPROM

        EEPROM.commit();

        Serial.println("Set X BackLash: " + String(String(cmd)));

        // Reading Data from EEPROM
        Serial.println("X BackLash in eeprom: " + ReadInfoEEPROM(24, 8)); //(start_position, data_length)

        X_backlash = ReadInfoEEPROM(24, 8).toInt();
      }

      else if (Contains(cmd, "Y"))
      {
        cmd.remove(0, 5);

        Y_backlash = cmd.toInt();

        CleanEEPROM(32, 8); //Clean EEPROM(int startPosition, int datalength)

        WriteInfoEEPROM(String(cmd), 32); //(data, start_position)  // Write Data to EEPROM

        EEPROM.commit();

        Serial.println("Set Y BackLash: " + String(String(cmd)));

        Serial.println("Y BackLash in eeprom: " + ReadInfoEEPROM(32, 8)); //(start_position, data_length)

        Y_backlash = ReadInfoEEPROM(32, 8).toInt();
      }

      else if (Contains(cmd, "Z"))
      {
        cmd.remove(0, 5);

        Z_backlash = cmd.toInt();

        CleanEEPROM(40, 8); //Clean EEPROM(int startPosition, int datalength)

        WriteInfoEEPROM(String(cmd), 40); //(data, start_position)  // Write Data to EEPROM

        EEPROM.commit();

        Serial.println("Set Z BackLash: " + String(String(cmd)));

        Serial.println("Z BackLash in eeprom: " + ReadInfoEEPROM(40, 8)); //(start_position, data_length)

        Z_backlash = ReadInfoEEPROM(40, 8).toInt();
      }
    }

    //Set Scan Steps Command
    else if (Contains(cmd, "_ScanSTP:"))
    {
      if (Contains(cmd, "X"))
      {
        cmd.remove(0, 10);

        X_ScanSTP = cmd.toInt();

        CleanEEPROM(48, 8);                                         //Clean EEPROM(int startPosition, int datalength)
        WriteInfoEEPROM(String(cmd), 48);                           //(data, start_position)  // Write Data to EEPROM
        Serial.println("Save in eeprom: " + ReadInfoEEPROM(48, 8)); //(start_position, data_length)

        Serial.println("Set X Scan Step: " + String(X_ScanSTP));
      }
      else if (Contains(cmd, "Y"))
      {
        cmd.remove(0, 10);
        Y_ScanSTP = cmd.toInt();

        CleanEEPROM(56, 8);                                         //Clean EEPROM(int startPosition, int datalength)
        WriteInfoEEPROM(String(cmd), 56);                           //(data, start_position)  // Write Data to EEPROM
        Serial.println("Save in eeprom: " + ReadInfoEEPROM(56, 8)); //(start_position, data_length)

        Serial.println("Set Y Scan Step: " + String(String(cmd)));
      }
      else if (Contains(cmd, "Z"))
      {
        cmd.remove(0, 10);
        Z_ScanSTP = cmd.toInt();

        CleanEEPROM(64, 8);                                         //Clean EEPROM(int startPosition, int datalength)
        WriteInfoEEPROM(String(cmd), 64);                           //(data, start_position)  // Write Data to EEPROM
        Serial.println("Save in eeprom: " + ReadInfoEEPROM(64, 8)); //(start_position, data_length)

        Serial.println("Set Z Scan Step: " + String(String(cmd)));
      }
    }

    //Get Ref Command
    else if (cmd == "REF?")
    {
      String eepromString = ReadInfoEEPROM(0, 8); //(int start_position, int data_length)

      Serial.println("Get_Ref:" + eepromString); //(start_position, data_length)  // Reading Data from EEPROM

      ref_Dac = eepromString.toDouble();
      ref_IL = ILConverter(ref_Dac);

      Serial.println("Dac:" + eepromString);  //(start_position, data_length)  // Reading Data from EEPROM
      Serial.println("IL:" + String(ref_IL)); //(start_position, data_length)  // Reading Data from EEPROM
    }

    //Set Ref Command
    else if (Contains(cmd, "Set_Ref:"))
    {
      cmd.remove(0, 8);

      CleanEEPROM(0, 8);               //Clean EEPROM(int startPosition, int datalength)
      WriteInfoEEPROM(String(cmd), 0); //(data, start_position)  // Write Data to EEPROM
      EEPROM.commit();

      Serial.println("Update Ref Value : " + String(cmd));

      String eepromString = ReadInfoEEPROM(0, 8); //(int start_position, int data_length)

      Serial.println("PD ref: " + eepromString); //(start_position, data_length)  // Reading Data from EEPROM

      ref_Dac = eepromString.toDouble();
      ref_IL = ILConverter(ref_Dac);
    }

    //Set Target IL
    else if (Contains(cmd, "Set_Target_IL:"))
    {
      cmd.remove(0, 14);
      Target_IL = WR_EEPROM(72, cmd).toDouble();
      MSGOutput("Set_Target_IL:" + String(Target_IL));
    }

    //Set Motor Step Ratio
    else if (Contains(cmd, "Set_MotorStepRatio:"))
    {
      cmd.remove(0, 19);
      MotorStepRatio = WR_EEPROM(208, cmd).toDouble();
      MSGOutput("Set_MotorStepRatio:" + String(MotorStepRatio));
    }

    //Set Manual Control Motor Speed
    else if (Contains(cmd, "Set_Motor_Speed_"))
    {
      cmd.remove(0, 16);

      if (Contains(cmd, "X"))
      {
        cmd.remove(0, 2);
        delayBetweenStep_X = cmd.toInt();
        WR_EEPROM(48, cmd);
      }
      else if (Contains(cmd, "Y"))
      {
        cmd.remove(0, 2);
        delayBetweenStep_Y = cmd.toInt();
        WR_EEPROM(56, cmd);

      }
      else if (Contains(cmd, "Z"))
      {
        cmd.remove(0, 2);
        delayBetweenStep_Z = cmd.toInt();
        WR_EEPROM(64, cmd);
      }

      Serial.println("Set Manual Control Motor Speed:" + cmd);
    }

    //Set PD average points
    else if (Contains(cmd, "Set_PD_average_Points:"))
    {
      cmd.remove(0, 22);
      Get_PD_Points = cmd.toInt();

      Serial.println("Set Get_PD_Points: " + WR_EEPROM(152, cmd));
    }

    //Set Board ID
    else if (Contains(cmd, "ID#"))
    {
      cmd.remove(0, 3);
      Serial.println("Set Board ID: " + WR_EEPROM(8, cmd));
    }

    //Get Board ID
    else if (cmd == "ID?")
    {
      Serial.println(ReadInfoEEPROM(8, 8));
    }

    //Set Station ID
    else if (Contains(cmd, "ID_Station#"))
    {
      cmd.remove(0, 11);
      Serial.println("Set Station ID: " + WR_EEPROM(16, cmd));
    }

    //Get Station ID
    else if (cmd == "ID_Station?")
    {
      Serial.println(ReadInfoEEPROM(16, 8));
    }

    //Set Server ID
    else if (Contains(cmd, "ID_Server#"))
    {
      cmd.remove(0, 10);
      Serial.println("Set Server ID: " + WR_EEPROM(88, 32, cmd));
    }

    //Get Server ID
    else if (cmd == "ID_Server?")
    {
      MSGOutput(ReadInfoEEPROM(88, 32));
    }

    //Set Server Password
    else if (Contains(cmd, "PW_Server#"))
    {
      cmd.remove(0, 10);
      Serial.println("Set Server Password: " + WR_EEPROM(120, 32, cmd));
    }

    //Get Server Password
    else if (cmd == "PW_Server?")
    {
      Serial.println(ReadInfoEEPROM(120, 32));
    }

    //Clena EEPROM : Start position (default length = 8)
    else if (Contains(cmd, "Clean_EEPROM:"))
    {
      cmd.remove(0, 13);
      CleanEEPROM(cmd.toInt(), 8);
      WR_EEPROM(cmd.toInt(), "");
      EEPROM.commit();
      Serial.println("Clean_EEPROM:" + cmd);
      // Serial.println(ReadInfoEEPROM(cmd.toInt(), 8));
    }

    //Command No.
    else if (Contains(cmd, "cmd"))
    {
      cmd.remove(0, 3);
      cmd_No = cmd.toInt();
      delay(10);

      //          cmd_No = 4;  //Auto-Align
      //          cmd_No = 5;   //Fine scan
      //          cmd_No = 6;   //Auto curing
      //          cmd_No = 7;  //To re-load position
      //          cmd_No = 8;  //To Home
      //          cmd_No = 9;  //To Home
      //          cmd_No = 10;  //To Home
      //          cmd_No = 16;  //Set reLoad
      //          cmd_No = 17;  //Set home
      //          cmd_No = 18;  //Set Z target
      //          cmd_No = 19;  //Get ref
      //          cmd_No = 20;  //Spiral
      //          cmd_No = 21;  //Keep print IL to PC
      //          cmd_No = 22;  //Scan X
      //          cmd_No = 23;  //Scan Y
      //          cmd_No = 24;  //Scan Z
    }

    //Action : Reply
    if (true)
    {
    }
  }
  else if (ButtonSelected >= 0)
  {
    //Keyboard No. to Cmd Set No.
    switch (ButtonSelected)
    {
    case 7:
      cmd_No = 1;
      break;

    case 8:
      cmd_No = 2;
      break;

    case 9:
      cmd_No = 3;
      break;

    default:
      cmd_No = ButtonSelected;
      break;
    }
  }

  return cmd_No;
}

//------------------------------------------------------------------------------------------------------------------------------------------

int Function_Excecutation(String cmd, int cmd_No)
{
  //Function Execution
  // String cmd = "";

  if (cmd_No != 0)
  {
    // Serial.println("Btn:" + String(ButtonSelected) + ", CMD:" + String(cmd_No));

    //Functions: Alignment
    if (cmd_No <= 100)
    {
      switch (cmd_No)
      {
        //Functions: Auto Align
      case 1: /* Auto Align */
        if (true)
        {
        
        }
        cmd_No = 0;
        break;

        //Functions: Fine Scan
      case 2: /* Fine Scan */
        if (!btn_isTrigger)
        {
          StopValue = 0; //0 dB

          bool K_OK = true;
          bool initial_wifi_isConnected = isWiFiConnected;
          isWiFiConnected = false;

          isLCD = true;
          PageLevel = 102;
          updateUI(PageLevel);
          AQ_Scan_Compensation_Steps_Z_A = 0;

          digitalWrite(Tablet_PD_mode_Trigger_Pin, false); //false is PD mode, true is Servo mode

          //------------------------------------------------------------------------------------------------------------------------------Q Scan Z

          // CMDOutput("AS");

          // Scan_AllRange_TwoWay(2, 6, 100, AA_ScanFinal_Scan_Delay_X_A, 0, 80, StopValue, 600, 2, "Z Scan, Trip_"); //--Z--
          Fine_Scan(3, false); 

          // CMDOutput("%:");

          if (isStop)
            true;

          //------------------------------------------------------------------------------------------------------------------------------Q Scan Y

          // CMDOutput("AS");
          // // K_OK = Scan_AllRange_TwoWay(1, 7, 20, AA_ScanFinal_Scan_Delay_X_A, 0, 120, StopValue, 500, 2, "Y Scan, Trip_"); //Fast
          // K_OK = Scan_AllRange_TwoWay(1, 6, 35, AA_ScanFinal_Scan_Delay_Y_A, 0, 100, StopValue, 600, 2, "Y Scan, Trip_"); //Slow
          // CMDOutput("%:");

          // if (!K_OK)
          // {
          //   CMDOutput("AS");
          //   // K_OK = Scan_AllRange_TwoWay(1, 7, 20, AA_ScanFinal_Scan_Delay_X_A, 0, 120, StopValue, 500, 2, "Y Scan, Trip_");//Fast
          //   K_OK = Scan_AllRange_TwoWay(1, 6, 35, AA_ScanFinal_Scan_Delay_Y_A, 0, 100, StopValue, 600, 2, "Y Scan, Trip_"); //Slow
          //   CMDOutput("%:");
          // }

           Fine_Scan(2, false); 
          // CMDOutput("%:");

          if (isStop)
            true;

          //------------------------------------------------------------------------------------------------------------------------------Q Scan X

          // CMDOutput("AS");

          // Scan_AllRange_TwoWay(0, 8, 22, AA_ScanFinal_Scan_Delay_X_A, 0, 120, StopValue, 500, 2, "X Scan, Trip_"); //steps:350
          Fine_Scan(1, false); 
          // CMDOutput("%:");

          if (isStop)
            true;

          digitalWrite(Tablet_PD_mode_Trigger_Pin, true); //false is PD mode, true is Servo mode

          MSGOutput("Auto Align End");

          isLCD = true;
          PageLevel = 0;
          updateUI(PageLevel);
          AQ_Scan_Compensation_Steps_Z_A = ReadInfoEEPROM(160, 8).toInt();

          isWiFiConnected = initial_wifi_isConnected;
        }
        cmd_No = 0;
        break;

      //Functions: Auto Curing
      case 3: /* Auto Curing */
        if (!btn_isTrigger)
        {
          btn_isTrigger = false;

          isILStable = false;

          ButtonSelected = -1;

          double IL_stable_count = 0;
          double Acceptable_Delta_IL = 12; //0.8
          Q_Time = 0;

          time_curing_0 = millis();
          time_curing_1 = time_curing_0;
          time_curing_2 = time_curing_1;

          digitalWrite(Tablet_PD_mode_Trigger_Pin, false); //false is PD mode, true is Servo mode
          delay(155);

          AutoCuring_Best_IL = Cal_PD_Input_IL(Get_PD_Points * 3);

          StopValue = AutoCuring_Best_IL;

          if(StopValue > -0.9)
            StopValue = -0.9;

          Z_ScanSTP = AQ_Scan_Steps_Z_A; //125 (AQ_Scan_Steps_Z_A)
          MSGOutput("Auto-Curing");
          CMDOutput("AQ");                             // Auto_Curing Start
          CMDOutput("QT" + String(AQ_Total_TimeSpan)); // Auto_Curing Start
          MSGOutput("StopValue : " + String(StopValue));
          // MSGOutput("AQ_Scan_Steps_Z_A : " + String(AQ_Scan_Steps_Z_A));
          // MSGOutput("AQ_Scan_Steps_Z_B : " + String(AQ_Scan_Steps_Z_B));
          // MSGOutput("AQ_Scan_Steps_Z_C : " + String(AQ_Scan_Steps_Z_C));
          // MSGOutput("AQ_Scan_Steps_Z_D : " + String(AQ_Scan_Steps_Z_D));

          while (true)
          {
            PD_Now = Cal_PD_Input_IL(Get_PD_Points);
            Q_Time = ((millis() - time_curing_0) / 1000);
            MSGOutput("Curing Time:" + String(Q_Time) + " s");
            // MSGOutput("Threshold: " + String(AutoCuring_Best_IL - Acceptable_Delta_IL) + ", Now: " + String(PD_Now));
            MSGOutput("PD_Power:" + String(PD_Now)); //dB

            digitalWrite(Tablet_PD_mode_Trigger_Pin, false); //false is PD mode, true is Servo mode
            delay(5);

            if (Serial.available())
              cmd = Serial.readString();

            // MSGOutput("ButtonSelected: " + String(ButtonSelected));

            // MSGOutput("cmd in Q loop: " + String(cmd));

            cmd_No = Function_Classification(cmd, ButtonSelected);

            cmd = ""; //Reset command from serial port

            isLCD = true;
            PageLevel = 103;
            updateUI(103);
            delay(1000); //default: 700

            if (isStop)
              break;

            //Q State
            if (true)
            {
              if (Q_Time <= 540)  // 540
              {
                Q_State = 1;
              }
              else if (Q_Time > 540 && Q_Time <= 600)  //540, 600
              {
                Q_State = 2;
              }
              else if (Q_Time > 600 && Q_Time <= 700)  //600, 700
              {
                Q_State = 3;
              }
              else if (Q_Time > 700)  //700
              {
                Q_State = 4;
              }
            }

            //Q Stop Conditions
            if (true)
            {
              //IL Stable Time ,  70 secs,  curing time threshold , 12.5 mins
              if (time_curing_2 - time_curing_1 > 70000 && Q_Time >= 800) // 800
              {
                MSGOutput("IL Stable - Stop Auto Curing");
                isStop = true;
                break;
              }
              //Total curing time , 14 mins, 840s
              else if (Q_Time >= AQ_Total_TimeSpan - 1)
              {
                MSGOutput("Over Limit Curing Time - Stop Auto Curing");
                isStop = true;
                break;
              }

              if (isILStable && (Q_Time) >= 800) //800
              {
                MSGOutput("IL Stable in Scan - Stop Auto Curing");
                break;
              }
            }

            if (isStop)
              break;

            // MSGOutput("Z_ScanSTP: " + String(Z_ScanSTP));

            //Q scan conditions
            if (true)
            {
              if (Q_State == 2)
              {
                if (Z_ScanSTP > AQ_Scan_Steps_Z_B)
                {
                  Z_ScanSTP = AQ_Scan_Steps_Z_B; //125 - > 35 (AQ_Scan_Steps_Z_B)
                  MSGOutput("Update Z Scan Step: " + String(Z_ScanSTP));
                }
              }
              else if (Q_State == 3)
              {
                if (Z_ScanSTP > AQ_Scan_Steps_Z_C)
                {
                  Z_ScanSTP = AQ_Scan_Steps_Z_C; //70 (AQ_Scan_Steps_Z_C)
                  MSGOutput("Update Z Scan Step: " + String(Z_ScanSTP));
                }
              }
              else if (Q_State == 4)
              {
                if (Z_ScanSTP > AQ_Scan_Steps_Z_D)
                {
                  Z_ScanSTP = AQ_Scan_Steps_Z_D; //50 (AQ_Scan_Steps_Z_D)
                  MSGOutput("Update Z Scan Step: " + String(Z_ScanSTP));
                }
              }

              if (Q_Time > 540)
              {
                if(Acceptable_Delta_IL != 0.2 ){
                  Acceptable_Delta_IL = 0.2; // Target IL changed 0.25
                  MSGOutput("Update Scan Condition: " + String(Acceptable_Delta_IL));     
                }
              }
            }

            PD_Now = Cal_PD_Input_IL(Get_PD_Points * 3);  //Increase IL stability

            if (PD_Now >= (AutoCuring_Best_IL - (Acceptable_Delta_IL)))
            {
              time_curing_2 = millis();
              continue;
            }
            else
            {
              time_curing_3 = millis();
              Q_Time = (time_curing_3 - time_curing_0) / 1000;
              MSGOutput("Auto-Curing Time: " + String(Q_Time) + " s");

              //Q Scan
              if (true)
              {
                PD_Now = Cal_PD_Input_IL(Get_PD_Points * 3);  //Increase IL stability

                if (PD_Now < (AutoCuring_Best_IL - Acceptable_Delta_IL))
                {
                  Fine_Scan(1, false); //Q Scan X

                  MSGOutput("X PD_Now:" + String(PD_Now) + ", IL:" + String(Cal_PD_Input_IL(Get_PD_Points)));

                  if (PD_Now - Cal_PD_Input_IL(Get_PD_Points) > 1)
                    Fine_Scan(1, false); //Q Scan X

                  if (Q_State >= 4 && (maxIL_in_FineScan - minIL_in_FineScan)<=0.25 && Q_Time>=800){
                    MSGOutput("Delta IL less than 0.25 , break curing loop");
                    MSGOutput("X maxIL_in_FineScan:" + String(maxIL_in_FineScan) 
                    + ", minIL_in_FineScan:" + String(minIL_in_FineScan));
                    break;
                  }
                }

                time_curing_3 = millis();
                Q_Time = (time_curing_3 - time_curing_0) / 1000;
                MSGOutput("Auto-Curing Time: " + String(Q_Time) + " s");

                if (isStop)
                  break;

                PD_Now = Cal_PD_Input_IL(Get_PD_Points * 3);  //Increase IL stability
                MSGOutput("Q_State: " + String(Q_State));

                if (PD_Now < (AutoCuring_Best_IL - Acceptable_Delta_IL) || Q_State == 1)
                {
                  Fine_Scan(2, false); //--------------------------------------------------------Q Scan Y

                  MSGOutput("Y PD_Now:" + String(PD_Now) + ", IL:" + String(Cal_PD_Input_IL(Get_PD_Points)));

                  if (PD_Now - Cal_PD_Input_IL(Get_PD_Points) > 1)
                    Fine_Scan(2, false); //------------------------------------------------------Q Scan Y

                  if (Q_State >= 4 && (maxIL_in_FineScan - minIL_in_FineScan)<=0.25 && Q_Time>=800){
                    MSGOutput("Delta IL less than 0.25 , break curing loop");
                    MSGOutput("Y maxIL_in_FineScan:" + String(maxIL_in_FineScan) 
                    + ", minIL_in_FineScan:" + String(minIL_in_FineScan));
                    break;
                  }
                }

                time_curing_3 = millis();
                Q_Time = (time_curing_3 - time_curing_0) / 1000;
                MSGOutput("Auto-Curing Time: " + String(Q_Time) + " s");

                if (isStop)
                  break;

                PD_Before = Cal_PD_Input_IL(Get_PD_Points);

                bool K_OK = true;

                if (PD_Now < (AutoCuring_Best_IL - Acceptable_Delta_IL))
                {
                  //-----------------------------------------------------------Q Scan Z
                  CMDOutput("AS");

                  K_OK = Scan_AllRange_TwoWay(2, FS_Count_Z, Z_ScanSTP, FS_Stable_Z, 0, FS_DelaySteps_Z, StopValue, FS_Avg_Z, FS_Trips_Z, "Z Scan,Trip_");
                  if (!K_OK)
                  {
                    Scan_AllRange_TwoWay(2, FS_Count_Z, Z_ScanSTP, FS_Stable_Z, 0, FS_DelaySteps_Z, StopValue, FS_Avg_Z, FS_Trips_Z, "Z Re-Scan,Trip_");
                    // Scan_AllRange_TwoWay(2, 8, Z_ScanSTP, 30, 0, 100, StopValue, Get_PD_Points, 2, "Z Re-Scan, Trip_");
                  }
                  
                  CMDOutput("%:");
                }

                if (isStop)
                  break;
              }

              PD_Now = Cal_PD_Input_IL(Get_PD_Points);
              MSGOutput("Q_State: " + String(Q_State));

              if (abs(PD_Before - PD_Now) < 0.3 && (time_curing_3 - time_curing_0) > 750000)
              {
                IL_stable_count++;

                if (IL_stable_count > 4)
                {
                  MSGOutput("IL stable to break");
                  break;
                }
              }

              time_curing_1 = millis();
              time_curing_2 = time_curing_1;
            }
          }

          time_curing_3 = millis();
          MSGOutput("Total Auto-Curing Time: " + String((time_curing_3 - time_curing_0) / 1000) + " s");

          StopValue = Target_IL;
          digitalWrite(Tablet_PD_mode_Trigger_Pin, true); //false is PD mode, true is Servo mode

          String eepromString = ReadInfoEEPROM(40, 8);                         //Reading z backlash from EEPROM
          MSGOutput("Reset Z backlash from EEPROM: " + ReadInfoEEPROM(40, 8)); //(start_position, data_length)
          Z_backlash = eepromString.toInt();

          isLCD = true;
          PageLevel = 0;
          updateUI(PageLevel);
          MSGOutput("LCD Re-Start");

          MSGOutput("Auto Q End");

          Q_Time = 0;
        }
        cmd_No = 0;
        break;

      case 5: /* Fine Scan X */
        if (!btn_isTrigger)
        {
          bool initial_wifi_isConnected = isWiFiConnected;
          isWiFiConnected = false;

          isLCD = true;
          PageLevel = 102;
          updateUI(PageLevel);

          Fine_Scan(1, false); 

          MSGOutput("Auto Align End");

          isLCD = true;
          PageLevel = 0;
          updateUI(PageLevel);

          isWiFiConnected = initial_wifi_isConnected;
        }
        cmd_No = 0;
        break;

      case 6: /* Fine Scan Y */
        if (!btn_isTrigger)
        {
          bool initial_wifi_isConnected = isWiFiConnected;
          isWiFiConnected = false;

          isLCD = true;
          PageLevel = 102;
          updateUI(PageLevel);

          Fine_Scan(2, false); 

          MSGOutput("Auto Align End");

          isLCD = true;
          PageLevel = 0;
          updateUI(PageLevel);

          isWiFiConnected = initial_wifi_isConnected;
        }
        cmd_No = 0;
        break;

      case 7: /* Fine Scan Z */
        if (!btn_isTrigger)
        {
          bool initial_wifi_isConnected = isWiFiConnected;
          isWiFiConnected = false;

          isLCD = true;
          PageLevel = 102;
          updateUI(PageLevel);

          Fine_Scan(3, false); 

          MSGOutput("Auto Align End");

          isLCD = true;
          PageLevel = 0;
          updateUI(PageLevel);

          isWiFiConnected = initial_wifi_isConnected;
        }
        cmd_No = 0;
        break;

      case 8:
        if (!btn_isTrigger)
        {
          bool initial_wifi_isConnected = isWiFiConnected;
          isWiFiConnected = false;

          isLCD = true;
          PageLevel = 102;
          updateUI(PageLevel);

          MSGOutput("");
          MSGOutput("Fine Scan ");

          MSGOutput("Stop Value: " + String(StopValue));

          digitalWrite(Tablet_PD_mode_Trigger_Pin, false); //false is PD mode, true is Servo mode
          delay(5);

          bool K_OK = true;

          MotorCC_X = digitalRead(X_DIR_Pin);

          CMDOutput("AS");
          K_OK = Scan_Fast(0, FS_Count_X, FS_Steps_X, FS_Stable_X, MotorCC_X, FS_DelaySteps_X, StopValue, FS_Avg_X, FS_Trips_X, "X Scan,Trip_");
          CMDOutput("%:");

          if (!K_OK)
          {
            MotorCC_X = digitalRead(X_DIR_Pin);
            CMDOutput("AS");
            Scan_Fast(0, FS_Count_X, FS_Steps_X, FS_Stable_X, MotorCC_X, FS_DelaySteps_X, StopValue, FS_Avg_X, FS_Trips_X, "X Re-Scan,Trip_");
            CMDOutput("%:");
          }

          MSGOutput("Auto Align End");

          isLCD = true;
          PageLevel = 0;
          updateUI(PageLevel);

          isWiFiConnected = initial_wifi_isConnected;
        }
        cmd_No = 0;
        break;

      case 9:
        if (!btn_isTrigger)
        {
          bool initial_wifi_isConnected = isWiFiConnected;
          isWiFiConnected = false;

          isLCD = true;
          PageLevel = 102;
          updateUI(PageLevel);

          MSGOutput("");
          MSGOutput("Fine Scan ");

          MSGOutput("Stop Value: " + String(StopValue));

          digitalWrite(Tablet_PD_mode_Trigger_Pin, false); //false is PD mode, true is Servo mode
          delay(5);

          bool K_OK = true;

          MotorCC_Y = digitalRead(Y_DIR_Pin);

          CMDOutput("AS");
          K_OK = Scan_Fast(0, FS_Count_Y, FS_Steps_Y, FS_Stable_Y, MotorCC_Y, FS_DelaySteps_Y, StopValue, FS_Avg_Y, FS_Trips_Y, "Y Scan,Trip_");
          CMDOutput("%:");

          if (!K_OK)
          {
            MotorCC_Y = digitalRead(Y_DIR_Pin);
            CMDOutput("AS");
            Scan_Fast(0, FS_Count_Y, FS_Steps_Y, FS_Stable_Y, MotorCC_Y, FS_DelaySteps_Y, StopValue, FS_Avg_Y, FS_Trips_Y, "Y Scan,Trip_");
            CMDOutput("%:");
          }

          MSGOutput("Auto Align End");

          isLCD = true;
          PageLevel = 0;
          updateUI(PageLevel);

          isWiFiConnected = initial_wifi_isConnected;
        }
        cmd_No = 0;
        break;

      case 10:
        if (!btn_isTrigger)
        {
          bool initial_wifi_isConnected = isWiFiConnected;
          isWiFiConnected = false;

          isLCD = true;
          PageLevel = 102;
          updateUI(PageLevel);

          MSGOutput("");
          MSGOutput("Fine Scan ");

          MSGOutput("Stop Value: " + String(StopValue));

          digitalWrite(Tablet_PD_mode_Trigger_Pin, false); //false is PD mode, true is Servo mode
          delay(5);

          bool K_OK = true;

          MotorCC_Z = digitalRead(Z_DIR_Pin);

          CMDOutput("AS");
          K_OK = Scan_Fast(0, FS_Count_Z, FS_Steps_Z, FS_Stable_Z, MotorCC_Z, FS_DelaySteps_Z, StopValue, FS_Avg_Z, FS_Trips_Z, "Z Scan,Trip_");
          CMDOutput("%:");

          if (!K_OK)
          {
            MotorCC_Z = digitalRead(Z_DIR_Pin);
            CMDOutput("AS");
            Scan_Fast(0, FS_Count_Z, FS_Steps_Z, FS_Stable_Z, MotorCC_Z, FS_DelaySteps_Z, StopValue, FS_Avg_Z, FS_Trips_Z, "Z Scan,Trip_");
            CMDOutput("%:");
          }

          MSGOutput("Auto Align End");

          isLCD = true;
          PageLevel = 0;
          updateUI(PageLevel);

          isWiFiConnected = initial_wifi_isConnected;
        }
        cmd_No = 0;
        break;

      case 11:
        MSGOutput("Board ID: " + ReadInfoEEPROM(8, 8)); 
        break;

      case 12:
        MSGOutput("Station ID: " + ReadInfoEEPROM(16, 8));
        break;

      case 13:
        MSGOutput("Server ID: " + String(server_ID)); 
        break;

      case 14:
        MSGOutput("Server Pw: " + String(server_Password)); 
        break;

      case 18: /* Set Target IL */
        if (true)
        {
          StopValue = Target_IL;

          Serial.println("Update Target IL : " + WR_EEPROM(72, String(Target_IL)));
        }
        cmd_No = 0;
        break;

      case 19: /* Get Ref */
        if (true)
        {
          digitalWrite(Tablet_PD_mode_Trigger_Pin, false); //false is PD mode, true is Servo mode
          delay(3);

          averagePDInput = 0;
          for (int i = 0; i < 30; i++)
            averagePDInput += ads.readADC_SingleEnded(0);

          averagePDInput = (averagePDInput / 30);

          ref_Dac = averagePDInput;
          ref_IL = ILConverter(averagePDInput);

          CleanEEPROM(0, 8); //Clean EEPROM(int startPosition, int datalength)

          WriteInfoEEPROM(String(averagePDInput), 0); //Write Data to EEPROM (data, start_position)
          EEPROM.commit();

          Serial.println("Ref_Dac: " + ReadInfoEEPROM(0, 8) + ", Ref_IL: " + String(ref_IL)); //Reading Data from EEPROM(start_position, data_length)

          MSGOutput("EEPROM(" + String(0) + ") - " + ReadInfoEEPROM(0, 8)); // For update HMI ref value

          digitalWrite(Tablet_PD_mode_Trigger_Pin, true); //false is PD mode, true is Servo mode
          delay(3);

          isLCD = true;
        }

        cmd_No = 0;
        break;

      case 20:
        if(true)
        {
          server_ID = ReadInfoEEPROM(88, 32);
          server_Password = ReadInfoEEPROM(120, 32);

          if (Contains(server_ID, "??") || server_ID == "")
          {
            server_ID = "GFI-ESP32-Access-Point";
          }

          if (Contains(server_Password, "??"))
          {
            server_Password = "22101782";
          }

          Serial.println("Server ID: " + server_ID);
          Serial.println("Server Password: " + server_Password);

          WiFi.begin(server_ID.c_str(), server_Password.c_str());
          Serial.println("Connecting");

          int wifiConnectTime = 0;
          while (WiFi.status() != WL_CONNECTED)
          {
            delay(300);
            Serial.print(".");

            wifiConnectTime += 300;
            if (wifiConnectTime > 2400)
              break;
          }

          if (wifiConnectTime <= 2400)
          {
            Serial.println("");
            Serial.print("Connected to WiFi network with IP Address:");
            Serial.println(WiFi.localIP());
            isWiFiConnected = true;
          }
          else
          {
            Serial.println("Connected to WiFi network failed");
          }
        }
        cmd_No = 0;
        break;

      case 21:
        isGetPower = true;
        // GetPower_Mode = 1;

        Serial.println("Cmd: Get IL On");
        digitalWrite(Tablet_PD_mode_Trigger_Pin, false); //false is PD mode, true is Servo mode

        cmd_No = 0;
        break;

      case 22:
        isGetPower = false;

        Serial.println("Cmd: Get Power Off");
        digitalWrite(Tablet_PD_mode_Trigger_Pin, true); //false is PD mode, true is Servo mode

        cmd_No = 0;
        break;

      case 23:
        GetPower_Mode = 1;
        Serial.println("Cmd: Get Power Mode: IL(dB)");
        cmd_No = 0;
        break;

      case 24:
        GetPower_Mode = 2;
        Serial.println("Cmd: Get Power Mode: Dac");
        cmd_No = 0;
        break;

      case 25:
        GetPower_Mode = 3;
        Serial.println("Cmd: Get Power Mode: Row IL(dBm)");
        cmd_No = 0;
        break;

      case 26:
        GetPower_Mode = 4;
        Serial.println("Cmd: Get Power Mode: Row Dac");
        cmd_No = 0;
        break;

      case 27:
        Serial.println(String(Cal_PD_Input_Row_Dac(Get_PD_Points)));
        cmd_No = 0;
        break;

      case 28:
        for (int i = 0; i < 511; i = i + 8)
        {
          if(i==88)
          {
            MSGOutput("EEPROM(" + String(i) + ") - " + ReadInfoEEPROM(i, 32)); //Server ID
            i=120;
          }
          else if(i==120)
          {
            MSGOutput("EEPROM(" + String(i) + ") - " + ReadInfoEEPROM(i, 32)); //Server Password
            i=152;
          }
          else
            MSGOutput("EEPROM(" + String(i) + ") - " + ReadInfoEEPROM(i, 8)); //Reading EEPROM(int start_position, int data_length)
        }

      case 29: /* Get XYZ Position */
        DataOutput(false);
        cmd_No = 0;
        break;

      case 31:
        isLCD = true;
        LCD_Update_Mode = 100;
        Serial.println("LCD Re-Start");
        cmd_No = 0;
        break;

      // case 51: /* Get ID */
      //   Serial.println(ReadInfoEEPROM(8, 8));
      //   cmd_No = 0;
      //   break;
      }
    }

    //Functions: Motion
    if (cmd_No > 100)
      switch (cmd_No)
      {
        // Function: Cont-------------------------------------------------------------------------
        //Z feed - cont
      case 101:
        while (true)
        {
          MotorCC = MotorCC_Z;
          Move_Motor_Cont(Z_DIR_Pin, Z_STP_Pin, false, 400, delayBetweenStep_Z);
          MotorCC_Z = false;

          if (cmd == "")
          {
            if (digitalRead(R_3))
              break;
          }
          else
          {
            if (Serial.available())
            {
              String msg = Serial.readString();
              if (Contains(msg, "0"))
                break;
            }
          }
        }
        DataOutput();
        // Serial.println("Position : " + String(X_Pos_Now) + ", " + String(Y_Pos_Now) + ", " + String(Z_Pos_Now));
        cmd_No = 0;
        break;
      case 103:
        while (true)
        {
          MotorCC = MotorCC_Z;
          Move_Motor_Cont(Z_DIR_Pin, Z_STP_Pin, true, 400, delayBetweenStep_Z);
          MotorCC_Z = true;

          // DataOutput();

          if (cmd == "")
          {
            if (digitalRead(R_3))
              break;
          }
          else
          {
            if (Serial.available())
            {
              String msg = Serial.readString();
              if (Contains(msg, "0"))
                break;
            }
          }
        }
        DataOutput();
        // Serial.println("Position : " + String(X_Pos_Now) + ", " + String(Y_Pos_Now) + ", " + String(Z_Pos_Now));
        cmd_No = 0;
        break;

        //X feed - cont
      case 102:

        while (true)
        {
          MotorCC = MotorCC_X;
          Move_Motor_Cont(X_DIR_Pin, X_STP_Pin, false, 400, delayBetweenStep_X);
          MotorCC_X = false;

          // DataOutput();

          if (cmd == "")
          {
            if (digitalRead(R_3))
              break;
          }
          else
          {
            if (Serial.available())
            {
              String msg = Serial.readString();
              if (Contains(msg, "0"))
                break;
            }
          }
        }
        DataOutput();
        // Serial.println("Position : " + String(X_Pos_Now) + ", " + String(Y_Pos_Now) + ", " + String(Z_Pos_Now));
        cmd_No = 0;
        break;
        //X+ - cont
      case 105:

        while (true)
        {
          MotorCC = MotorCC_X;
          Move_Motor_Cont(X_DIR_Pin, X_STP_Pin, true, 400, delayBetweenStep_X);
          MotorCC_X = true;

          // DataOutput();

          if (cmd == "")
          {
            if (digitalRead(R_2))
              break;
          }
          else
          {
            if (Serial.available())
            {
              String msg = Serial.readString();
              if (Contains(msg, "0"))
                break;
            }
          }
        }
        DataOutput();
        // Serial.println("Position : " + String(X_Pos_Now) + ", " + String(Y_Pos_Now) + ", " + String(Z_Pos_Now));
        cmd_No = 0;
        break;

        //Y- feed - cont
      case 106:

        while (true)
        {
          MotorCC = MotorCC_Y;
          Move_Motor_Cont(Y_DIR_Pin, Y_STP_Pin, false, 400, delayBetweenStep_Y);
          MotorCC_Y = false;

          // DataOutput();

          if (cmd == "")
          {
            if (digitalRead(R_2))
              break;
          }
          else
          {
            if (Serial.available())
            {
              String msg = Serial.readString();
              if (Contains(msg, "0"))
                break;
            }
          }
        }
        DataOutput();
        // Serial.println("Position : " + String(X_Pos_Now) + ", " + String(Y_Pos_Now) + ", " + String(Z_Pos_Now));
        cmd_No = 0;
        break;

      //Y+ feed - cont
      case 104:

        while (true)
        {
          MotorCC = MotorCC_Y;
          Move_Motor_Cont(Y_DIR_Pin, Y_STP_Pin, true, 400, delayBetweenStep_Y);
          MotorCC_Y = true;

          // DataOutput();

          if (cmd == "")
          {
            if (digitalRead(R_2))
              break;
          }
          else
          {
            if (Serial.available())
            {
              String msg = Serial.readString();
              if (Contains(msg, "0"))
                break;
            }
          }
        }

        DataOutput();
        // Serial.println("Position : " + String(X_Pos_Now) + ", " + String(Y_Pos_Now) + ", " + String(Z_Pos_Now));
        cmd_No = 0;
        break;

        // Function: Jog-------------------------------------------------------------------------

      //X+ feed - jog
      case 107:
        MotorCC = MotorCC_X;
        Move_Motor_Cont(X_DIR_Pin, X_STP_Pin, false, 400, delayBetweenStep_X);
        MotorCC_X = true;

        break;

        //X- feed - jog
      case 108:
        MotorCC = MotorCC_X;
        Move_Motor_Cont(X_DIR_Pin, X_STP_Pin, true, 400, delayBetweenStep_X);
        MotorCC_X = false;

        break;

      //Y+ feed - jog
      case 109:
        MotorCC = MotorCC_Y;
        Move_Motor_Cont(Y_DIR_Pin, Y_STP_Pin, false, 400, delayBetweenStep_Y);
        MotorCC_Y = true;
        break;

      //Y- feed - jog
      case 110:
        MotorCC = MotorCC_Y;
        Move_Motor_Cont(Y_DIR_Pin, Y_STP_Pin, true, 400, delayBetweenStep_Y);
        MotorCC_Y = false;
        break;

        //Z+ feed - jog
      case 111:
        MotorCC = MotorCC_Z;
        Move_Motor_Cont(Z_DIR_Pin, Z_STP_Pin, true, 400, delayBetweenStep_Z);
        MotorCC_Z = true;
        break;

        //Z- feed - jog
      case 112:
        MotorCC = MotorCC_Z;
        Move_Motor_Cont(Z_DIR_Pin, Z_STP_Pin, false, 400, delayBetweenStep_Z);
        MotorCC_Z = false;
        break;

        //Go Home
      case 130:
        Move_Motor_abs_all(0, 0, 0);
        break;
      }
  }

  return cmd_No;
}

//------------------------------------------------------------------------------------------------------------------------------------------

void BLE_Function(String cmd)
{
  //Bluetooth : Receive Data
  // if (cmd == "" && cmd_No == 0)
  // {
  //   // Serial.println("BLE mode");
  //   if (BT.connected(30))
  //   {
  //     isMsgShow = true;

  //     if (BT.available())
  //     {
  //       String BTdata = BT.readString();

  //       BT.println(BTdata);
  //       Serial.println(BTdata);

  //       if (BTdata == "Z+")
  //       {
  //         Move_Motor(Z_DIR_Pin, Z_STP_Pin, true, 500, 8, 150, true);
  //       }
  //       else if (BTdata == "Z-")
  //       {
  //         Move_Motor(Z_DIR_Pin, Z_STP_Pin, true, 500, 8, 150, true);
  //       }
  //     }
  //   }
  // }

  // if (ButtonSelected < 0 && cmd == "")
  // {
  //   cmd_No = 0;
  // }
}

//------------------------------------------------------------------------------------------------------------------------------------------

// bool isWiFiConnected = false;
void CMDOutput(String cmd)
{
  String msg = "CMD::" + cmd;
  Serial.println(msg);

  // if (isWiFiConnected)
  // {
  //   httpTestRequest(ServerIP.c_str(), msg.c_str());
  // }

  // Check WiFi connection status
  // if (WiFi.status() == WL_CONNECTED)
  // {
  //   // Data = ServerIP + "?" + ID + "=" + msg;
  //   // httpTestRequest(Data.c_str()); //Send message to server
  //   httpTestRequest(ServerIP.c_str(), msg.c_str());

  //   isWiFiConnected = true;
  // }
  // else
  // {
  //   if (isWiFiConnected)
  //   {
  //     Serial.println("WiFi Disconnected");
  //     isWiFiConnected = false;
  //   }
  // }
}

void DataOutput()
{
  double IL = Cal_PD_Input_IL(Get_PD_Points);
  Serial.println("Position:" + String(X_Pos_Now) + "," + String(Y_Pos_Now) + "," + String(Z_Pos_Now) + "," + String(IL));
}

void DataOutput(bool isIL)
{
  if (isIL)
  {
    double IL = Cal_PD_Input_IL(Get_PD_Points);
    // Serial.println("Position : " + String(X_Pos_Now) + ", " + String(Y_Pos_Now) + ", " + String(Z_Pos_Now) + ", " + String(IL));
    MSGOutput("Position:" + String(X_Pos_Now) + "," + String(Y_Pos_Now) + "," + String(Z_Pos_Now) + "," + String(IL));
  }
  else
    // Serial.println("Position : " + String(X_Pos_Now) + ", " + String(Y_Pos_Now) + ", " + String(Z_Pos_Now));
    MSGOutput("Position:" + String(X_Pos_Now) + "," + String(Y_Pos_Now) + "," + String(Z_Pos_Now));
}

void DataOutput(int xyz, double pdValue)
{
  switch (xyz)
  {
  case 0:
    CMDOutput(">:" + String(X_Pos_Now) + "," + String(pdValue));
    break;

  case 1:
    CMDOutput(">:" + String(Y_Pos_Now) + "," + String(pdValue));
    break;

  case 2:
    CMDOutput(">:" + String(Z_Pos_Now) + "," + String(pdValue));
    break;
  }
}

long Get_Position(int xyz)
{
  switch (xyz)
  {
  case 0:
    return X_Pos_Now;
    break;

  case 1:
    return Y_Pos_Now;
    break;

  case 2:
    return Z_Pos_Now;
    break;
  }
}

void MSGOutput(String msg)
{
  Serial.println(msg);

  // Check WiFi connection status
  // if (isWiFiConnected)
  // {
  //   httpTestRequest(ServerIP.c_str(), msg.c_str());
  // }
  // if (WiFi.status() == WL_CONNECTED)
  // {
  //   // Data = ServerIP + "?" + ID + "=" + msg;
  //   // httpTestRequest(Data.c_str()); //Send message to server

  //   httpTestRequest(ServerIP.c_str(), msg.c_str());

  //   isWiFiConnected = true;
  // }
  // else
  // {
  //   if (isWiFiConnected)
  //   {
  //     Serial.println("WiFi Disconnected");
  //     isWiFiConnected = false;
  //   }
  // }
}
