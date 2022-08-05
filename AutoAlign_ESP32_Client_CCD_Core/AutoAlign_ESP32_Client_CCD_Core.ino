// #include <avr/wdt.h> //使用看門狗計時器的含括檔
// #include <esp_task_wdt.h>
// #include <FLASHLED.h>
// #include <ArduinoSort.h>
// #include <U8glib.h>
// #include <U8g2lib.h>
// #include <BluetoothSerial.h>
#include <WiFi.h>
#include <esp_now.h>
#include <EEPROM.h>

TaskHandle_t Task_1;
#define WDT_TIMEOUT 3

// Set your access point network credentials

bool isWiFiConnected = false;

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

int direction_X = 0;
int direction_Y = 0;
int direction_Z = 0;

int xyz = 0;

long X_Pos_Record = 0;
long Y_Pos_Record = 0;
long Z_Pos_Record = 0;
long X_Pos_Now = 0;
long Y_Pos_Now = 0;
long Z_Pos_Now = 0;
long Z_Pos_reLoad = 0;

typedef struct struct_Pos_Data {
    long X_Pos = 0;
    long Y_Pos = 0;
    long Z_Pos = 0;
} struct_Pos_Data;

// Create a struct_message
struct_Pos_Data Pos_1_Record;
struct_Pos_Data Pos_2_Record;
struct_Pos_Data Pos_3_Record;
struct_Pos_Data Pos_4_Record;

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
    delayBetweenStep = delayBetweenStep_X;
    axis = "X";
    break;
  case 1:
    MotorDir_Pin = Y_DIR_Pin;
    MotorSTP_Pin = Y_STP_Pin;
    Pos_Now = Y_Pos_Now;
    delayBetweenStep = delayBetweenStep_Y;
    axis = "Y";
    break;
  case 2:
    MotorDir_Pin = Z_DIR_Pin;
    MotorSTP_Pin = Z_STP_Pin;
    Pos_Now = Z_Pos_Now;
    delayBetweenStep = delayBetweenStep_Z;
    axis = "Z";
    break;
  }

  if (Target - Pos_Now < 0)
    MotorCC = false;
  else if (Target - Pos_Now > 0)
    MotorCC = true;
  else
    return;

  MinMotroStep = abs(Target - Pos_Now);

  MSGOutput(axis + " Go to position: " + String(Target) + ", origin position: " + String(Pos_Now));

  Move_Motor(MotorDir_Pin, MotorSTP_Pin, MotorCC, MinMotroStep, delayBetweenStep, 0);
}

void Move_Motor_abs_all(int x, int y, int z)
{
  Move_Motor_abs(0, x);
  Move_Motor_abs(1, y);
  Move_Motor_abs(2, z);
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

    if(!digitalRead(R_0))
    {
      keyValueSum += 1000;
    }

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

    case 1001:
      keyNo = keyNo; /* Z- */
      break;
    case 1002:
      keyNo = keyNo; /* X+ */
      break;
    case 1003:
      keyNo = keyNo; /* Z+ */
      break;
    case 1004:
      keyNo = keyNo; /* Y+ */
      break;
    case 1005:
      keyNo = keyNo; /* X- */
      break;
    case 1006:
      keyNo = keyNo; /* Y- */
      break;
    case 1011:
      keyNo = 1007; /* 7 */
      break;
    case 1012:
      keyNo = 1008; /* 8 */
      break;
    case 1013:
      keyNo = 1009; /* 9 */
      break;

    default:
      keyNo = -1;
      break;
    }

    isKeyPressed = false;
  }

  return keyNo;
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

int Emer_Count = 0;
void EmergencyStop()
{
  isStop = true;

  Serial.println("EmergencyStop");
  Emer_Count ++;
  digitalWrite(Tablet_PD_mode_Trigger_Pin, true); //false is PD mode, true is Servo mode

  if(Emer_Count > 40)
  {
    X_Pos_Now = 0;
    Y_Pos_Now = 0;
    Z_Pos_Now = 0;
    Serial.println("Reset Position");
    Emer_Count=0;
  }

  isLCD = true;
  PageLevel = 0;
}
//------------------------------------------------------------------------------------------------------------------------------------------
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
    
  pinMode(R_2, INPUT_PULLUP); // /keyValue:5
  pinMode(R_3, INPUT_PULLUP); // /keyValue:0

  pinMode(C_1, OUTPUT); ///keyValue:1
  pinMode(C_2, OUTPUT); ///keyValue:2
  pinMode(C_3, OUTPUT); ///keyValue:3

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

  eepromString = ReadInfoEEPROM(24, 8);
  direction_X = eepromString.toInt();
  MSGOutput("direction_X: " + String(direction_X));

  eepromString = ReadInfoEEPROM(32, 8);
  direction_Y = eepromString.toInt();
  MSGOutput("direction_Y: " + String(direction_Y));

  eepromString = ReadInfoEEPROM(40, 8);
  direction_Z = eepromString.toInt();
  MSGOutput("direction_Z: " + String(direction_Z));

  eepromString = ReadInfoEEPROM(48, 8);
  delayBetweenStep_X = eepromString.toInt();
  MSGOutput("delayBetweenStep_X: " + String(delayBetweenStep_X));

  eepromString = ReadInfoEEPROM(56, 8);
  delayBetweenStep_Y = eepromString.toInt();
  MSGOutput("delayBetweenStep_Y: " + String(delayBetweenStep_Y));

  eepromString = ReadInfoEEPROM(64, 8);
  delayBetweenStep_Z = eepromString.toInt();
  MSGOutput("delayBetweenStep_Z: " + String(delayBetweenStep_Z));

  #pragma endregion
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
    ButtonSelected = KeyValueConverter();

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

void Move_Motor(byte dir_pin, byte stp_pin, bool dirt, long moveSteps, int delayStep, int stableDelay)
{
  MotorCC = dirt;
  digitalWrite(dir_pin, dirt);
  delay(2);
  if (moveSteps > 0)
  {
    step(stp_pin, moveSteps, delayStep);
    delay(stableDelay);
    DataOutput();
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
      DataOutput();
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
   
    //Set Motor Step Ratio
    // else if (Contains(cmd, "Set_MotorStepRatio:"))
    // {
    //   cmd.remove(0, 19);
    //   MotorStepRatio = WR_EEPROM(208, cmd).toDouble();
    //   MSGOutput("Set_MotorStepRatio:" + String(MotorStepRatio));
    // }

    else if (Contains(cmd, "ID?"))
    {
      Serial.println("CCD");
    }

    // //Set Manual Control Motor Speed
    else if (Contains(cmd, "DIR "))
    {
      cmd.remove(0, 4);

      if (Contains(cmd, "X"))
      {
        cmd.remove(0, 2);
        direction_X = cmd.toInt();
        WR_EEPROM(24, cmd);
        Serial.println("Motor Direction X:" + String(direction_X));
      }
      else if (Contains(cmd, "Y"))
      {
        cmd.remove(0, 2);
        direction_Y = cmd.toInt();
        WR_EEPROM(32, cmd);
        Serial.println("Motor Direction Y:" + String(direction_Y));
      }
      else if (Contains(cmd, "Z"))
      {
        cmd.remove(0, 2);
        direction_Z = cmd.toInt();
        WR_EEPROM(40, cmd);
        Serial.println("Motor Direction Z:" + String(direction_Z));
      }
    }

    // //Set Manual Control Motor Speed
    else if (Contains(cmd, "SPD "))
    {
      cmd.remove(0, 4);

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

      Serial.println("Set Motor Speed:" + cmd);
    }

    // //Set Manual Control Motor Speed
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

     else if (Contains(cmd, "SAVE "))
     {
        cmd.remove(0, 5);

        if (Contains(cmd, "1"))
        {
          Pos_1_Record.X_Pos = X_Pos_Now;
          Pos_1_Record.Y_Pos = Y_Pos_Now;
          Pos_1_Record.Z_Pos = Z_Pos_Now;
          WR_EEPROM(72, String(X_Pos_Now));
          WR_EEPROM(80, String(Y_Pos_Now));
          WR_EEPROM(88, String(Z_Pos_Now));
        }
        else if (Contains(cmd, "2"))
        {
          Pos_2_Record.X_Pos = X_Pos_Now;
          Pos_2_Record.Y_Pos = Y_Pos_Now;
          Pos_2_Record.Z_Pos = Z_Pos_Now;
          WR_EEPROM(96, String(X_Pos_Now));
          WR_EEPROM(104, String(Y_Pos_Now));
          WR_EEPROM(112, String(Z_Pos_Now));
        }
        else if (Contains(cmd, "3"))
        {
          Pos_3_Record.X_Pos = X_Pos_Now;
          Pos_3_Record.Y_Pos = Y_Pos_Now;
          Pos_3_Record.Z_Pos = Z_Pos_Now;
          WR_EEPROM(120, String(X_Pos_Now));
          WR_EEPROM(128, String(Y_Pos_Now));
          WR_EEPROM(136, String(Z_Pos_Now));
        }
        else if (Contains(cmd, "4"))
        {
          Pos_4_Record.X_Pos = X_Pos_Now;
          Pos_4_Record.Y_Pos = Y_Pos_Now;
          Pos_4_Record.Z_Pos = Z_Pos_Now;
          WR_EEPROM(144, String(X_Pos_Now));
          WR_EEPROM(152, String(Y_Pos_Now));
          WR_EEPROM(160, String(Z_Pos_Now));
        }

        Serial.println("Save Pos");
        Serial.println("X:" + String(X_Pos_Now));
        Serial.println("Y:" + String(Y_Pos_Now));
        Serial.println("Z:" + String(Z_Pos_Now));
     }

    //Command No.
    else if (Contains(cmd, "cmd"))
    {
      cmd.remove(0, 3);
      cmd_No = cmd.toInt();
      delay(10);
    }
  }
  else if (ButtonSelected >= 0)
  {
    Emer_Count = 0;   //Reset Emergency Count 
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

    case 1007:
      Pos_1_Record.X_Pos = X_Pos_Now;
      Pos_1_Record.Y_Pos = Y_Pos_Now;
      Pos_1_Record.Z_Pos = Z_Pos_Now;
      WR_EEPROM(72, String(X_Pos_Now));
      WR_EEPROM(80, String(Y_Pos_Now));
      WR_EEPROM(88, String(Z_Pos_Now));
      Serial.println("Save Pos 1");
      break;

    case 1008:
      Pos_2_Record.X_Pos = X_Pos_Now;
      Pos_2_Record.Y_Pos = Y_Pos_Now;
      Pos_2_Record.Z_Pos = Z_Pos_Now;
      WR_EEPROM(96, String(X_Pos_Now));
      WR_EEPROM(104, String(Y_Pos_Now));
      WR_EEPROM(112, String(Z_Pos_Now));
      Serial.println("Save Pos 2");
      break;

    case 1009:
      Pos_3_Record.X_Pos = X_Pos_Now;
      Pos_3_Record.Y_Pos = Y_Pos_Now;
      Pos_3_Record.Z_Pos = Z_Pos_Now;
      WR_EEPROM(120, String(X_Pos_Now));
      WR_EEPROM(128, String(Y_Pos_Now));
      WR_EEPROM(136, String(Z_Pos_Now));
      Serial.println("Save Pos 3");
      break;

    // case 1006:
    //   Pos_4_Record.X_Pos = X_Pos_Now;
    //   Pos_4_Record.Y_Pos = Y_Pos_Now;
    //   Pos_4_Record.Z_Pos = Z_Pos_Now;
    //   WR_EEPROM(144, String(X_Pos_Now));
    //   WR_EEPROM(152, String(Y_Pos_Now));
    //   WR_EEPROM(160, String(Z_Pos_Now));
    //   Serial.println("Save Pos 4");
    //   break;

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
        //Function 1: Auto Align
      case 1: /* Auto Align */
        Move_Motor_abs_all(Pos_1_Record.X_Pos, Pos_1_Record.Y_Pos, Pos_1_Record.Z_Pos);
        cmd_No = 0;
        break;

        //Function 2: Fine Scan
      case 2: /* Fine Scan */
        Move_Motor_abs_all(Pos_2_Record.X_Pos, Pos_2_Record.Y_Pos, Pos_2_Record.Z_Pos);
        cmd_No = 0;
        break;

      //Function 3: Auto Curing
      case 3: /* Auto Curing */
        Move_Motor_abs_all(Pos_3_Record.X_Pos, Pos_3_Record.Y_Pos, Pos_3_Record.Z_Pos);
        cmd_No = 0;
        break;

      case 1006: /* Auto Curing */
        Move_Motor_abs_all(Pos_4_Record.X_Pos, Pos_4_Record.Y_Pos, Pos_4_Record.Z_Pos);
        cmd_No = 0;
        break;

      case 5: /* Fine Scan X */
        cmd_No = 0;
        break;

      case 6: /* Fine Scan Y */
        cmd_No = 0;
        break;

      case 7: /* Fine Scan Z */
        cmd_No = 0;
        break;

      case 8:
       cmd_No = 0;
        break;

      case 9:
        cmd_No = 0;
        break;

      case 10:
        cmd_No = 0;
        break;

      case 11:
        break;

      case 12:
        break;

      case 13:
        break;

      case 14:
        break;

      case 18: /* Set Target IL */
        
        cmd_No = 0;
        break;

      case 19: /* Get Ref */
       
        cmd_No = 0;
        break;

      case 20:
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
        cmd_No = 0;
        break;

      case 29: /* Get XYZ Position */
        DataOutput();
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

    //Function: Motion
    if (cmd_No > 100)
      switch (cmd_No)
      {
        // Function: Cont-------------------------------------------------------------------------
        //Z feed - cont
      case 101:
        while (true)
        {
          MotorCC = MotorCC_Z;
          bool dir = direction_Z == 0 ? false : true;
          Move_Motor_Cont(Z_DIR_Pin, Z_STP_Pin, dir, 400, delayBetweenStep_Z);
          MotorCC_Z = dir;

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
        cmd_No = 0;
        break;
      case 103:
        while (true)
        {
          MotorCC = MotorCC_Z;
          bool dir = direction_Z == 0 ? true : false;
          Move_Motor_Cont(Z_DIR_Pin, Z_STP_Pin, dir, 400, delayBetweenStep_Z);
          MotorCC_Z = dir;

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
        cmd_No = 0;
        break;

        //X feed - cont
      case 102:

        while (true)
        {
          MotorCC = MotorCC_X;
          bool dir = direction_X == 0 ? false : true;
          Move_Motor_Cont(X_DIR_Pin, X_STP_Pin, dir, 400, delayBetweenStep_X);
          MotorCC_X = dir;

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
        cmd_No = 0;
        break;
        //X+ - cont
      case 105:

        while (true)
        {
          MotorCC = MotorCC_X;
          bool dir = direction_X == 0 ? true : false;
          Move_Motor_Cont(X_DIR_Pin, X_STP_Pin, dir, 400, delayBetweenStep_X);
          MotorCC_X = dir;

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
        cmd_No = 0;
        break;

        //Y- feed - cont
      case 106:

        while (true)
        {
          MotorCC = MotorCC_Y;
          bool dir = direction_Y == 0 ? false : true;
          Move_Motor_Cont(Y_DIR_Pin, Y_STP_Pin, dir, 400, delayBetweenStep_Y);
          MotorCC_Y = dir;

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
        cmd_No = 0;
        break;

      //Y+ feed - cont
      case 104:

        while (true)
        {
          MotorCC = MotorCC_Y;
          bool dir = direction_Y == 0 ? true : false;
          Move_Motor_Cont(Y_DIR_Pin, Y_STP_Pin, dir, 400, delayBetweenStep_Y);
          MotorCC_Y = dir;

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
  Serial.println("Position:" + String(X_Pos_Now) + "," + String(Y_Pos_Now) + "," + String(Z_Pos_Now));
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
