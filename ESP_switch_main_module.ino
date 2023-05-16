//#include <ESP8266WiFi.h>        /*WiFi driver header*/
#include <ESP8266mDNS.h>        /*mDNS driver for OTA flash*/
#include <WiFiUdp.h>            /*UDP driver for OTA flash*/
#include <ArduinoOTA.h>         /*OTA Flash driver*/
#include <EEPROM.h>             /*EEPROM emulation driver*/


/*====================SUPLA includes=======================*/
#include <SuplaDevice.h>
#include <supla/control/virtual_relay.h>  /*Supla Virtual Relay class*/
#include <supla/network/esp_wifi.h>       /*Supla WiFi network driver*/
#include <supla/storage/littlefs_config.h>/*Supla Flash Filesystem*/
#include <supla/control/button.h>         /*Supla button class*/
/*Webserver includes*/
#include <supla/device/status_led.h>
#include <supla/network/esp_web_server.h>
#include <supla/network/html/device_info.h>
#include <supla/network/html/protocol_parameters.h>
#include <supla/network/html/status_led_parameters.h>
#include <supla/network/html/wifi_parameters.h>
#include <supla/events.h>
//#include <supla/storage/eeprom.h>

/*=========================================================*/

/*====================GPIO defines=========================*/
#define BUTTON_PIN 0                      /*Built in button GPIO 0*/
#define EXT_SWITCH_PIN 4                  /*External switch pin connector GPIO*/
#define RELAY_PIN 12                          /*Built in relay GPIO*/
#define STATUS_LED_PIN 13                     /*WiFi status LED GPIO*/

/*=========================================================*/

/*====================DEFINE constants=====================*/
#define RELAY_STATE_MEMORY_ADDR               /*EEPROM address for Relay status*/
#define START_SIGN '!'                        /*Start sign in HTTP commands*/
#define END_SIGN '$'                          /*End sign in HTTP commands*/
#define STATUS_COMMAND "S="                   /*Status start in HTTP command*/
#define IP_COMMAND "I="                       /*IP in HTTP command*/
#define SWITCH_DEBOUNCE ((uint16_t)500)       /*Contact debounce time in ms*/
#define CLIENT_BUFFER_SIZE 60                 /*Buffer size for wifi client*/
#define CONTROL_PORT 8000                     /*Port for server/client communication */
#define SLAVE_CHECK_TIME 60000                /*Time between slave status check*/
#define CLIENT_TIMEOUT 3000                   /*Time out for client response*/
#define CONNECT_RETRY 2                       /*Retry times if no connection to slave*/

/*=========================================================*/

/*====================MACRO================================*/
#define EXT_SWITCH_ON (!digitalRead(EXT_SWITCH_PIN))
#define SWITCH_RELAY_ON digitalWrite(RELAY_PIN, HIGH)
#define SWITCH_RELAY_OFF digitalWrite(RELAY_PIN, LOW)
#define SWITCH_STATUS_LED_ON digitalWrite(STATUS_LED_PIN, HIGH)
#define SWITCH_STATUS_LED_OFF digitalWrite(STATUS_LED_PIN, LOW)

/*=========================================================*/


/*===================Typedefs==============================*/

typedef enum Relay_state_Tag
{
  RELAY_OFF,                                        /*Relay set to off status*/
  RELAY_EXT_ON,                                     /*Relay switched on from HTTP/SUPLA/Slave Button*/
  RELAY_SWITCH_ON,                                  /*Relay switched on from external bistable switch*/
}
Relay_State_T;

typedef enum Check_state_Tag
{
  SAME_STATE,                                               /*Relay set to off status*/
  DIFF_STATE,                                       /*Relay switched on from HTTP/SUPLA/Slave Button*/
  CONNECTION_ERROR,                                 /*Relay switched on from external bistable switch*/
}
Check_state_T;

/*=========================================================*/

/*===============Global constants and variables============*/
uint32_t Last_Switch_Change = 0;                    /*Time in ms when ext switch was changed*/
bool Last_Switch_State = false;                     /*External Switch state memory*/
bool Slave_Is_Actual = false;                       /*Slave relay state set with no error*/
uint32_t Last_Slave_Check = 0;                      /*Time in ms when Slave status was checked*/
Relay_State_T Relay_State;                          /*Actual Relay state*/
const char* Ssid = "SSID";                 /* WIFI network Ssid*/
const char* Password = "PASS";              /* WIFI network Password */
unsigned char ip[4] = {192, 168, 1, 80};
/*Supla global network objects*/
//Supla::Eeprom eeprom;
//Supla::ESPWifi wifi;
Supla::ESPWifi wifi(Ssid, Password, ip);
Supla::Device::StatusLed statusLed(STATUS_LED_PIN, true); // inverted state

/*Supla global custom objects*/
Supla::Control::VirtualRelay Supla_Relay;             /*Supla virtual relay for control real module relay*/
Supla::Control::Button Config_Button(BUTTON_PIN, true, true); /*Supla config button object*/

/*User global network objects*/
WiFiServer Control_Server(CONTROL_PORT);                       /*Server object on port 8000*/
//IPAddress Slave_Host(192, 168, 0, 227);
const char* Slave_Host = "ESP-88B5F9.lan";                 /*Slave module host*/
IPAddress Master_IP(192, 168, 1, 81);
IPAddress Slave_IP(192, 168, 1, 80);

/*=========================================================*/

/*====================Functions prototypes=================*/
void Server_Loop(void);                                /*Function to check and receive HTTP commands*/
void Set_Relay_State(uint8_t new_state);               /*Function to set new relay state*/
void Switch_Check(void);                               /*Function to check external switch state and change relay state*/
bool Set_Slave_Relay(uint8_t state);                   /*Function to set Slave Relay*/
Check_state_T Slave_Check(void);                       /*Function to check Slave_State*/

/*=========================================================*/

void setup() {
  uint8_t EEPROM_Value;
  char GUID[SUPLA_GUID_SIZE] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  // Replace the following AUTHKEY with value that you can retrieve from: https://www.supla.org/arduino/get-authkey
  char AUTHKEY[SUPLA_AUTHKEY_SIZE] = {0x00, 0x00, 0x00, 0x2A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  ESP.wdtEnable(8000);
  Serial.begin(115200);                                /*Start serial port 0*/
  pinMode(RELAY_PIN, OUTPUT);                          /*Set Relay GPIO as output*/
  pinMode(EXT_SWITCH_PIN, INPUT_PULLUP);               /*Set External switch GPIO*/
  delay(100);

  /*Supla config button configuration*/
  Config_Button.setHoldTime(1000);
  Config_Button.setMulticlickTime(300);
  Config_Button.addAction(Supla::TOGGLE, Supla_Relay, Supla::ON_CLICK_1);
  Config_Button.addAction(Supla::TOGGLE_CONFIG_MODE, SuplaDevice, Supla::ON_HOLD);



  ArduinoOTA.setHostname("main_switch");
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  /*ArduinoOTA setup START*/
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    ESP.wdtFeed();
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  /*ArduinoOTA setup END*/
  /*begin Supla device*/
  SuplaDevice.begin(GUID,              // Global Unique Identifier
                    "116.203.221.202",  // SUPLA server address
                    "mail@mail",   // Email address used to login to Supla Cloud
                    AUTHKEY);
  Control_Server.begin();
  SuplaDevice.iterate();
  ESP.wdtFeed();
  EEPROM.begin(16);                                    /*Start EEPROM emulation*/
  EEPROM.get(0, EEPROM_Value);                         /*Read last switch status*/
  if (EEPROM_Value != 0)                               /*Copy EEPROM value to Last_Switch_State*/
  {
    Last_Switch_State = true;
  }
  else
  {
    Last_Switch_State = false;
  }
  if (Last_Switch_State == EXT_SWITCH_ON)              /*Compare if there was a switch status change during reboot*/
  {
    EEPROM.get(1, EEPROM_Value);                       /*If no change, then read last Relay_State saved in EEPROM*/
    if (EEPROM_Value <= 2)
    {
      Set_Relay_State((Relay_State_T)EEPROM_Value);
    }
    else
    {
      Set_Relay_State(RELAY_OFF);
    }
  }
  else
  {
    Switch_Check();
  }
  MDNS.addService("http", "tcp", CONTROL_PORT);



}

void loop() {
  // put your main code here, to run repeatedly:
  uint32_t current_time = millis();
  uint32_t check_interval;
  ESP.wdtFeed();
  ArduinoOTA.handle();
  SuplaDevice.iterate();
  if (Supla_Relay.isOn() && Relay_State == RELAY_OFF)
  {
    Set_Relay_State(RELAY_EXT_ON);
  }
  else if (!Supla_Relay.isOn() && Relay_State != RELAY_OFF)
  {
    Set_Relay_State(RELAY_OFF);
  }
  Server_Loop();
  Switch_Check();
}

void Server_Loop() {
  WiFiClient client = Control_Server.available();         /*Server client object*/
  char client_buffer[CLIENT_BUFFER_SIZE];                 /*Buffer for client data*/
  char server_response[CLIENT_BUFFER_SIZE];               /*Buffer for response to client*/
  const char http_header[] = {"HTTP/1.1 200 SAME_STATE\r\n"};     /*HTTP header response*/
  uint8_t i = 0;                                          /*Client sign counter*/
  char* command_start = NULL;                             /*Pointer for command start*/
  uint8_t command;                                        /*Command from client*/
  uint32_t timeout;                                       /*Client response timeout*/
  if (!client)                                            /*No client connected*/
  {
    return;
  }
  timeout = millis();                                     /*client timeout*/
  while (!client.available() && ((millis() - timeout) < 3000))
  {
    delay(1);
  }
  if ((millis() - timeout) > 3000)
  {
    client.flush();
    client.stop();
    return;
  }

  while (client.available() > 0)
  {
    client_buffer[i] = client.read();
    Serial.write(client_buffer[i]);
    i++;
    if (i >= CLIENT_BUFFER_SIZE)
    {
      break;
    }
  }
  client.flush();
  command_start = strchr(client_buffer, START_SIGN);
  while (command_start != NULL)
  {
    if ((strncmp((command_start + 1), STATUS_COMMAND, 2)) == 0)
    {
      command = *(command_start + 3) - 48;       /*Convert ASCII sign to digit*/
      if ((command >= 0) && (command < 3))  /*Correct command*/
      {
        Set_Relay_State((Relay_State_T)command);
        sprintf(server_response, "%s\n\rRelay Set", http_header);
        client.print(server_response);
        client.flush();
        delay(10);
        client.stop();
        return;
      }
      else
      {
        sprintf(server_response, "%s\n\rWrong command", http_header);
        client.print(server_response);
        client.flush();
        delay(10);
        client.stop();
        return;
      }

    }
    else
    {
      command_start = strchr((command_start + 1), START_SIGN);
    }
  }
  sprintf(server_response, "%s\n\rNo command", http_header);
  client.print(server_response);
  client.flush();
  delay(10);
  client.stop();
  return;

}

void Set_Relay_State(Relay_State_T new_state) {
  if (Relay_State == new_state)              /*no changes, do nothing*/
  {
    return;
  }
  else
  {
    switch (new_state) {
      case RELAY_OFF:
        {
          SWITCH_RELAY_OFF;
          Supla_Relay.turnOff(0);
          break;
        }
      case RELAY_EXT_ON:
        {
          SWITCH_RELAY_ON;
          Supla_Relay.turnOn(0);
          break;
        }
      case RELAY_SWITCH_ON:
        {
          SWITCH_RELAY_ON;
          Supla_Relay.turnOn(0);
          break;
        }


    }
    Relay_State = new_state;
    //Slave_Is_Actual = Set_Slave_Relay((uint8_t)Relay_State);
    EEPROM.put(1, (uint8_t)Relay_State);
    EEPROM.commit();
  }
}

void Switch_Check() {
  uint32_t current_time = millis();
  if ((current_time - Last_Switch_Change) > SWITCH_DEBOUNCE)
  {
    Last_Switch_Change = millis();
    if ((Last_Switch_State == false) && EXT_SWITCH_ON)
    {
      Last_Switch_State = true;
      Set_Relay_State(RELAY_SWITCH_ON);
      EEPROM.put(0, (uint8_t)1);
      EEPROM.commit();
    }
    else if ((Last_Switch_State == true) && !EXT_SWITCH_ON)
    {
      Last_Switch_State = false;
      Set_Relay_State(RELAY_OFF);
      EEPROM.put(0, (uint8_t)0);
      EEPROM.commit();
    }
  }

}

bool Set_Slave_Relay(uint8_t state) {
  WiFiClient client;
  char server_response[CLIENT_BUFFER_SIZE];               /*Buffer for response to client*/
  uint8_t retry_count = 0;
  while (retry_count < CONNECT_RETRY)
  {
    Switch_Check();
    retry_count++;
    ESP.wdtFeed();
    if (client.connect(Slave_IP, CONTROL_PORT))
    {
      client.flush();
      sprintf(server_response, "!S=%d", state);
      client.print(server_response);
      client.println('$');
      client.flush();
      client.stop();
      Last_Slave_Check = millis();
      Serial.println(F("OK"));
      return true;
    }

  }

  client.stop();
  Serial.println(F("Slave relay set error"));

  return false;
}


Check_state_T Slave_Check() {
  WiFiClient client;
  char client_buffer[CLIENT_BUFFER_SIZE] = {0};                 /*Buffer for client data*/
  uint8_t i = 0;
  unsigned long start_time = millis();
  Relay_State_T slave_status;
  char* command_start = NULL;                             /*Pointer for command start*/
  uint8_t retry_count = 0;
  while (retry_count < CONNECT_RETRY)
  {    
    ESP.wdtFeed();
    if (client.connect(Slave_IP, CONTROL_PORT))
    {
      client.flush();
      client.println(F("GET /!S=?$"));
      break;
    }
    retry_count++;
  }

  if (retry_count >= 5)
  {
    Serial.println(F("Cannot connect"));
    client.stop();
    return CONNECTION_ERROR;
  }

  while (!client.available() && (millis() - start_time < CLIENT_TIMEOUT)) {
    delay(1);
  }
  ESP.wdtFeed();
  if ((millis() - start_time >= CLIENT_TIMEOUT)) {
    client.flush();
    client.stop();
    return CONNECTION_ERROR;
  }
  while (client.available() > 0 && i <= sizeof(client_buffer)) {
    client_buffer[i] = client.read();         // read the incoming data
    i++;
  }
  client_buffer[i] = '\0';
  client.flush();
  client.stop();
  command_start = strchr(client_buffer, START_SIGN);
  while (command_start != NULL) {
    if ((strncmp((command_start + 1), STATUS_COMMAND, 2)) == 0)
    {
      slave_status = (Relay_State_T)(*(command_start + 3) - 48); /*Convert ASCII sign to digit*/
      if (slave_status == Relay_State)
      {
        return SAME_STATE;
      }
      else
      {
        return DIFF_STATE;
      }
    }
    else
    {
      command_start = strchr((command_start + 1), START_SIGN);
    }
  }
  Serial.println(F("Connected but no answer"));

  return DIFF_STATE;

}
