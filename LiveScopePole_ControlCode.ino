#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>                                                                  
#include <ESPmDNS.h>
#include <SPIFFS.h>
#include <WebSocketsServer.h>
#include <EEPROM.h>

#include <QMC5883LCompass.h>
#include <movingAvg.h>
#include <AccelStepper.h>

#include <ArduinoJson.h>


//Create a web server
WebServer server ( 80 );

// Create a Websocket server....
WebSocketsServer webSocket(81);

// Mode Control (MODE)
const byte qmc5883l_mode_stby = 0x00;
const byte qmc5883l_mode_cont = 0x01;
// Output Data Rate (ODR)
const byte qmc5883l_odr_10hz  = 0x00;
const byte qmc5883l_odr_50hz  = 0x04;
const byte qmc5883l_odr_100hz = 0x08;
const byte qmc5883l_odr_200hz = 0x0C;
// Full Scale Range (RNG)
const byte qmc5883l_rng_2g    = 0x00;
const byte qmc5883l_rng_8g    = 0x10;
// Over Sample Ratio (OSR)
const byte qmc5883l_osr_512   = 0x00;
const byte qmc5883l_osr_256   = 0x40;
const byte qmc5883l_osr_128   = 0x80;
const byte qmc5883l_osr_64    = 0xC0;


unsigned int state;
  
#define SEQUENCE_IDLE 0x00
#define GET_SAMPLE 0x10

#define GET_SAMPLE__WAITING 0x12

#define USE_SERIAL Serial
#define DBG_OUTPUT_PORT Serial

byte promValues[] = {0,0,0};

uint8_t remote_ip;
uint8_t socketNumber;
uint8_t send_num;
float value;
int ontime;   //On time setting from mobile web app
int offtime;  //Off time setting from mobile web app


const int DIR = 27;//12 - 21
const int STEP = 26;//14 - 22
const int SLEEP = 25;
const int M2 = 23;
const int LEFTBUTTON = 18;
const int RIGHTBUTTON = 23;
const int ENABLE = 33; //15 - 26
const int V3V3 = 12;
const int steps_per_rev = 200;

//These will need to be updated to the GPIO pins for each control circuit.
int POWER = 13;
int TIMER_SWITCH = 2; 
int WIFI_CONNECTION = 15;
int WIFI_CLIENT_CONNECTED = 16;
int Timer_LED = 17;
int SPEED = 14; 
int LEFT = 12; 
int RIGHT = 13;
const int ANALOG_PIN = A0;

int onoff = 1; 

int enable_buttonState,left_buttonState,right_buttonState = 0;
int heading_lastreading, heading_currentreading = 0;



volatile byte switch_state = HIGH;
boolean pumpOn = false;
boolean timer_state = false;
boolean timer_started = false;
boolean wifi_state = false;
boolean wifi_client_conn = false;
boolean left_button_touchstart = false;
boolean right_button_touchstart = false;
boolean enable_checkHeading = false;
int startup_state;
int ontime_value;  //number of ON minutes store in EEPROM
int offtime_value; //number of OFF minutes store in EEPROM

int Clock_seconds;

int MaxSpeed = 1000;   // ****** Adjust this setting for Max Speed of Stepper as to not miss steps    ******
int Acceleration = 800;   // ****** Adjust this setting for Acceleration of Stepper as to not miss steps ******

int    LastHeading = 0;   // Register for Last heading
int    NewHeading  = 0;   // Register for New heading
int    Movement;   // Register for difference between Last and New heading

QMC5883LCompass compass;  //Modify QMC5883LCompass.cpp to set the GPIO pins
movingAvg avgHeading(50); //was 30


AccelStepper Stepper1(AccelStepper::DRIVER,26,27);  //14,12  

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t lenght) {
    String text = String((char *) &payload[0]);
    char * textC = (char *) &payload[0];
    String voltage;
    String temp1, temp2, temp3, temp4;
    float percentage;
    float actual_voltage;  //voltage calculated based on percentage drop from 5.0 circuit.
    int nr;
    //int onof;
    uint32_t rmask;
    int i;
    
    switch(type) {

        case WStype_DISCONNECTED:
            //Reset the control for sending samples of ADC to idle to allow for web server to respond.
            USE_SERIAL.printf("[%u] Disconnected!\n", num);
            state = SEQUENCE_IDLE;
            digitalWrite(WIFI_CLIENT_CONNECTED, LOW);
            break;
        case WStype_CONNECTED:
          {
            //Display client IP info that is connected in Serial monitor and set control to enable samples to be sent every two seconds (see analogsample() function)
            IPAddress ip = webSocket.remoteIP(num);
            USE_SERIAL.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
            digitalWrite(WIFI_CLIENT_CONNECTED, HIGH);
            socketNumber = num;
            state = GET_SAMPLE;
       
            wifi_client_conn = true;
          }
            break;
 
        case WStype_TEXT:
            if (payload[0] == 'L')
                {
                  if (payload[1] == 'N')
                    {
                      left_button_touchstart = true;
                      Serial.println("Left Touchstart Active");
                    }
                  else
                  {
                    Serial.println("Turn Left OFF");
                    left_button_touchstart = false;
                    digitalWrite(ENABLE,HIGH);
                  }
               }
            if (payload[0] == 'R')
                {
                  if (payload[1] == 'N')
                     {
                      right_button_touchstart = true;
                      Serial.println("Touchstart Active");
                    }
                  else
                  {
                    Serial.println("Turn right OFF");
                    right_button_touchstart = false;
                    digitalWrite(ENABLE,HIGH);
                  }
                  
                }
                
            if (payload[0] == 'O')
                {
                USE_SERIAL.printf("[%u] Timer state event %s\n", num, payload);
                //Serial.println((char *)&payload);
                if (payload[1] == 'N')
                  {
                    startup_state = 1;
                    //EEPROM.write(0,startup_state);
                    //EEPROM.commit();
                    //byte value = EEPROM.read(0);
                    Serial.println("Enable Driver " + String(value));
                    enable_checkHeading = true;
                     digitalWrite(ENABLE,LOW);
                  }
                if (payload[2] == 'F')
                  {
                    startup_state = 0;
                    //EEPROM.write(0,startup_state);
                    //EEPROM.commit();
                    //byte value = EEPROM.read(0);
                    Serial.println("Disable Driver" + String(value));
                    enable_checkHeading = false;
                    digitalWrite(ENABLE,HIGH);
                  }
                }
                
         case WStype_BIN:
         {
            /*USE_SERIAL.printf("[%u] get binary lenght: %u\n", num, lenght);
            //hexdump(payload, lenght);
            //analogWrite(13,atoi((const char *)payload));
            Serial.printf("Payload");
            Serial.printf("[%u] Analog GPIO Control Msg: %s\n", num, payload);
            Serial.println((char *)payload);
            int temp = atoi((char *)payload);
            uint32_t time_setting = (uint32_t) strtol((const char *) &payload[1], NULL, 8);
            //analogWrite(SPEED,temp);   ******************** NEED TO CREATE ANALOG FUNCTION ***********
            //webSocket.sendTXT(num,"Got Speed Change");*/
         }
         break;
         
         case WStype_ERROR:
            USE_SERIAL.printf(WStype_ERROR + " Error [%u] , %s\n",num, payload); 
    }
}

void rotateLeft(){
  //digitalWrite(DIR, LOW);
  //Serial.println("Button is LOW");
  //digitalWrite(STEP, HIGH);
  //delayMicroseconds(1000);
  //digitalWrite(STEP, LOW);
  //delayMicroseconds(1000);
  //int i = 0;
  //i = i + 1;
  Stepper1.runToNewPosition(-10);
}

void rotateRight(){
  //digitalWrite(DIR,HIGH);
  //Serial.println("Button is HIGH");
  //digitalWrite(STEP, HIGH);
  //delayMicroseconds(1000);
  //digitalWrite(STEP, LOW);
  //delayMicroseconds(1000);
  Stepper1.runToNewPosition(10);
}

void handleRoot() {
  server.send(200, "text/html", "<h1>You are connected</h1>");
}

String getContentType(String filename){
  if(server.hasArg("download")) return "application/octet-stream";
  else if(filename.endsWith(".htm")) return "text/html";
  else if(filename.endsWith(".html")) return "text/html";
  else if(filename.endsWith(".css")) return "text/css";
  else if(filename.endsWith(".js")) return "application/javascript";
  else if(filename.endsWith(".png")) return "image/png";
  else if(filename.endsWith(".gif")) return "image/gif";
  else if(filename.endsWith(".jpg")) return "image/jpeg";
  else if(filename.endsWith(".ico")) return "image/x-icon";
  else if(filename.endsWith(".xml")) return "text/xml";
  else if(filename.endsWith(".pdf")) return "application/x-pdf";
  else if(filename.endsWith(".zip")) return "application/x-zip";
  else if(filename.endsWith(".gz")) return "application/x-gzip";
  else if(filename.endsWith(".svg")) return "image/svg+xml";
  return "text/plain";
}

bool handleFileRead(String path){
  DBG_OUTPUT_PORT.println("handleFileRead: " + path);
  if(path.endsWith("/"))
    {
      path += "relay.html";
      state = SEQUENCE_IDLE;
      File file = SPIFFS.open(path, "r");
      Serial.println("Sending relay.html ");
      Serial.println(path);
      String contentType = getContentType(path);
      size_t sent = server.streamFile(file, contentType);
      file.close();
      return true;
    }
  
  String pathWithGz = path + ".gz";
  DBG_OUTPUT_PORT.println("PathFile: " + pathWithGz);
  if(SPIFFS.exists(pathWithGz) || SPIFFS.exists(path)){
    if(SPIFFS.exists(pathWithGz))
      path += ".gz";
    File file = SPIFFS.open(path, "r");
    //size_t sent = server.streamFile(file, contentType);
    file.close();
    return true;
  }
  return false;
}

void setupWiFi()
{
  WiFi.mode(WIFI_AP);
  
  String AP_NameString = "Bird Dog";

  char AP_NameChar[AP_NameString.length() + 1];
  memset(AP_NameChar, 0, AP_NameString.length() + 1);

  for (int i=0; i<AP_NameString.length(); i++)
    AP_NameChar[i] = AP_NameString.charAt(i);

  WiFi.softAP(AP_NameChar);
  wifi_state = true;
  digitalWrite(WIFI_CONNECTION, HIGH);
}

void checkHeading()
{
  int heading;
  int azimuth;  // 0° - 359°

  digitalWrite(SLEEP, HIGH);

  compass.read(); // Read compass values via I2C
  azimuth   = compass.getAzimuth(); // Calculated from X and Y value 

  heading = map(azimuth,-180, 180, 0, 359);

  int avg = avgHeading.reading(heading);
  heading_currentreading = heading;

  //Serial.print("Heading = ");
  //Serial.println(avg);
  //Serial.println(heading_currentreading);

  NewHeading = heading_currentreading;   // Convert input heading string to int value heading
  //NewHeading = (NewHeading/2)*2;   // Eliminates 1 degree movements. Reduces movement errors with small movements
  //Serial.print(NewHeading); 

  //Movement = ((NewHeading + 180 - LastHeading) % 360) - 180;   // Allows movement from North East to North West
  //if(Movement < -180){Movement = Movement + 360;}   // Allows movement from North West to North East

  if (LastHeading != NewHeading)
  {
    if (NewHeading > LastHeading)
      {
        if ((NewHeading - LastHeading) <= -1)
        {
          NewHeading = LastHeading;
        }
      } else {
        if ((LastHeading - NewHeading) <= 1)
        {
          NewHeading = LastHeading;
        }
      }  
  }
  
  //Serial.println("New Heading = " + String(NewHeading) + " Last Heading = " + String(LastHeading));
  Movement = ((NewHeading + 180 - LastHeading) % 360) - 180;   // Allows movement from North East to North West

  //Serial.println("NE>NW Movement = " + String(Movement));
  if(Movement < -180)
    {
      Movement = Movement + 360;
      Serial.println("NW>NE Movement =" + String(Movement));
    }   // Allows movement from North West to North East
  Stepper1.setCurrentPosition(0);   // Zero the motor.
  Stepper1.setMaxSpeed(MaxSpeed);   // Set Max Speed of Stepper1 
  Stepper1.setAcceleration(Acceleration );   // Set Acceleration of Stepper1

  Stepper1.move(Movement);   // Set new move to position of Stepper1...increase the multiplier to increase the turn ratio...was 6
  LastHeading = NewHeading;   // New heading now becomes last heading 


  if (enable_checkHeading == true)
  {
    while(Stepper1.distanceToGo() != 0)
      {
        Serial.println("Movement = " + String(Movement));
        Stepper1.run();   // Keep updating AccelStepper for current movement
        //int XInChar = Serial.read();
        //XInChar = Serial.read();
      }   // Keep clearing serial buffer- Only allow new heading input once movement finished.   
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.print("Hello.....");
  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(V3V3, OUTPUT);
  pinMode(LEFTBUTTON, INPUT);
  pinMode(RIGHTBUTTON, INPUT);
  pinMode(ENABLE, INPUT);
  pinMode(SLEEP, OUTPUT);
  //pinMode(M2, INPUT);

  digitalWrite(STEP,LOW);
  digitalWrite(DIR,LOW);
  digitalWrite(ENABLE,HIGH);
  digitalWrite(SLEEP, HIGH);  //Take the A998 driver out of sleep mode
  //digitalWrite(M2, LOW);

  
  SPIFFS.begin();
  Serial.println();
  Serial.print("Configuring access point...");
  /* You can remove the password parameter if you want the AP to be open. */
  setupWiFi();
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);

  
  server.on("/", HTTP_GET, [](){
    handleFileRead("/");
    //handleRoot();
  });

//Handle when user requests a file that does not exist
  server.onNotFound([](){
    if(!handleFileRead(server.uri()))
  server.send(404, "text/plain", "File Not Found");
  });
  server.begin();
  Serial.println("HTTP server started");

  // start webSocket server
  webSocket.begin();
  Serial.println("Websocket server started");
  webSocket.onEvent(webSocketEvent);

//+++++++ MDNS will not work when WiFi is in AP mode but I am leave this code in place incase this changes++++++
//if (!MDNS.begin("esp8266")) {
//    Serial.println("Error setting up MDNS responder!");
//    while(1) { 
//      delay(1000);
//    }0;9
//  }
//  Serial.println("mDNS responder started");

  // Add service to MDNS
  //  MDNS.addService("http", "tcp", 80);
  //  MDNS.addService("ws", "tcp", 81);

  
  compass.init();
  compass.setMode(qmc5883l_mode_cont, qmc5883l_odr_200hz, qmc5883l_rng_2g, qmc5883l_osr_512);
  compass.setSmoothing(30, false);
  compass.setCalibrationOffsets(251.00, -105.00, -584.00);
  compass.setCalibrationScales(0.90, 0.95, 1.18);

  avgHeading.begin();

  Stepper1.setMaxSpeed(500); //number of steps per second
  Stepper1.setAcceleration(100);

}

void loop()
{
  checkHeading();
  left_buttonState = digitalRead(LEFTBUTTON);
  //left_buttonState = LOW;
  right_buttonState = digitalRead(RIGHTBUTTON);
  enable_buttonState = digitalRead(ENABLE);

  if ((left_buttonState == HIGH) || (right_buttonState == HIGH) || (left_button_touchstart == true) || (right_button_touchstart == true)) {
    digitalWrite(ENABLE,LOW);
  } else {
    digitalWrite(ENABLE,HIGH);
  }

  if ((left_buttonState == HIGH) || (left_button_touchstart == true)) {
    //rotateLeft();
    Stepper1.runToNewPosition(-10);
  }

  if ((right_buttonState == HIGH) || (right_button_touchstart == true)){
    //rotateRight();
    Stepper1.runToNewPosition(10);
  }
//delay(250);


  webSocket.loop();
  server.handleClient();

  JsonDocument clock_time;
  clock_time["message_type"] = "Heading";
  clock_time["value"] = LastHeading;
  //JsonObject time_data = clock_time.createNestedObject();
  JsonObject time_data = clock_time.add<JsonObject>();
  
  
  String databuf;
  serializeJson(clock_time, databuf);
 
 
  webSocket.sendTXT(0, databuf);

}
