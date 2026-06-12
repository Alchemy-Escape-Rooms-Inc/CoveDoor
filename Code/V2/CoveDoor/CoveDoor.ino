//===================================
//          MACROS
//===================================

#include <WiFi.h>
#include <PubSubClient.h>

#define VERSION "2.0.0"

#define GAME_NAME "MermaidsTale"
#define PROP_NAME "CoveDoor"

#define MQTT_TOPIC_COMMAND  "MermaidsTale/CoveDoor/command"
#define MQTT_TOPIC_STATUS   "MermaidsTale/CoveDoor/status"
#define MQTT_TOPIC_LOG      "MermaidsTale/CoveDoor/log"
#define MQTT_TOPIC_MESSAGE  "MermaidsTale/CoveDoor/message"
#define MQTT_TOPIC_SYSTEM   "MermaidsTale/CoveDoor/system"

//========may end up not using==== 
#define RPWM_PIN 2
#define LPWM_PIN 5
//================================

#define DIR_OPEN LOW
#define DIR_CLOSE HIGH

#define PWM_CHANNEL_R 0
#define PWM_CHANNEL_L 1
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

#define LIMIT_OPEN_PIN 16                
#define LIMIT_CLOSE_PIN 17

#define MOTOR_TIMEOUT_MS 10000
#define MOTOR_SPEED 220

//===================================
//          GLOBAL VARIABLES
//===================================
const unsigned long heartBeatPulse = 5 * 1000UL;
unsigned long lastTime = 0;



WiFiClient espClient;
PubSubClient mqttClient(espClient);

// WiFi credentials
const char* WIFI_SSID = "AlchemyGuest";
const char* WIFI_PASS = "VoodooVacation5601";

// MQTT broker
const char* MQTT_SERVER = "10.1.10.115";
const int MQTT_PORT = 1883;


bool limitOpenTriggered = false;
bool limitCloseTriggered = false;
bool motorDir = false;

//===================================
//          WIFI (NETWORK)
//===================================
void setupWiFi() {
  delay(1000);
  Serial.println("*********** WIFI ***********");
  Serial.print("Connecting to SSID: ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID,WIFI_PASS);

  while(WiFi.status() != WL_CONNECTED){
    delay(100);
    Serial.print("-");
  }
  Serial.println("\nConnected.");
}
//===================================
//          MQTT FUNCTIONS
//===================================
void connectMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Connecting to MQTT...");

    String clientId = PROP_NAME;
    clientId += "_";
    clientId += String(random(0xffff), HEX);

    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("connected!");

      // Subscribe to command topic
      mqttClient.subscribe(MQTT_TOPIC_COMMAND);

      // Announce we're online
      mqttClient.publish(MQTT_TOPIC_STATUS, "ONLINE");
      mqttLogf("%s v%s online", PROP_NAME, VERSION);

    } else {
      Serial.printf("failed (rc=%d), retrying in 5s\n", mqttClient.state());
      delay(5000);
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  char topicBuf[128];
  strncpy(topicBuf,topic,sizeof(topicBuf)-1);
  topicBuf[sizeof(topicBuf)-1] = '\0';

  char message[128];
  if(length >= sizeof(message)){
    length = sizeof(message) - 1;
  }

  memcpy(message,payload,length);
  message[length] = '\0';

  char * msg = message;
  while(*msg == ' ' || *msg == '\r' || *msg == '\n')
    msg++;
  char * end = msg + strlen(msg) -1;
  while(end > msg && (*end == ' ' || *end == 't' || *end == '\r' || *end == '\n')){
    *end = '\0';
    end--;
  }

  Serial.printf("[MQTT] Received on %s: %s\n", topicBuf,msg);

  if(strcmp(topicBuf,MQTT_TOPIC_COMMAND) != 0){
    return;
  }

  if(strcmp(msg,"PING") == 0){
    mqttClient.publish(MQTT_TOPIC_COMMAND,"PONG");
    Serial.println("[MQTT] PING -> PONG");
    return;
  }
  if(strcmp(msg,"STATUS") == 0){
    const char* state = "READY";
    mqttClient.publish(MQTT_TOPIC_COMMAND,state);
    Serial.printf("[MQTT] STATUS -> %s\n",state);
    return;
  }
  if(strcmp(msg,"RESET") == 0){
    mqttClient.publish(MQTT_TOPIC_COMMAND,"OK");
    Serial.println("[MQTT] RESET -> Rebooting...");
    delay(100);
    ESP.restart();
    return;
  }

  if (strcmp(msg,"promptStatus") == 0) {
    promptStatus();
    return;
  }
  if (strcmp(msg,"OPEN") == 0) {
    openDoor();
    return;
  }
  if (strcmp(msg,"CLOSE") == 0) {
    closeDoor();
    return;
  }
  if (strcmp(msg,"STOP") == 0) {
    stopDoor();
    return;
  }
  Serial.printf("[MQTT] Unknown command: %s\n", msg);
}

void setupMQTT() {
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(512);  // Increase if needed
}

void mqttLogf(const char* format, ...) {
  char buffer[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  mqttClient.publish(MQTT_TOPIC_LOG, buffer);
  Serial.println(buffer);
}
void heartBeat(){
  unsigned long currentTime = millis();
  if(!(currentTime - lastTime > heartBeatPulse))
    return;
  lastTime = currentTime;
  // Announce we're online
  mqttClient.publish(MQTT_TOPIC_STATUS, "ONLINE");
  mqttLogf("%s v%s online", PROP_NAME, VERSION);
  
  promptStatus();
}
//===================================
//        GENERAL FUNCTIONS
//===================================
void openDoor(){
  //check if door is already open
  //if not, set the direction to open
  //and begin opening the door
  if(!checkLimitSwitch(DIR_OPEN)){
    mqttClient.publish(MQTT_TOPIC_MESSAGE,"Open Limit Switch: Triggered.");
    return;
  }
  motorDir = DIR_OPEN;              //update global motor direction variable
  ledcWrite(PWM_CHANNEL_R,MOTOR_SPEED);   //start opening
}

void closeDoor(){
  //check if the door is already close,
  //if not, set the direction to close
  //and begin opening the door

  if(!checkLimitSwitch(DIR_CLOSE)){
    mqttClient.publish(MQTT_TOPIC_MESSAGE,"Close Limit Switch: Triggered.");
    return;
  }
  motorDir = DIR_CLOSE;             //update global motor direction variable
  ledcWrite(PWM_CHANNEL_L,MOTOR_SPEED);   //start opening
}

void stopDoor(){
  ledcWrite(PWM_CHANNEL_R,0);
  ledcWrite(PWM_CHANNEL_L,0);
}

bool checkLimitSwitch(bool dir){
  bool result;
  if(!dir){
    result = digitalRead(LIMIT_OPEN_PIN); //Open limit is triggered low
                                          //make sure that the open conditions are met when triggered
    if(!result){
      limitOpenTriggered = true;
      stopDoor();
      limitCloseTriggered = false;
      mqttClient.publish(MQTT_TOPIC_MESSAGE,"Open limit is reached. Door is stopped.");
    }

  }else{
    result =  digitalRead(LIMIT_CLOSE_PIN); //Close limit is triggered low
                                            //make sure that the close conditions are met when triggereed
    if(!result){
      limitCloseTriggered = true;
      stopDoor();
      limitOpenTriggered = false;
      mqttClient.publish(MQTT_TOPIC_MESSAGE,"Close limit is reached. Door is closed.");
    }

  }
  return result;
}

void promptStatus(){
  bool openLimitState = digitalRead(LIMIT_OPEN_PIN);
  bool closeLimitState = digitalRead(LIMIT_CLOSE_PIN);


  String topic1 = String(MQTT_TOPIC_SYSTEM) + "/OpenLimitState";
  mqttClient.publish(topic1.c_str(),(openLimitState ? "Not triggered":"Triggered" ));
  String topic2 = String(MQTT_TOPIC_SYSTEM) + "/CloseLimitState";
  mqttClient.publish(topic2.c_str(),(closeLimitState ? "Triggered" : "Not triggered"));


  String topic4 = String(MQTT_TOPIC_SYSTEM) + "/DoorState";
  if(!openLimitState)
    mqttClient.publish(topic4.c_str(),"Opened.");
  else if (closeLimitState)
    mqttClient.publish(topic4.c_str(),"Closed.");
  else if(openLimitState && closeLimitState)
    mqttClient.publish(topic4.c_str(),"Partially opened or closed.");
}

//I/O setup
void setupIO(){
  pinMode(LIMIT_OPEN_PIN,INPUT_PULLUP);
  pinMode(LIMIT_CLOSE_PIN,INPUT_PULLUP);

  Serial.println("IO pins initialization complete.");
}

//System initialization
void _init(){
  Serial.begin(9600);
  //IO setup
  setupIO();
  //WiFi setup
  setupWiFi();
  //Mqtt server setup
  setupMQTT();
  //motor control configuration
  //old API
  //ledcSetup(PWM_CHANNEL_R, PWM_FREQ, PWM_RESOLUTION);
  //ledcSetup(PWM_CHANNEL_L, PWM_FREQ, PWM_RESOLUTION);
  //ledcAttachPin(RPWM_PIN, PWM_CHANNEL_R);
  // ledcAttachPin(LPWM_PIN, PWM_CHANNEL_L);
  //new API
  ledcAttach(PWM_CHANNEL_R, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(PWM_CHANNEL_L, PWM_FREQ, PWM_RESOLUTION);
  ledcWrite(PWM_CHANNEL_R, 0);
  ledcWrite(PWM_CHANNEL_L, 0);
}
void program(){
  if (!mqttClient.connected()) {
    connectMQTT();
  }
  mqttClient.loop();

  heartBeat();
  checkLimitSwitch(motorDir);
}
//===================================
//          MAIN SETUP
//===================================
void setup(){
  _init();
}
//===================================
//          MAIN LOOP
//===================================
void loop(){
  program();
}
