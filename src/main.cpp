#include <Arduino.h>
#include <cmath>
#include <WiFi.h>
#include <IPAddress.h>
#include <esp_camera.h>
#include <DateTime.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <Timer.h>
#include <Logger.h>

// PINS
#define LED_FLASH 4
#define PIN_LEFT_FORWARD 16
#define PIN_LEFT_BACKWARD 12
#define PIN_RIGHT_FORWARD 13
#define PIN_RIGHT_BACKWARD 14

// PWM Channels (Reserve channel 0 and 1 for camera)
#define PWM_LEFT_FORWARD LEDC_CHANNEL_2
#define PWM_LEFT_BACKWARD LEDC_CHANNEL_3
#define PWM_RIGHT_FORWARD LEDC_CHANNEL_4
#define PWM_RIGHT_BACKWARD LEDC_CHANNEL_5

// MIN and MAX values for PWM
#define PWM_FREQUENCY 50
#define PWM_RESOLUTION LEDC_TIMER_16_BIT

// These values are determined by experiment and are unique to every robot
#define PWM_MOTOR_MIN 5000    // The value where the motor starts moving
#define PWM_MOTOR_MAX 65535   // Full rotation


// You can also define your wifi access in 'platformio.ini' or by setting environment variables.
// https://docs.platformio.org/en/latest/projectconf/section_env_build.html#build-flags
// https://docs.platformio.org/en/latest/envvars.html#envvar-PLATFORMIO_BUILD_FLAGS
// If you do, be sure to escape the quotes (\"), e.g.: -D WIFI_SSID=\"YOUR_SSID\"
#ifndef WIFI_SSID
#define WIFI_SSID "YOUR_SSID"
#endif
#ifndef WIFI_PASS
#define WIFI_PASS "YOUR_PASSWORD"
#endif


namespace esp32cam {

const char *ssid = WIFI_SSID;
const char *password = WIFI_PASS;

// ROS
IPAddress serialServer(10, 0, 0, 10);
ros::NodeHandle *node;
ros::Subscriber<geometry_msgs::Twist> *cmdvelSub;
ros::Subscriber<std_msgs::Bool> *flashSub;
ros::Publisher *fpsPub;
ros::Publisher *logPub;
ros::Publisher *streamPub;
std_msgs::Float32 *fpsMsg;
std_msgs::String *logMsg;
//sensor_msgs::Image *streamMsg;

// ROS topics
String chipId = "ESP32_"; // Will be appended by mac address
String cmdVelTopic;       // chipId + /cmd_vel
String streamTopic;       // chipId + /stream
String flashTopic;        // chipId + /flash
String fpsTopic;          // chipId + /fps
String logTopic;          // chipId + /log


// Variables
bool wifiConnected = false;
bool rosConnected = false;
bool movement = false;
float linear, angular = 0;

// Timing
#define AVERAGE_ALPHA 0.7f
float frameDuration = 0;
uint32_t lastFrameTime = 0;
uint32_t lastCmdVelMessage = 0;

// Tickers
//#define MAX_CMD_VEL_INTERVAL 1000 // If there are no new CMD_VEL message during this interval, the robot stops

// Function definitions
void onCmdVel(const geometry_msgs::Twist &msg);
void onFlash(const std_msgs::Bool &msg);
void stop();

void handleMovement();
void checkConnection();

void publishFps();
void publishLog(const char *format, ...);

float fmap(float val, float in_min, float in_max, float out_min, float out_max);


// Setup functions
// -------------------------------------------------------------------

void setupPins() {
  // Initialize status & flash LEDs. (LED_BUILTIN is active on LOW)
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_FLASH, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(LED_FLASH, LOW);

  // PWM channels: left and right motors have a forward and backward channel each
  ledcSetup(PWM_LEFT_FORWARD, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_LEFT_BACKWARD, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_RIGHT_FORWARD, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_RIGHT_BACKWARD, PWM_FREQUENCY, PWM_RESOLUTION);

  ledcAttachPin(PIN_LEFT_FORWARD, PWM_LEFT_FORWARD);
  ledcAttachPin(PIN_LEFT_BACKWARD, PWM_LEFT_BACKWARD);
  ledcAttachPin(PIN_RIGHT_FORWARD, PWM_RIGHT_FORWARD);
  ledcAttachPin(PIN_RIGHT_BACKWARD, PWM_RIGHT_BACKWARD);

  ledcWrite(PIN_LEFT_FORWARD, 0);
  ledcWrite(PIN_LEFT_BACKWARD, 0);
  ledcWrite(PIN_RIGHT_FORWARD, 0);
  ledcWrite(PIN_RIGHT_BACKWARD, 0);
}

void setupSerial() {
  Serial.begin(115200);
  Serial.setDebugOutput(false);
  Serial.println();
  delay(500);
}

void setupWifi() {
  // Setting persistent to false disables writing wifi credentials to EEPROM.
  WiFi.persistent(false);
  WiFi.softAPdisconnect();
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  // Get serial number from MAC address
  chipId.concat(WiFi.macAddress());
  chipId.replace(":", "");
  Serial.printf("ChipId: %s\n", chipId.c_str());

  DateTimeClass time(BUILD_TIME);
  Serial.printf("Build:  %s\n", time.toISOString().c_str());
}

void setupTopics() {

  cmdVelTopic = chipId + "/cmd_vel";
  flashTopic = chipId + "/flash";
  fpsTopic = chipId + "/fps";
  logTopic = chipId + "/log";
  streamTopic = chipId + "/stream";
}

void setupRos() {
  Serial.printf("ROS serial server: %s\n", serialServer.toString().c_str());

  // Connect to ros-serial socket server and init node. (Using default port of 11411)
  node = new ros::NodeHandle();
  node->getHardware()->setConnection(serialServer);
  node->initNode();

  // Subscribers
  cmdvelSub = new ros::Subscriber<geometry_msgs::Twist>(cmdVelTopic.c_str(), &onCmdVel);
  node->subscribe(*cmdvelSub);

  flashSub = new ros::Subscriber<std_msgs::Bool>(flashTopic.c_str(), &onFlash);
  node->subscribe(*flashSub);

  // Publishers
  fpsMsg = new std_msgs::Float32();
  fpsPub = new ros::Publisher(fpsTopic.c_str(), fpsMsg);
  node->advertise(*fpsPub);

  logMsg = new std_msgs::String();
  logPub = new ros::Publisher(logTopic.c_str(), logMsg);
  node->advertise(*logPub);
}

void setupTickers() {
  Timer.setInterval(&Logger, 5, -1, -10);
  Timer.setInterval(checkConnection, 1000);
  Timer.setInterval(handleMovement, 10);
  Timer.setInterval(publishFps, 1000);
}

// ROS callbacks
// -------------------------------------------------------------------

void onFlash(const std_msgs::Bool &msg) {
  digitalWrite(LED_FLASH, msg.data); // false -> off, true -> on
  publishLog("Flash: %s", msg.data ? "ON" : "OFF");
}

// Receive messages and store them. They are handled once per frame in main loop.
void onCmdVel(const geometry_msgs::Twist &msg) {
  // Cap values at [-1 .. 1]
  linear = constrain(msg.linear.x, -1, 1);
  angular = constrain(msg.angular.z, -1, 1);

  lastCmdVelMessage = millis();
  movement = true;
}

// Robot control
// -------------------------------------------------------------------


// Stop in next frame
void stop() {
  linear = 0;
  movement = true;
}

// Set pwm channels according to stored linear and angular values
void handleMovement() {
#ifdef MAX_CMD_VEL_INTERVAL
  // If defined, interval between two cmd_vel messages must not exceed this value or else the robot stops
  if (millis() > lastCmdVelMessage + MAX_CMD_VEL_INTERVAL)
    stop();
#endif

  if (!movement)
    return;

  // This robot is an RC tank and uses a differential drive.
  // Calculate the intensity of left and right wheels. Simple version.
  // Taken from https://hackernoon.com/unicycle-to-differential-drive-courseras-control-of-mobile-robots-with-ros-and-rosbots-part-2-6d27d15f2010#1e59
  float left = (linear - angular) / 2;
  float right = (linear + angular) / 2;

  // Then map those values to PWM intensities. PWM_MOTOR_MAX = full speed, PWM_MOTOR_MIN = the minimal amount of power at which the motors begin moving.
  auto pwmLeft = (uint16_t) fmap(std::fabs(left), 0, 1, PWM_MOTOR_MIN, PWM_MOTOR_MAX);
  auto pwmRight = (uint16_t) fmap(std::fabs(right), 0, 1, PWM_MOTOR_MIN, PWM_MOTOR_MAX);

  // Each wheel has a channel for forwards and backwards movement
  ledcWrite(PWM_LEFT_FORWARD, pwmLeft * (left > 0));
  ledcWrite(PWM_LEFT_BACKWARD, pwmLeft * (left < 0));
  ledcWrite(PWM_RIGHT_FORWARD, pwmRight * (right > 0));
  ledcWrite(PWM_RIGHT_BACKWARD, pwmRight * (right < 0));

  publishLog("%d, %d %d, %d, %d %d, %f, %f", pwmLeft, left > 0, left < 0, pwmRight, right > 0, right < 0, left, right);
  movement = false;
}

// ROS
// -------------------------------------------------------------------

void publishFps() {
  if (frameDuration > 0) {
    fpsMsg->data = 1000.0f / frameDuration;
    fpsPub->publish(fpsMsg);
//    Serial.printf("%f\n", fpsMsg->data);
  }
}

void publishLog(const char *format, ...) {
  char buffer[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, 256, format, args);
  va_end(args);
  logMsg->data = buffer;
  logPub->publish(logMsg);
  Serial.printf("%s\n", buffer);
}

void measureFramerate() {
  uint32_t now = millis();
  uint32_t duration = now - lastFrameTime;
  lastFrameTime = now;

  // Calculate Exponential moving average of frame time
  // https://en.wikipedia.org/wiki/Moving_average#Exponential_moving_average
  frameDuration = duration * AVERAGE_ALPHA + (frameDuration * (1 - AVERAGE_ALPHA));
}

// Check connection functions
// -------------------------------------------------------------------

bool checkWifi() {
  if (wifiConnected) {
    if (WiFi.status() != WL_CONNECTED) {
      wifiConnected = false;
      stop();

      Serial.println("WiFi disconnected");
    }
  } else {
    if (WiFi.status() == WL_CONNECTED) {
      wifiConnected = true;

      Serial.println("WiFi connected");
      Serial.printf("SSID: %s\n", WiFi.SSID().c_str());
      Serial.printf("IP:   %s\n", WiFi.localIP().toString().c_str());
    }
  }
  return wifiConnected;
}

bool checkROS() {

  if (rosConnected) {
    if (!node->connected()) {
      rosConnected = false;
      stop();

      Serial.println("ROS disconnected");
      digitalWrite(LED_BUILTIN, !rosConnected); // false -> on, true -> off
    }
  } else {
    if (node->connected()) {
      rosConnected = true;

      Serial.println("ROS connected");
      digitalWrite(LED_BUILTIN, !rosConnected); // false -> on, true -> off
    }
  }
  return rosConnected;
}

void checkConnection() {
  if (checkWifi()) {
    checkROS();
  }
}

// Main functions
// -------------------------------------------------------------------

void setup() {
  setupPins();
  setupSerial();
  setupWifi();
  setupTopics();
  setupRos();
  setupTickers();
}

// Note: regular loop function gets called way too fast, so we run our own loop here
void loop() {
  measureFramerate();
  node->spinOnce();
  Timer.loop();
}


// Helper functions
// -------------------------------------------------------------------

float fmap(float val, float in_min, float in_max, float out_min, float out_max) {
  return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
}

void setup() {
  esp32cam::setup();
}

void loop() {
  esp32cam::loop();
}
