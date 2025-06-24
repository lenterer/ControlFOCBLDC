#include <SimpleFOC.h>
#include <AS5600.h>
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <time.h>

// DRV8302 pins connections
// don't forget to connect the common ground pin
#define   INH_A 4
#define   INH_B 2
#define   INH_C 15
#define   EN_GATE 5
#define   M_PWM 23 
#define   M_OC 19
#define   OC_ADJ 18
//SCL 22 SDA 21

// Wi-Fi credentials
const char* ssid = "sukasuka";
const char* password = "gaksukakamu";

// MQTT broker details private
const char* mqtt_broker = "abda57f3c12743db931e9bed3781aca3.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_username = "Testing.Rippers";
const char* mqtt_password = "Bismillah1";

// MQTT topic subscribe
const char* topic = "esp32/TEST";

// Create instances
WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);

// Variables for timing
long previous_time = 0;

// motor instance
BLDCMotor motor = BLDCMotor(14, 0.263, 140);

// driver instance
BLDCDriver3PWM driver = BLDCDriver3PWM(INH_A, INH_B, INH_C, EN_GATE);

// Inisialisasi sensor AS5600 dengan I2C
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);


// angle set point variable
float target_angle = 0;

// commander interface
Commander command = Commander(Serial);
void onTarget(char* cmd){ command.scalar(&target_angle, cmd); }

void setupMQTT() {
  mqttClient.setServer(mqtt_broker, mqtt_port);
  mqttClient.setCallback(callback);
}

void reconnect() {
  Serial.println("Connecting to MQTT Broker...");
  while (!mqttClient.connected()) {
    Serial.println("Reconnecting to MQTT Broker...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    
    if (mqttClient.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("Connected to MQTT Broker.");
      mqttClient.subscribe(topic);
    } else {
      Serial.print("Failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

// Callback jika ada pesan masuk dari topik yang disubscribe
void callback(char* topic, byte* payload, unsigned int length) {
  printCurrentTime();

  // Salin payload ke string
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  // Split 2:180 jadi 2 dan 180
  int split = message.indexOf(':'); 
  String buttons = message.substring(0, split);
  message = message.substring(split + 1);
  // Konversi string ke float (dalam derajat)
  int button = buttons.toInt();
  float degrees = message.toFloat();

  // Ubah ke radians
  float radians = 6 * ( degrees * PI / 180.0 );

  if(button == 1){
    target_angle = radians;

    // Tampilkan hasil
    //Serial.printf("Derajat: %.2f°, Radian: %.4f\n", degrees, target_angle);  // 4 digit di belakang koma
    Serial.println( degrees );
  }
}

void setupTime() {
  configTime(7 * 3600, 0, "pool.ntp.org", "time.nist.gov"); // GMT+7 (WIB)
  Serial.print("Menunggu sinkronisasi waktu");
  while (time(nullptr) < 100000) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nWaktu sinkron!");
}

void printCurrentTime() {
  time_t now = time(nullptr);
  struct tm* timeinfo = localtime(&now);
  Serial.printf("%02d:%02d:%02d -> ",
    timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
}

void setup() {
  // monitoring port
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("Connected to Wi-Fi");

  // Initialize secure WiFiClient
  wifiClient.setInsecure(); // Use this only for testing, it allows connecting without a root certificate
  
  setupTime();
  setupMQTT();

  // configure i2C
  Wire.setClock(400000);
  // initialise magnetic sensor hardware
  sensor.init();
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // DRV8302 specific code
  // M_OC  - enable over-current protection
  pinMode(M_OC,OUTPUT);
  digitalWrite(M_OC,LOW);
  // M_PWM  - enable 3pwm mode
  pinMode(M_PWM,OUTPUT);
  digitalWrite(M_PWM,HIGH);
  // OD_ADJ - set the maximum over-current limit possible
  // Better option would be to use voltage divisor to set exact value
  pinMode(OC_ADJ,OUTPUT);
  digitalWrite(OC_ADJ,HIGH);

  // configure driver
  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);

  // choose FOC modulation
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  // set control loop type to be used
  motor.controller = MotionControlType::angle;

  // controller configuration 
  // default parameters in defaults.h

  // controller configuration based on the control type 
  // velocity PID controller parameters
  // default P=0.5 I = 10 D =0
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 10;
  motor.PID_velocity.D = 0.001;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 300;

  // velocity low pass filtering
  // default 5ms - try different values to see what is the best. 
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.01;

  // angle P controller -  default P=20
  motor.P_angle.P = 20;
  
  // angle loop velocity limit
  motor.velocity_limit = 5;
  // // aligning voltage
  //motor.voltage_sensor_align = 2;
  // default voltage_power_supply
  motor.voltage_limit = 12;
  //motor.current_limit = 8;

  // use monitoring with serial for motor init
  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialize motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();

  Serial.println("Motor ready.");
  Serial.println("Set the target angle using serial terminal:");
  // add target command T
  command.add('T', onTarget, "target angle");
  
  _delay(1000);
}


void loop() {
  if (!mqttClient.connected()) {
    Serial.println("MQTT disconnected!");
    reconnect();
  }
  mqttClient.loop();

  // iterative setting FOC phase voltage
  motor.loopFOC();

  // iterative function setting the outer loop target
  // velocity, position or voltage
  // if target not set in parameter uses motor.target variable
  // 2 π(6.28) rad/s = 1 motor rotation per second = 60 RPM
  motor.move(target_angle);

  // user communication
  command.run();

  // Monitoring 
  // motor.monitor();
}