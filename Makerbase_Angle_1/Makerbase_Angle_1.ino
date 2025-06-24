// MKS ESP32 FOC Open Loop Position Control Example; Test Library：SimpleFOC 2.1.1; Test Hardware：MKS ESP32 FOC V1.0
// Enter "T+number" in the serial port to set the position of the two motors.
// For example, enter "T3.14" (180 degrees in radians) to make the motor rotate to the 180 degree position.
// When using your own motor, please remember to modify the default number of pole pairs, the value in BLDCMotor()
// The default power supply voltage set by the program is 12V.
// Please remember to modify the values in voltage_power_supply and voltage_limit variables if you use other voltages for power supply

#include <SimpleFOC.h>
#include <Wire.h>
#include <AS5600.h>
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <time.h>

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

BLDCMotor motor = BLDCMotor(7);         //According to the selected motor, modify the value in BLDCMotor()
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 22);

// BLDCMotor motor1 = BLDCMotor(7);        //Also modify the value of pole logarithm here
// BLDCDriver3PWM driver1 = BLDCDriver3PWM(26, 27, 14, 12);

// Inisialisasi sensor AS5600 dengan I2C
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

//Target Variable
float target_angle = 0;

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

  // Ubah ke radian
  float radians = ( degrees * PI / 180.0 );

  if(button == 2){
    printCurrentTime();
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

//Serial Command Setting
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_angle, cmd); }

void setup() {
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

  // initialise magnetic sensor hardware
  Wire.begin(19, 18);
  sensor.init();
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = 12;       //According to the supply voltage of the motor, modify the value of the voltage_power_supply variable
  driver.init();
  motor.linkDriver(&driver);
  motor.voltage_limit = 3;   // [V]      //According to the supply voltage of the motor, modify the value of the voltage_limit variable
  motor.velocity_limit = 4; // [rad/s]
  
  // driver1.voltage_power_supply = 12;
  // driver1.init();
  // motor1.linkDriver(&driver1);
  // motor1.voltage_limit = 3;   // [V]
  // motor1.velocity_limit = 10; // [rad/s]

  // choose FOC modulation
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  //Open Loop Control Mode Setting
  motor.controller = MotionControlType::angle;
  // motor1.controller = MotionControlType::angle_openloop;

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

  motor.useMonitoring(Serial);
  //Initialize the Hardware
  motor.init();
  motor.initFOC();
  // motor1.init();
  // align encoder and start FOC

  //Add T Command
  //Enter the "T+number" command in the serial monitor and click send
  //to control the two motors to rotate to the specified position
  //For example, T3.14 is to rotate to the position of 180° in the positive direction
  command.add('T', doTarget, "target angle");

  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s]");
  _delay(1000);
}

void loop() {
  if (!mqttClient.connected()) {
    Serial.println("MQTT disconnected!");
    reconnect();
  }
  mqttClient.loop();
  
  motor.loopFOC();
  motor.move(target_angle);
  // motor1.move(target_velocity);

  //User Newsletter
  command.run();
}