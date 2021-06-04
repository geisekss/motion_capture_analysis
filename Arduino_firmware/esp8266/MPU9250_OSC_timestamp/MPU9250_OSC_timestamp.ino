#include "I2Cdev.h"
#include "MPU9250.h"
#include <Wire.h>
#include <SPI.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>
#include <Ticker.h>
#include <TimeLib.h>
#include <NTPClient.h>

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69

// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCEL
#define OUTPUT_READABLE_GYRO
#define MPU9250_ADDRESS 0x68
#define MPU9250_WHOAMI_DEFAULT_VALUE 0x71
// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO

//MPU6050 accelgyro(0x69); // <-- use for AD0 high  
uint8_t i2c_err_;

float  gyrosensitivity  = 131.0;   // = 131 LSB/degrees/sec
float  accelsensitivity = 16384.0;  // = 16384 LSB/g
        
int16_t ax, ay, az;
int16_t gx, gy, gz;
float AcX, AcY, AcZ, GyX, GyY, GyZ;

WiFiUDP udp;
boolean connected = false;

const char * networkName = "bodysuit";
const char * networkPswd = "";
const char * udpAddress = "192.168.1.3";
const int udpPort = 7777;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

bool blinkState = false;
uint8_t led_status=LOW; 
const int ledPin = 14;

const int buttonPin = 15;
int buttonState = 0; 
volatile int interruptCounter = 0;
int numberOfInterrupts = 0;

Ticker blink_scheduler, acc_scheduler;
MPU9250 mpu;

int32_t time_init;
int32_t millis_init;


void handleInterrupt() {
   Serial.println("An interrupt has occurred.");
   ESP.restart();
}

void pirntI2CError()
{
    if (i2c_err_ == 7) return; // to avoid stickbreaker-i2c branch's error code
    Serial.print("I2C ERROR CODE : ");
    Serial.println(i2c_err_);
}


uint8_t readByte(uint8_t address, uint8_t subAddress)
    {
        uint8_t data = 0; // `data` will store the register data
        Wire.beginTransmission(address);         // Initialize the Tx buffer
        Wire.write(subAddress);                   // Put slave register address in Tx buffer
        i2c_err_ = Wire.endTransmission(false);        // Send the Tx buffer, but send a restart to keep connection alive
        if (i2c_err_) pirntI2CError();
        Wire.requestFrom(address, (size_t)1);  // Read one byte from slave register address
        if (Wire.available()) data = Wire.read();                      // Fill Rx buffer with result
        return data;                             // Return data read from slave register
    }
    
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
    Wire.beginTransmission(address);  // Initialize the Tx buffer
    Wire.write(subAddress);           // Put slave register address in Tx buffer
    Wire.write(data);                 // Put data in Tx buffer
    Wire.endTransmission();           // Send the Tx buffer
}
    
bool isConnectedMPU9250()
{
    byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
    Serial.print("MPU9250 WHO AM I = ");
    Serial.println(c, HEX);
    return (c == MPU9250_WHOAMI_DEFAULT_VALUE);
}


void init_mpu() {
  mpu.setup();

  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);
  delay(200);
  writeByte(MPU9250_ADDRESS, MPU_CONFIG, 0x00);
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);
  
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00); 
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x01);
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); 
  writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
  writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);
  delay(100);

  byte c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2);
  Serial.print("0x");
  Serial.print(c, HEX);
  Serial.println("");
}

void capture_mpu() {
  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  //Solicita os dados do sensor
  Wire.requestFrom(MPU9250_ADDRESS,14,true);  
  //Armazena o valor dos sensores nas variaveis correspondentes
  ax = (Wire.read()<<8|Wire.read());  //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcX = ax / accelsensitivity;
  ay = (Wire.read()<<8|Wire.read());  //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcY = ay / accelsensitivity;
  az = (Wire.read()<<8|Wire.read());  //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  AcZ = az / accelsensitivity;
  gx = (Wire.read()<<8|Wire.read());  //0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyX = gx / gyrosensitivity;
  gy = (Wire.read()<<8|Wire.read());  //0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyY = gy / gyrosensitivity;
  gz = (Wire.read()<<8|Wire.read());  //0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  GyZ = gz / gyrosensitivity;
}


void setup()
{
    Serial.begin(115200);

    Wire.begin();

    delay(2000);

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

  pinMode(buttonPin,  INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(buttonPin), handleInterrupt, FALLING);

  init_mpu();
  Serial.println("Comunicating with MPU...");
  delay(5000);
  connectToWiFi(networkName, networkPswd);

  timeClient.begin();
  timeClient.setTimeOffset(0);
  timeClient.forceUpdate();
  
  time_init = timeClient.getEpochTime();
  millis_init = millis();
  blink_scheduler.attach(0.05, blink_led);
  acc_scheduler.attach(0.01, send_OSC);
}

void loop()
{
//   send_OSC();
}

//
//void setup() {
//
//        // join I2C bus (I2Cdev library doesn't do this automatically)
//    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
//        Wire.begin();
//    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
//        Fastwire::setup(400, true);
//    #endif
//    
//    Serial.begin(115200);
//    Serial.println("setup");
//    pinMode(ledPin, OUTPUT);
//    digitalWrite(ledPin, HIGH);
//    
//    pinMode(buttonPin,  INPUT_PULLUP);
//    attachInterrupt(digitalPinToInterrupt(buttonPin), handleInterrupt, FALLING);
//    
//    
//    Serial.println("Initializing I2C devices...");
//    //init_mpu();
//    mpu.setup();
//    
//    Serial.println("Comunicating with MPU...");
//    delay(5000);
//    connectToWiFi(networkName, networkPswd);
//
//    timeClient.begin();
//    timeClient.setTimeOffset(0);
//    timeClient.forceUpdate();
//    
//    time_init = timeClient.getEpochTime();
//    millis_init = millis();
//    
//    blink_scheduler.attach(0.05, blink_led);
//    acc_scheduler.attach(0.002, send_OSC);
// 
//}
//
//void loop() {
//    // read raw accel/gyro measurements from device
//    //mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//    //capture_mpu();
//}

void send_OSC(){
    capture_mpu();

    int32_t time_now = (int32_t)millis();
    OSCMessage msg("/imu");
    msg.add(time_init);
    msg.add(millis_init);
    msg.add(time_now);
    Serial.print(time_now); Serial.print("\t");
    
    
    #ifdef OUTPUT_READABLE_ACCEL
        Serial.print("a:\t");
        Serial.print(ax); Serial.print(",");
        Serial.print(ay); Serial.print(",");
        Serial.print(az); Serial.print("\t");

        msg.add("acc");
        msg.add((float) AcX);
        msg.add((float) AcY);
        msg.add((float) AcZ);
    #endif

    #ifdef OUTPUT_READABLE_GYRO
        // display tab-separated accel/gyro x/y/z values
        Serial.print("g:\t");
        Serial.print(gx); Serial.print(",");
        Serial.print(gy); Serial.print(",");
        Serial.println(gz); 

        msg.add("gyro");
        msg.add((float) GyX);
        msg.add((float) GyY);
        msg.add((float) GyZ);
    #endif


    udp.beginPacket(udpAddress, udpPort);
    msg.send(udp);
    udp.endPacket();
    msg.empty();
}

void connectToWiFi(const char * ssid, const char * pwd) {
  Serial.println("Connecting to WiFi network: " + String(ssid));

  // delete old config
  //WiFi.disconnect(true);
  //register event handler
  //WiFi.onEvent(WiFiEvent);

  //Initiate connection
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pwd);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  connected = true;

}


void blink_led() {
  if (led_status==LOW) {
    led_status = HIGH;
  } else {
    led_status = LOW;
  }
  digitalWrite(ledPin, led_status);
}
