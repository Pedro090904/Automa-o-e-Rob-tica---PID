#include <Arduino.h> // OBRIGATÓRIO NO PLATFORMIO
#include "I2Cdev.h"
#include <PID_v1.h>
#include "MPU6050_6Axis_MotionApps20.h"

// Declaração das funções para o compilador (Function Prototypes)
void Forward();
void Reverse();
void Stop();
void dmpDataReady();

MPU6050 mpu;

// ================================================================
// DEFINIÇÃO DOS PINOS DO MOTOR (L298N - 6 Pinos)
// ================================================================
const int ENA = 10; 
const int IN1 = 9;  
const int IN2 = 8;  
const int IN3 = 7;  
const int IN4 = 6;  
const int ENB = 5;  

bool dmpReady = false;  
uint8_t mpuIntStatus;   
uint8_t devStatus;      
uint16_t packetSize;    
uint16_t fifoCount;     
uint8_t fifoBuffer[64]; 

Quaternion q;           
VectorFloat gravity;    
float ypr[3];           

double setpoint = 182; 
double Kp = 15;  
double Kd = 0.9; 
double Ki = 140; 

double input, output;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

volatile bool mpuInterrupt = false;     

void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
    Serial.begin(115200);
    
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    
    devStatus = mpu.dmpInitialize();
    
    mpu.setXGyroOffset(-479);
    mpu.setYGyroOffset(84);
    mpu.setZGyroOffset(15);
    mpu.setZAccelOffset(1638); 

    if (devStatus == 0) {
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(2), dmpDataReady, RISING); // Melhor prática para interrupções
        mpuIntStatus = mpu.getIntStatus();
        
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
        
        packetSize = mpu.dmpGetFIFOPacketSize();
        
        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(10);
        pid.SetOutputLimits(-255, 255); 
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENB, OUTPUT);

    Stop();
}

void loop() {
    if (!dmpReady) return;

    while (!mpuInterrupt && fifoCount < packetSize) {
        pid.Compute();
        
        Serial.print(input); Serial.print(" => "); Serial.println(output);

        if (input > 150 && input < 200) {
            if (output > 0) {
                Forward(); 
            } else if (output < 0) {
                Reverse(); 
            }
        } else {
            Stop(); 
        }
    }

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        
        mpu.dmpGetQuaternion(&q, fifoBuffer); 
        mpu.dmpGetGravity(&gravity, &q); 
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); 
        
        input = ypr[1] * 180/M_PI + 180;
   }
}

void Forward() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, output);
    analogWrite(ENB, output);
}

void Reverse() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, output * -1);
    analogWrite(ENB, output * -1);
}

void Stop() {
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}