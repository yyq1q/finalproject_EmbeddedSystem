#include <Servo.h>
#include <IRremote.h>

#define channel1 0x2FD807F
#define channel2 0x2FD40BF
#define channel3 0x2FDC03F
#define channel4 0x2FD20DF
#define channel5 0x2FDA05F
#define channel6 0x2FD609F
#define channel7 0x2FDE01F
#define channel8 0x2FD10EF
#define channel9 0x2FD906F
#define channel10 0x2FD50AF
#define channel11 0x2FDD02F
#define channel12 0x2FD30CF
#define channelup 0x2FDD827
#define channeldown 0x2FDF807
#define volumeup 0x2FD58A7
#define volumedown 0x2FD7887
#define modechange 0x2FDF00F //入力切替
#define blueButton 0x2FDCE31
#define redButton 0x2FD2ED1
#define greenButton 0x2FDAE51
#define yellowButton 0x2FD6E91
#define decitonButton 0x2FDBC43

Servo Servo0;
Servo Servo1;
Servo Servo2;
Servo Servo3;

//初期位置
int arg0IP = 85;
int arg1IP = 90;
int arg2IP = 0;
int arg3open = 0;

//目標位置
int arg0GP = 90;
int arg1GP = 180;
int arg2GP = 90;
int arg3close = 180;

//ごみ箱の位置
int arg0DP = 90;
int arg1DP = 90;
int arg2DP = 0;

//アームの動き
#define StoG 0//フラグ
#define GtoS 1//フラグ
#define StoD 2//フラグ
#define DtoS 3//フラグ

//アームの動きを決める定数
float a = (arg0GP - arg0IP);
float b = (arg1GP - arg1IP);
float c = (arg2GP - arg2IP);
float d = (arg3close - arg3open);
float e = (arg0IP - arg0GP);
float f = (arg1IP - arg1GP);
float g = (arg2IP - arg2GP);
float h = (arg3open - arg3close);
float Da = (arg0DP - arg0IP);
float Db = (arg1DP - arg1IP);
float Dc = (arg2DP - arg2IP);
float Dd = (90 - arg3open);
float De = (arg0IP - arg0DP);
float Df = (arg1IP - arg1DP);
float Dg = (arg2IP - arg2DP);
float Dh = (arg3open - arg3close);

//距離センサーのピン設定
#define trigPin 13
#define echoPin 12

//IRのピン設定
#define receiverPin 9


IRrecv receiver(receiverPin);
decode_results results;

//ヨ―角を求めるための関数など
#include <avr/wdt.h>
#include "MPU6050.h"
#include "MPU6050_getdata.h"

MPU6050_getdata AppMPU6050getdata;
static float Yaw;

//P制御するためのヨ―角保存
float preYaw = 0;

//速度上限
int UpperLimit = 255;

//P制御のゲイン
int Kp = 10;

float Duration = 0;  // 計測した時間
float Distance = 0;  // 距離
float DistanceToTarget = 10; //目標までの距離
float DistanceToTarget2 = 5; //目標までの距離
float T = 20; //温度（音速設定に必要）

int Speed;//目標へ向かう速度

bool Flag = true; //目標をつかんだ否か
bool Flag1 = true; //180度回転したか否か

//リモコンでの操作における設定
int SPEED = 64; //初期速度
int arg0 = 0; //初期角度
int arg1 = 0; //初期角度
int arg2 = 0; //初期角度
int arg3 = 0; //初期角度

bool flag_mode = false;
bool flag_up = false;
bool flag_down = false;
bool flag_servo0 = false;
bool flag_servo1 = false;
bool flag_servo2 = false;
bool flag_servo3 = false;

void setup() {
    Serial.begin(9600);

    receiver.enableIRIn();

    AppMPU6050getdata.MPU6050_dveInit();
    delay(2000);
    AppMPU6050getdata.MPU6050_calibration();

    pinMode(echoPin, INPUT);
    pinMode(trigPin, OUTPUT);
    pinMode(3, OUTPUT);
    pinMode(5, OUTPUT); //右の回転速度
    pinMode(6, OUTPUT); //左の回転速度
    pinMode(7, OUTPUT);
    pinMode(8, OUTPUT);
    digitalWrite(3, HIGH); //モーター電源ON
    armSetup();
    delay(1000);
}

void loop() {
a = (arg0GP - arg0IP);
b = (arg1GP - arg1IP);
c = (arg2GP - arg2IP);
d = (arg3close - arg3open);
e = (arg0IP - arg0GP);
f = (arg1IP - arg1GP);
g = (arg2IP - arg2GP);
h = (arg3open - arg3close);
Da = (arg0DP - arg0IP);
Db = (arg1DP - arg1IP);
Dc = (arg2DP - arg2IP);
Dd = (90 - arg3open);
De = (arg0IP - arg0DP);
Df = (arg1IP - arg1DP);
Dg = (arg2IP - arg2DP);
Dh = (arg3open - arg3close);
  Serial.println(Distance);
    if (flag_mode == false) {
        if (receiver.decode(&results)) {
            remotecontrolcar(results.value);
            receiver.resume();
        }
    }

    else {
        if (Flag == true) {
            Servo0.write(arg0IP);
            Servo1.write(arg1IP);
            Servo2.write(arg2IP);
            Servo3.write(arg3open);
            AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);

            digitalWrite(trigPin, LOW);              // 計測前に一旦トリガーピンをLowに
            delayMicroseconds(2);

            digitalWrite(trigPin, HIGH);             // トリガーピンを10μsの時間だけHighに
            delayMicroseconds(10);
            digitalWrite(trigPin, LOW);

            Duration = pulseIn(echoPin, HIGH);      // エコーピンからの入力
            Distance = Duration * (331.5 + 0.6 * T) * 100 / 1000000;   // 音速を掛けて単位をcmに
            if(Distance > 400){
              Distance = 400;
            }

            if (Distance > DistanceToTarget) {
                Speed = (Distance - DistanceToTarget) * 20;
                if (Speed > 64) {
                    Speed = 64;
                }
                P_Speed(Speed);
            }

            else {
                P_Speed(0);
                armMove(StoG);
                delay(1000);
                armMove(GtoS);
                delay(1000);
                AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);
                preYaw = Yaw;
                Flag = false;
            }
        }

        if (Flag == false) {
            AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);
            if (Flag1 == true) {
                if (Yaw - preYaw < 180) {
                    Serial.println(Yaw);
                    digitalWrite(8, HIGH);
                    digitalWrite(7, LOW);
                    analogWrite(5, 64);
                    analogWrite(6, 64);
                }
                else {
                    Flag1 = false;
                }
            }
            else {
                digitalWrite(trigPin, LOW);              // 計測前に一旦トリガーピンをLowに
                delayMicroseconds(2);

                digitalWrite(trigPin, HIGH);             // トリガーピンを10μsの時間だけHighに
                delayMicroseconds(10);
                digitalWrite(trigPin, LOW);

                Duration = pulseIn(echoPin, HIGH);
                Distance = Duration * (331.5 + 0.6 * T) * 100 / 1000000;
                if(Distance > 400){
                  Distance = 400;
                }

                if (Distance > DistanceToTarget2) {
                    Speed = (Distance - DistanceToTarget2) * 20;
                    if (Speed > 64) {
                        Speed = 64;
                    }
                    else {
                        Speed = Speed;
                    }
                    P_Speed(Speed);
                }
                else {
                    P_Speed(0);
                    armMove(StoD);
                    delay(1000);
                    armMove(DtoS);
                    delay(1000);
                    receiver.resume();
                    while (flag_mode){
                      if (receiver.decode(&results)) {
                        Flag = true;
                        Flag1 = true;
                        flag_mode = false;
                        receiver.resume();
                      }
                    }
                }
            }
        }
    }
}

void P_Speed(int speed) {
    AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);
    int R = (Yaw - preYaw) * Kp + speed;
    if (R > UpperLimit)
    {
        R = UpperLimit;
    }
    else if (R < 10)
    {
        R = 10;
    }
    int L = (preYaw - Yaw) * Kp + speed;
    if (L > UpperLimit)
    {
        L = UpperLimit;
    }
    else if (L < 10)
    {
        L = 10;
    }
    digitalWrite(8, HIGH);
    digitalWrite(7, HIGH);
    analogWrite(5, R);
    analogWrite(6, L);
    preYaw = Yaw;
    delay(50);
}

void P_Speed_Back(int speed) {
    AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);
    int R = (Yaw - preYaw) * Kp + speed;
    if (R > UpperLimit)
    {
        R = UpperLimit;
    }
    else if (R < 0)
    {
        R = 0;
    }
    int L = (preYaw - Yaw) * Kp + speed;
    if (L > UpperLimit)
    {
        L = UpperLimit;
    }
    else if (L < 0)
    {
        L = 0;
    }
    digitalWrite(8, LOW);
    digitalWrite(7, LOW);
    analogWrite(5, R);
    analogWrite(6, L);
    preYaw = Yaw;
    delay(50);
}

void armSetup() {
    Servo0.attach(10);
    Servo1.attach(11);
    Servo2.attach(A0);
    Servo3.attach(A1);
    Servo0.write(arg0IP);
    Servo1.write(arg1IP);
    Servo2.write(arg2IP);
    Servo3.write(arg3open);
    delay(1000);
}

void armMove(int action) {
    switch (action)
    {
    case StoG:
        for (int k = 1; k <= 100; k++) {
            Servo0.write(arg0IP + a * k / 100);
            Servo1.write(arg1IP + b * k / 100);
            Servo2.write(arg2IP + c * k / 100);
            Servo3.write(arg3open);
            delay(10);
        }
        delay(500);
        Servo3.write(arg3close);
        break;

    case GtoS:
        for (int k1 = 1; k1 <= 100; k1++) {
            Servo0.write(arg0GP + e * k1 / 100);
            Servo1.write(arg1GP + f * k1 / 100);
            Servo2.write(arg2GP + g * k1 / 100);
            Servo3.write(arg3close);
            delay(10);
        }
        break;

    case StoD:
        for (int k2 = 1; k2 <= 100; k2++) {
            Servo0.write(arg0DP + Da * k2 / 100);
            Servo1.write(arg1DP + Db * k2 / 100);
            Servo2.write(arg2DP + Dc * k2 / 100);
            Servo3.write(arg3close);
            delay(10);
        }
        Servo3.write(arg3open);
        break;

    case DtoS:
        for (int k3 = 1; k3 <= 100; k3++) {
            Servo0.write(arg0DP + De * k3 / 100);
            Servo1.write(arg1DP + Df * k3 / 100);
            Servo2.write(arg2DP + Dg * k3 / 100);
            Servo3.write(arg3open + Dd * k3 / 100);
            delay(10);
        }
        break;
    }
}

void remotecontrolcar(int Signal) {
    AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);
    preYaw = Yaw;
    switch (Signal) {
    case channel1:
        digitalWrite(3, HIGH);
        digitalWrite(7, HIGH);
        digitalWrite(8, HIGH);
        analogWrite(5, SPEED);
        analogWrite(6, SPEED / 2);
        flag_up = false;
        flag_down = false;
        flag_servo0 = false;
        flag_servo1 = false;
        flag_servo2 = false;
        flag_servo3 = false;
        break;

    case channel2:
        digitalWrite(3, HIGH);
        P_Speed(SPEED);
        /*
        digitalWrite(7, HIGH);
        digitalWrite(8, HIGH);
        analogWrite(5, SPEED);
        analogWrite(6, SPEED);*/
        flag_up = false;
        flag_down = false;
        flag_servo0 = false;
        flag_servo1 = false;
        flag_servo2 = false;
        flag_servo3 = false;
        break;

    case channel3:
        digitalWrite(3, HIGH);
        digitalWrite(7, HIGH);
        digitalWrite(8, HIGH);
        analogWrite(5, SPEED / 2);
        analogWrite(6, SPEED);
        flag_up = false;
        flag_down = false;
        flag_servo0 = false;
        flag_servo1 = false;
        flag_servo2 = false;
        flag_servo3 = false;
        break;

    case channel4:
        digitalWrite(3, HIGH);
        digitalWrite(7, HIGH);
        digitalWrite(8, LOW);
        analogWrite(5, SPEED);
        analogWrite(6, SPEED);
        flag_up = false;
        flag_down = false;
        flag_servo0 = false;
        flag_servo1 = false;
        flag_servo2 = false;
        flag_servo3 = false;
        break;

    case channel5:
        digitalWrite(3, LOW);
        digitalWrite(7, HIGH);
        digitalWrite(8, HIGH);
        analogWrite(5, 0);
        analogWrite(6, 0);
        flag_up = false;
        flag_down = false;
        flag_servo0 = false;
        flag_servo1 = false;
        flag_servo2 = false;
        flag_servo3 = false;
        break;

    case channel6:
        digitalWrite(3, HIGH);
        digitalWrite(7, LOW);
        digitalWrite(8, HIGH);
        analogWrite(5, SPEED);
        analogWrite(6, SPEED);
        flag_up = false;
        flag_down = false;
        flag_servo0 = false;
        flag_servo1 = false;
        flag_servo2 = false;
        flag_servo3 = false;
        break;

    case channel7:
        digitalWrite(3, HIGH);
        digitalWrite(7, LOW);
        digitalWrite(8, LOW);
        analogWrite(5, SPEED);
        analogWrite(6, SPEED / 2);
        flag_up = false;
        flag_down = false;
        flag_servo0 = false;
        flag_servo1 = false;
        flag_servo2 = false;
        flag_servo3 = false;
        break;

    case channel8:
        digitalWrite(3, HIGH);
        P_Speed_Back(SPEED);
        /*
        digitalWrite(7, LOW);
        digitalWrite(8, LOW);
        analogWrite(5, SPEED);
        analogWrite(6, SPEED);*/
        flag_up = false;
        flag_down = false;
        flag_servo0 = false;
        flag_servo1 = false;
        flag_servo2 = false;
        flag_servo3 = false;
        break;

    case channel9:
        digitalWrite(3, HIGH);
        digitalWrite(7, LOW);
        digitalWrite(8, LOW);
        analogWrite(5, SPEED / 2);
        analogWrite(6, SPEED);
        flag_up = false;
        flag_down = false;
        flag_servo0 = false;
        flag_servo1 = false;
        flag_servo2 = false;
        flag_servo3 = false;
        break;

    case volumeup:
        flag_up = true;
        flag_down = false;
        flag_servo0 = false;
        flag_servo1 = false;
        flag_servo2 = false;
        flag_servo3 = false;
        break;

    case volumedown:
        flag_up = false;
        flag_down = true;
        flag_servo0 = false;
        flag_servo1 = false;
        flag_servo2 = false;
        flag_servo3 = false;
        break;

    case blueButton:
        flag_up = false;
        flag_down = false;
        flag_servo0 = true;
        flag_servo1 = false;
        flag_servo2 = false;
        flag_servo3 = false;
        break;

    case redButton:
        flag_up = false;
        flag_down = false;
        flag_servo0 = false;
        flag_servo1 = true;
        flag_servo2 = false;
        flag_servo3 = false;
        break;

    case greenButton:
        flag_up = false;
        flag_down = false;
        flag_servo0 = false;
        flag_servo1 = false;
        flag_servo2 = true;
        flag_servo3 = false;
        break;

    case yellowButton:
        flag_up = false;
        flag_down = false;
        flag_servo0 = false;
        flag_servo1 = false;
        flag_servo2 = false;
        flag_servo3 = true;
        break;

    case decitonButton:
        arg0GP = arg0;
        arg1GP = arg1;
        arg2GP = arg2;
        arg3close = arg3;
        flag_up = false;
        flag_down = false;
        flag_servo0 = false;
        flag_servo1 = false;
        flag_servo2 = false;
        flag_servo3 = false;
        break;

    case modechange:
        digitalWrite(3, HIGH);
        flag_mode = true;
        break;

    default:
        if (flag_up) {
            SPEED += 1;
            if (SPEED > 255) {
                SPEED = 255;
            }
            analogWrite(5, SPEED);
            analogWrite(6, SPEED);
        }

        if (flag_down)
        {
            SPEED -= 1;
            if (SPEED < 0) {
                SPEED = 0;
            }
            analogWrite(5, SPEED);
            analogWrite(6, SPEED);
        }

        if (flag_servo0) {
            arg0 += 3;
            if (arg0 > 180) {
                arg0 = 0;
            }
            Servo0.write(arg0);
        }

        if (flag_servo1) {
            arg1 += 3;
            if (arg1 > 180) {
                arg1 = 0;
            }
            Servo1.write(arg1);
        }

        if (flag_servo2) {
            arg2 += 3;
            if (arg2 > 180) {
                arg2 = 0;
            }
            Servo2.write(arg2);
        }

        if (flag_servo3) {
            arg3 += 3;
            if (arg3 > 180) {
                arg3 = 0;
            }
            Servo3.write(arg3);
        }
    }
    delay(10);
}
