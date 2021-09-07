#include "mbed.h"
#include "TextLCD.h"
#include "rtos.h"
#include <math.h>
#include <string>

//割り込み関係
Thread thread1;
Thread thread2;
Ticker inverter;
Ticker spreader;

//入出力定義
DigitalOut uPhase(D2);
DigitalOut vPhase(D3);
DigitalOut wPhase(D4);
Serial pc(USBTX,USBRX);
I2CSlave slave(D14,D15);


//乱数
//const int randomiser[] = {-159.38,103.27,-6.07,142.7,-160.24,62.72,-396.24,22.31,-155.52,7.2,-306.91,45.37,-305.75,79.33,-9.96,61.41,-148.94,97.55,-243.3,44.84,-47.43,43.68,-322.05,169.44,-172.51,111.19,-128.33,35.16,-230.36,67.69,-369.25,34.8,-117.16,126.1,-27.13,132.91,-212.36,168.73,-89.92,197.34,-184.9,194.37,-14.05,127.14,-250.47,184.85,-388.39,48.62,-181.54,68.05,-41.79,109.77,-264.08,34.07,-232.59,2.38,-249.63,31.24,-342.37,105.29,-274.49,171.47,-348.05,135.21,-37.74,85.71,-40.29,72.09,-6.15,105.4,98.17,-101.46,80.67,-2.89,32.96,-38.24,142.32,-117.58,362.85,-81.46,329.28,-169.29,28.78,-53.87,311.6,-15.2,251.32,-157.56,201.07,-175.7,32.52,-120.94,210.76,-21.66,230.22,-80.92,284.04,-77.8,184.96,-100.32,144.78,-29.39,47.65,-164.68,256.95,-128.78,17.23,-43.72,37.39,-57.88,76.77,-317.51,107.55,-89.13,53.6,-147.14,125.56,-172.02,179.71,-25.09,183.51,-105.57,199.06,-20.97,150.42,-327.35,133.03,17.18,-34.36,42.16,-35.21,183.59,-36.84,99.98,-82.71,355.69,-150.37,233.48,-136.7,329.46,-88.46,373.45,-32.97,374.99,-173.31,356.51,-62.98,215.45,-7.7,333.56,-42.16,368.1,-166.83,251.92,-61.25,265.91,-20.64,172.43,-29.21,225.53,-37.39,57.88,-76.77,159.38,-103.27,6.07,-142.7,160.24,-62.72,396.24,-22.31,155.52,281.04,-74.44,55.83,-63.95,130.44,-179.82,205.85,-19.9,104.62,-107.41,68.49,-197.99,341.96,-79.2,221.01,-125.24,378.57,-90.62,-140.81,156.81,101.46,-80.67,2.89,-32.96,38.24,-142.32,117.58,-362.85,81.46,-329.28,169.29,-28.78,53.87,-311.6,15.2,-251.32,157.56,-201.07,175.7,-32.52,120.94,-210.76,21.66,-230.22,80.92,-284.04,77.8,-184.96,100.32,-144.78,29.39,-47.65,164.68,-256.95,128.78,-17.23,43.72,-281.04,74.44,-55.83,63.95,-130.44,179.82,-205.85,19.9,-104.62,107.41,-68.49,-132.91,212.36,-168.73,89.92,-197.34,184.9,-194.37,14.05,-127.14,250.47,-184.85,388.39,-48.62,181.54,-68.05,41.79,-109.77,264.08,-34.07,232.59,-2.38,197.99,-341.96,79.2,-221.01,125.24,-378.57,90.62,-17.18,34.36,-42.16,35.21,-183.59,36.84,-99.98,82.71,-355.69,150.37,-233.48,136.7,-329.46,88.46,-373.45,32.97,-374.99,173.31,-356.51,62.98,-215.45,7.7,-333.56,42.16,-368.1,166.83,-251.92,61.25,-265.91,20.64,-172.43,29.21,-225.53,-7.2,306.91,-45.37,305.75,-79.33,9.96,-61.41,148.94,-97.55,243.3,-44.84,47.43,-43.68,322.05,-169.44,172.51,-111.19,128.33,-35.16,230.36,-67.69,369.25,-34.8,117.16,-126.1,27.13,249.63,-31.24,342.37,-105.29,274.49,-171.47,140.81,-156.81,317.51,-107.55,89.13,-53.6,147.14,-125.56,172.02,-179.71,25.09,-183.51,105.57,-199.06,20.97,-150.42,327.35,-133.03,348.05,-135.21,37.74,-85.71,40.29,-72.09,6.15,-105.4,-98.1700} ;
int irand = 0;


//変数定義
int notch = 2;
string notchName[] = {"B2", "B1", "N", "P1", "P2"};
float speed;
const float gear_rate = 7.07;
const float wheel_diameter = 0.89; //m
const float t1 = 0.04; //(s)
const int step_notch = 5;
const float accel[step_notch] = {-3, -1, 0, 1, 2.5}; //Km/h/s
const int accel_Vmax[step_notch] = {155, 155, 155, 60, 150}; //Km/h

const float res = 0.05; //(ms)
float v = 0;
double carrier_freq = 0;
double signal_freq = 0;
float signal_voltage = 0;
const int step_SIN = 360; //sin1周期の刻み
const int step_TRI = 360; //三角波1周期の刻み
float table_SIN[step_SIN];
float table_TRI[step_TRI];
float table_TRI_W3[step_TRI];
float table_SQR[step_TRI];
float table_carr_11P[360] = {0,-0.1,-0.2,-0.3,-0.4,-0.5,-0.6,-0.7,-0.8,-0.9,-1,-0.9,-0.8,-0.7,-0.6,-0.5,-0.4,-0.3,-0.2,-0.1,0,0.125,0.25,0.375,0.5,0.625,0.75,0.875,1,0.875,0.75,0.625,0.5,0.375,0.25,0.125,0,-0.125,-0.25,-0.375,-0.5,-0.625,-0.75,-0.875,-1,-0.875,-0.75,-0.625,-0.5,-0.375,-0.25,-0.125,0,0.125,0.25,0.375,0.5,0.625,0.75,0.875,1,0.875,0.75,0.625,0.5,0.375,0.25,0.125,0,-0.125,-0.25,-0.375,-0.5,-0.625,-0.75,-0.875,-1,-0.875,-0.75,-0.625,-0.5,-0.375,-0.25,-0.125,0,0.125,0.25,0.375,0.5,0.625,0.75,0.875,1,0.875,0.75,0.625,0.5,0.375,0.25,0.125,0,-0.125,-0.25,-0.375,-0.5,-0.625,-0.75,-0.875,-1,-0.875,-0.75,-0.625,-0.5,-0.375,-0.25,-0.125,0,0.125,0.25,0.375,0.5,0.625,0.75,0.875,1,0.875,0.75,0.625,0.5,0.375,0.25,0.125,0,-0.125,-0.25,-0.375,-0.5,-0.625,-0.75,-0.875,-1,-0.875,-0.75,-0.625,-0.5,-0.375,-0.25,-0.125,0,0.125,0.25,0.375,0.5,0.625,0.75,0.875,1,0.875,0.75,0.625,0.5,0.375,0.25,0.125,0,-0.125,-0.25,-0.375,-0.5,-0.625,-0.75,-0.875,-1,-0.875,-0.75,-0.625,-0.5,-0.375,-0.25,-0.125,0,0.125,0.25,0.375,0.5,0.625,0.75,0.875,1,0.875,0.75,0.625,0.5,0.375,0.25,0.125,0,-0.125,-0.25,-0.375,-0.5,-0.625,-0.75,-0.875,-1,-0.875,-0.75,-0.625,-0.5,-0.375,-0.25,-0.125,0,0.125,0.25,0.375,0.5,0.625,0.75,0.875,1,0.875,0.75,0.625,0.5,0.375,0.25,0.125,0,-0.125,-0.25,-0.375,-0.5,-0.625,-0.75,-0.875,-1,-0.875,-0.75,-0.625,-0.5,-0.375,-0.25,-0.125,0,0.125,0.25,0.375,0.5,0.625,0.75,0.875,1,0.875,0.75,0.625,0.5,0.375,0.25,0.125,0,-0.125,-0.25,-0.375,-0.5,-0.625,-0.75,-0.875,-1,-0.875,-0.75,-0.625,-0.5,-0.375,-0.25,-0.125,0,0.125,0.25,0.375,0.5,0.625,0.75,0.875,1,0.875,0.75,0.625,0.5,0.375,0.25,0.125,0,-0.125,-0.25,-0.375,-0.5,-0.625,-0.75,-0.875,-1,-0.875,-0.75,-0.625,-0.5,-0.375,-0.25,-0.125,0,0.125,0.25,0.375,0.5,0.625,0.75,0.875,1,0.875,0.75,0.625,0.5,0.375,0.25,0.125,0,-0.125,-0.25,-0.375,-0.5,-0.625,-0.75,-0.875,-1,-0.875,-0.75,-0.625,-0.5,-0.375,-0.25,-0.125,0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1,0.9,0.8,0.7,0.6,0.5,0.4,0.3,0.2,0.1
};
float rondomiser = 0;
float spreadWidth = 100;


//u,v,w相の順
float carrier[3];
float signal[3];
double phase_carrier[3] = {0, step_SIN / 3, 2 * step_SIN / 3};
double phase_signal[3] = {0, step_TRI / 3, 2 * step_TRI / 3};
double phase_signalp[3] = {0, step_TRI / 3, 2 * step_TRI / 3};

int phase_carrier1[3] = {0, step_SIN / 3, 2 * step_SIN / 3};
int phase_signal1[3] = {0, step_TRI / 3, 2 * step_TRI/ 3};
int phase_signal1p[3] = {0, step_TRI / 3, 2 * step_TRI / 3};

int mode = 0 ; //0:非同期 1:同期 2:広域3パルス 3:1パルス 11:11P
int Spread = 0 ; //0;通常 1:スペクトラム拡散
int Pile = 0;
int lastmode;

int i;
int j;

int ctoi(char c) {
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    return 0;
}


//各車両の設定
void Set_carrier_freq_hitachikoki(float signal_freq){
        if ((signal_freq >= 0) and (signal_freq < 20)) {
        carrier_freq = 400;
        mode = 0;
        Spread = 0;
        Pile = 0;
    }
    if ((signal_freq >= 20) and (signal_freq < 24)) {
        carrier_freq = 15 * signal_freq ; 
        mode = 1;
        Spread = 0;
        Pile = 0;
    }
    if ((signal_freq >= 24) and (signal_freq < 32)) {
        carrier_freq = 11 * signal_freq ;
        mode = 11;
        Spread = 0;
        Pile = 0;
    }
    
    if ((signal_freq >= 32) and (signal_freq < 36)) {
        carrier_freq = 5 * signal_freq ;
        mode = 1;
        Spread = 0;
        Pile = 0;
    }
    
    if ((signal_freq >= 36) and (signal_freq < 38)) {
        carrier_freq = 3 * signal_freq ;
        mode = 1;
        Spread = 0;
        Pile = 0;
    }
    
    if ((signal_freq >= 38) and (signal_freq < 39)) {
        carrier_freq = signal_freq ;
        mode = 2;
        Spread = 0;
        Pile = 0;
    }
    }


void Set_signal_voltage_hitachikoki(float signal_freq) {
    if((signal_freq >= 0) and (signal_freq < 2)) {
        signal_voltage = 0.06 + signal_freq * 0.07 ;
    }
    if((signal_freq >= 2) and (signal_freq <39 )) {
        signal_voltage = 0.2 + 0.8 * (signal_freq - 2) / 37  ;
    }
    if(signal_freq >= 39) {
        signal_voltage = 1 ;
    }  
}

void Set_carrier_freq_toyo(float signal_freq) {
if((signal_freq >= 0) and (signal_freq < 4)) {
  carrier_freq = 260;
  Spread = 1;
  mode = 0;
}
if((signal_freq >= 4) and (signal_freq < 27)) {
  carrier_freq = 260  + 165 * (signal_freq-4) / 23;
  Spread = 1;
  mode = 0;
}
if((signal_freq >= 27) and (signal_freq < 43)) {
  carrier_freq = 9 * signal_freq ;
  Spread = 0;
  mode = 1;
}
if((signal_freq >= 43) and (signal_freq < 52)) {
  carrier_freq = 5 * signal_freq ;
    Spread = 0;
    mode = 1;
}
if((signal_freq >= 52) and (signal_freq < 60)) {
  carrier_freq = 3 * signal_freq ;
    Spread = 0;
    mode = 1;
}
if(signal_freq >= 60) {
  carrier_freq = signal_freq ;
    Spread = 0;
    mode = 2;
}
}

void Set_carrier_freq_5000(float signal_freq) {
    if((signal_freq >= 0) and (signal_freq < 23)) {
        carrier_freq = 740;
        mode = 0;
        Spread = 1;
    }
    if((signal_freq >= 23) and (signal_freq < 44)) {
        carrier_freq = 740 - 40 * (signal_freq - 23) / 19;
        mode = 0;
        Spread = 1;
    }
    if((signal_freq >= 44) and (signal_freq < 53)) {
        carrier_freq = 700 + 1100 * (signal_freq - 44) /9 ;
        mode = 0;
    }
    if(signal_freq >= 53) {
        carrier_freq = signal_freq ;
        mode = 2;
    }
}

void Set_carrier_freq_5000bk(float signal_freq) {
    if((signal_freq >= 0) and (signal_freq < 4)) {
        carrier_freq = 200;
        mode = 0;
    }   
        if((signal_freq >= 4) and (signal_freq < 23)) {
        carrier_freq = 740;
        mode = 0;
    }
    if((signal_freq >= 23) and (signal_freq < 44)) {
        carrier_freq = 740 - 40 * (signal_freq - 23) / 19;
        mode = 0;
    }
    if((signal_freq >= 44) and (signal_freq < 53)) {
        carrier_freq = 700 + 1100 * (signal_freq - 44) /9 ;
        mode = 0;
    }
    if(signal_freq >= 53) {
        carrier_freq = signal_freq ;
        mode = 2;
    }
}
void Set_carrier_freq_toyoigbt(float signal_freq) {
if((signal_freq >= 0) and (signal_freq < 34)) {
  carrier_freq = 1050;
  mode = 0;
}
if((signal_freq >= 34) and (signal_freq < 54)) {
  carrier_freq = 9 * signal_freq;
  mode = 1;
}
if((signal_freq >= 54) and (signal_freq < 140)) {
  carrier_freq = signal_freq ;
  mode = 2;
}
}

void Set_carrier_freq_toyoigbtbk(float signal_freq) {
if((signal_freq >= 0) and (signal_freq < 33)) {
  carrier_freq = 1050;
  mode = 0;
}
if((signal_freq >= 33) and (signal_freq < 98)) {
  carrier_freq = 9 * signal_freq;
  mode = 1;
}
if((signal_freq >= 98) and (signal_freq < 140)) {
  carrier_freq = signal_freq ;
  mode = 2;
}
}
void Set_signal_voltage_toyoigbt(float signal_freq) {
    if((signal_freq >= 0) and (signal_freq < 2)) {
        signal_voltage = 0.06 + signal_freq * 0.07 ;
    }
    if((signal_freq >= 2) and (signal_freq <50 )) {
        signal_voltage = 0.2 + 0.8 * (signal_freq - 2) / 48  ;
    }
    if((signal_freq >= 50) and (signal_freq <54 )) {
        signal_voltage = 1.0 + 0.4 * (signal_freq - 50) / 4;
    }  
    if((signal_freq >= 54) and (signal_freq <55 )) {
        signal_voltage = 0.9 + (signal_freq - 54);
    }
    if((signal_freq >= 54) and (signal_freq <140 )) {
        signal_voltage = 1.0;
    }    
}
void Set_signal_voltage_toyoigbtbk(float signal_freq) {
    if((signal_freq >= 0) and (signal_freq < 2)) {
        signal_voltage = 0.06 + signal_freq * 0.07 ;
    }
    if((signal_freq >= 2) and (signal_freq <55 )) {
        signal_voltage = 0.2 + 0.8 * (signal_freq - 2) / 53  ;
    }
    if((signal_freq >= 55) and (signal_freq <100 )) {
        signal_voltage = 1;
    }  
}
void Set_carrier_freq_N1000(float signal_freq) {
    if((signal_freq >= 0) && (signal_freq < 0.7)) {
        carrier_freq = 175;
        mode = 0;
        Spread = 0;
    }
    if((signal_freq >= 0.7) && (signal_freq < 1.4)) {
        carrier_freq = 196;
        mode = 0;
        Spread = 0;
    }
    if((signal_freq >= 1.4) && (signal_freq < 2.1)) {
        carrier_freq = 223;
        mode = 0;
        Spread = 0;
    }
    if((signal_freq >= 2.1) && (signal_freq < 2.8)) {
        carrier_freq = 233;
        mode = 0;
        Spread = 0;
    }
    if((signal_freq >= 2.8) && (signal_freq < 3.5)) {
        carrier_freq = 262;
        mode = 0;
        Spread = 0;
    }
    if((signal_freq >= 3.5) && (signal_freq < 4.2)) {
        carrier_freq = 294;
        mode = 0;
        Spread = 0;
    }
    if((signal_freq >= 4.2) && (signal_freq < 4.9)) {
        carrier_freq = 311;
        mode = 0;
        Spread = 0;
    }
    if((signal_freq >= 4.9) && (signal_freq < 5.6)) {
        carrier_freq = 350;
        mode = 0;
        Spread = 0;
    }
    if((signal_freq >= 5.6) && (signal_freq < 24)) {
        carrier_freq = 400;
        mode = 0;
        Spread = 0;
    }
    if((signal_freq >= 24) && (signal_freq < 26.5)) {
        carrier_freq = 18 * signal_freq;
        mode = 1;
        Spread = 0;
    }
    if((signal_freq >= 26.5) && (signal_freq < 30.7)) {
        carrier_freq = 15 * signal_freq ;
        mode = 1;
        Spread = 0;
    }
    if((signal_freq >= 30.7) && (signal_freq < 36.2)) {
        carrier_freq = 12 * signal_freq ;
        mode = 1;
        Spread = 0;
    }
    if((signal_freq >= 36.2) && (signal_freq < 44)) {
        carrier_freq = 7 * signal_freq ;
        mode = 1;
        Spread = 0;
    }
    if((signal_freq >= 44) && (signal_freq < 50)) {
        carrier_freq = 5 * signal_freq ;
        mode = 1;
        Spread = 0;
    }
    if((signal_freq >= 50) && (signal_freq < 180)) {
        carrier_freq = signal_freq ;
        mode = 2;
        Spread = 0;
    }
}

void Set_carrier_freq_N1000bk(float signal_freq) {
    if((signal_freq >= 0) && (signal_freq < 24)) {
        carrier_freq = 400;
        mode = 0;
        Spread = 0;
    }
    if((signal_freq >= 24) && (signal_freq < 26.5)) {
        carrier_freq = 18 * signal_freq;
        mode = 1;
        Spread = 0;
    }
    if((signal_freq >= 26.5) && (signal_freq < 30.7)) {
        carrier_freq = 15 * signal_freq ;
        mode = 1;
        Spread = 0;
    }
    if((signal_freq >= 30.7) && (signal_freq < 36.2)) {
        carrier_freq = 12 * signal_freq ;
        mode = 1;
        Spread = 0;
    }
    if((signal_freq >= 36.2) && (signal_freq < 44)) {
        carrier_freq = 7 * signal_freq ;
        mode = 1;
        Spread = 0;
    }
    if((signal_freq >= 44) && (signal_freq < 50)) {
        carrier_freq = 5 * signal_freq ;
        mode = 1;
        Spread = 0;
    }
    if((signal_freq >= 50) && (signal_freq < 180)) {
        carrier_freq = signal_freq ;
        mode = 2;
        Spread = 0;
    }
}
void Set_signal_voltage_N1000(float signal_freq) {
    if((signal_freq >= 0) and (signal_freq < 2)) {
        signal_voltage = 0.06 + signal_freq * 0.07 ;
    }
    if((signal_freq >= 2) and (signal_freq <80 )) {
        signal_voltage = 0.2 + 0.8 * (signal_freq - 2) / 78  ;
    }
    if((signal_freq >= 80) and (signal_freq <130)) {
        signal_voltage = 1;
    }  
}

int k;
void setSignal_freq (){
    while (true) {
        for (k = 0; k < step_notch; k++) {
            if ((notch == k) && (signal_freq >= 0) && (signal_freq <= accel_Vmax[k] *gear_rate / 4.863) ){
            signal_freq = signal_freq + accel[k] * t1 * gear_rate / 4.863 ; //加速度(Hz/s)
            }
        }
        if(signal_freq < 0){
            signal_freq = 0;
        }
        wait(t1);
    }
}

void sendInfoPC() {
    while (true) {
        speed = 4.863 * signal_freq / gear_rate ;
        pc.printf("Speed = %5.2f Km/h ", speed);
        pc.printf("m %5.2f Hz ", signal_freq);
        pc.printf("c %5.2f Hz ", carrier_freq);
        pc.printf("Notch = %s\r\n" , notchName[notch]);
        pc.printf("mode = %d moduration = %5.2f\r\n\r\n",mode,signal_voltage);
        wait(0.4);
    }
}

void Ad_phase (float ADtime, float frq_carrier, float frq_signal) {
    if(((lastmode != 1) && (mode == 1)) or ((lastmode != 2) && (mode == 2))) {
        /*for(i = 0; i<3; i++){
            phase_carrier[i] = i * step_SIN / 3;
            phase_signal[i] = i * step_TRI / 3;
            phase_carrier1[i] = i * step_SIN / 3;
            phase_signal1[i] = i * step_TRI / 3 ;
        }
        */
    } else {
    for (i = 0; i < 3; i++) {
        phase_carrier[i] = phase_carrier[i] + step_TRI * frq_carrier * ADtime / 1000;
        phase_signal[i] = phase_signal[i] + step_SIN * frq_signal * ADtime / 1000;
            
        if ((phase_carrier[i] >= step_TRI - 1) && (phase_signal[i] < step_SIN)) {
            phase_carrier[i] = phase_carrier[i] - step_TRI;
        }
        if ((phase_carrier[i] < step_TRI) && (phase_signal[i] >= step_SIN - 1)) {
            phase_signal[i] = phase_signal[i] - step_SIN;
        }
        if ((phase_carrier[i] >= step_TRI - 1) && (phase_signal[i] >= step_SIN - 1)) {
            phase_carrier[i] = phase_carrier[i] - step_TRI;
            phase_signal[i] = phase_signal[i] - step_SIN;
        }
        phase_carrier1[i] = phase_carrier[i]; //整数に丸めておく
        phase_signal1[i] = phase_signal[i];
    } 
    //重畳  
    for (i = 0; i < 3; i++) {
        phase_signalp[i] = phase_signalp[i] + step_SIN * frq_signal * 3 * ADtime / 1000;   
        if (phase_signalp[i] >= step_SIN - 1) {
            phase_signalp[i] = phase_signalp[i] - step_SIN;
        }
        phase_signal1p[i] = phase_signalp[i];
    }
}
}

void compare () {
    if (signal_freq < 0.12) {
        uPhase = 0;
        vPhase = 0;
        wPhase = 0;
    } else {
        if ((mode == 0) or (mode == 1)) {
            for (i = 0; i < 3; i++) {
                signal[i] = table_SIN[phase_signal1[i]];
                carrier[i] = table_TRI[phase_carrier1[i]];
                if (Pile == 1) {
                    signal[i] = signal[i] + table_SIN[phase_signal1p[i]] / 6;
                } 
            }
        }
        if (mode == 2) {
            for (i = 0; i < 3; i++) {
                signal[i] = table_SQR[phase_signal1[i]];
                carrier[i] = table_TRI_W3[phase_signal1[i]];
            }
        }
        
        if (mode == 11) {
            for (i = 0; i < 3; i++) {
                signal[i] = table_SIN[phase_signal1[i]] ;
                carrier[i] = table_SQR[phase_carrier1[i]];
            }
        }
        if (mode == 999){
            for (i = 0; i < 3; i++) {
                carrier[i] = table_TRI[phase_carrier1[i]];
            }
                signal[0] = 0.9 * signal_freq / 4.0;
                signal[1] = 0.9 * signal_freq / 4.0;
                signal[2] = 0;
        }

        if ((mode == 3) or (mode == 2) ) {
            if (signal_voltage * signal[0] > carrier[0]) {
                uPhase = 1;
            } else {
                uPhase = 0;
            }
            if (signal_voltage * signal[1] > carrier[1]) {
                vPhase = 1;
            } else {
                vPhase = 0;
            }
            if (signal_voltage * signal[2] > carrier[2]) {
                wPhase = 1;
            } else {
                wPhase = 0;
            }
        } else {
            if (signal_voltage * signal[0] > carrier[0]) {
                uPhase = 1;
            } else {
                uPhase = 0;
            }
            if (signal_voltage * signal[1] > carrier[0]) {
                vPhase = 1;
            } else {
                vPhase = 0;
            }
            if (signal_voltage * signal[2] > carrier[0]) {
                wPhase = 1;
            } else {
                wPhase = 0;
            }
        }
    }
}

void spread(){
        if(Spread == 1){
            rondomiser = rand()%50;
        }
}

void pwmOut(){ 
        switch(notch){
            case 0:
            case 1:
                Set_carrier_freq_N1000bk(signal_freq);
                Set_signal_voltage_N1000(signal_freq);
            break;
            case 2:
            case 3:
            case 4:
                Set_carrier_freq_N1000(signal_freq);
                Set_signal_voltage_N1000(signal_freq);
            break;
        }
        if (Spread == 1) {
            carrier_freq = carrier_freq - 25 + rondomiser;
        }
        Ad_phase(res, carrier_freq, signal_freq);
        compare();
        lastmode = mode;
}

int main() {    
    pc.printf("VVVF Inverter for mbed\r\n");
    //波形テーブル作成
    for (i = 0; i < step_SIN; i++) {
        table_SIN[i] = sin(2 * 3.1415 * i / step_SIN);
    }
    for (i = 0; i < step_TRI; i++) {
        if (i < step_TRI / 2 ) {
            table_TRI[i] = -1.000 + 2.000 * i / (step_TRI / 2);
            } else {
            table_TRI[i] = 1.000 - 2.000 * (i - step_TRI / 2) / (step_TRI / 2);
            } 
    }
    for (i = 0; i < step_TRI; i++) {
        if (i < step_TRI / 6 ) {
            table_TRI_W3[i] = 1.000 - 1.000 * i / (step_TRI / 6);
        }
        if ((i >= step_TRI / 6 ) && (i < step_TRI / 3)) {
            table_TRI_W3[i] = 0;
        }
        if ((i >= step_TRI / 3 ) && (i < step_TRI / 2)) {
            table_TRI_W3[i] = 1.000 * (i - (step_TRI / 3))/(step_TRI / 6);
        }
        if (i >= step_TRI / 2 ) {
            table_TRI_W3[i] = -1.000 * table_TRI_W3[(i - (step_TRI / 2))];
        }
    }
    for (i = 0; i <step_TRI; i++) {
        if (i < step_TRI / 2 ) {
            table_SQR[i] = 1.000;
            } else {
            table_SQR[i] = -1.000;
        }
    } 
    pc.printf("Table Generated\r\n");
    wait(0.25);
    thread1.start(sendInfoPC);
    thread2.start(setSignal_freq);
 //   spreader.attach_us(&spread, 10000);
 //   thread1.set_priority(osPriorityLow);
    inverter.attach_us(&pwmOut, res *1000);
    slave.address(0x08);
    pc.printf("Start\r\n");
    
    while(1) {
        char buf[1];

        int receive = slave.receive();
        switch (receive) {
            //case I2CSlave::ReadAddressed:
                //slave.write(msg, strlen(msg) + 1); // Includes null char
                //break;
            //case I2CSlave::WriteGeneral:
                //slave.read(buf, 1);
                //break;
            case I2CSlave::WriteAddressed:
                slave.read(buf, 1);
                pc.printf("Read A: %s\r\n", buf);
                if(ctoi(buf[0]) >=0 and ctoi(buf[0])<5) {
                notch = ctoi(buf[0]);
                }
                break;
        }
        //for(int i = 0; i < 10; i++) buf[i] = 0;    // Clear buffer
    }
}