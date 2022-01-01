
#ifndef IP_CONFIG_h_
#define IP_CONFIG_h_

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define IP_1.0

#define PIN_BUS_SENSOR_TEMP  5
#define BAUD_RATE_UART       9600
#define PIN_HOT_PWM          6

#define Kp_ 10    //Constante proporcional
#define Ki_ 0.1   //Constante integral
#define Kd_ 0     //Constante derivada

#define TEMP_INIC 36.5   //Esta temperatura debe de estar disponible en la configuración
#define OFFSET_CORRECTION 0.5  //Corrección de calibración única en cada máquina

bool sistemActive = false;

#endif

