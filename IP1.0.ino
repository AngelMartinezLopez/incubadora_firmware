#include <Wire.h>
#include <TimerOne.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Nextion.h> 

#include "IP_config/IP_config.h"

volatile float Tinicial = TEMP_INIC;     //Temperatura de regulación inicial
volatile float Offset = OFFSET_CORRECTION;        //Offset de regulación de temperatura, selección por HMI
volatile float Temperatura;         //
volatile float Tpro = 0;
float *Tpwm = 0;

volatile float error = 0;
volatile float error_ant;
float *Tp = 0;
float *Ti = 0;
float *Td = 0;
volatile float Kp = Kp_;
volatile float Ki = Ki_;
volatile float Kd = Kd_;

NexButton Start = NexButton(0, 1, "Start");
NexButton Stop  = NexButton(1,2, "Stop");

NexTouch *nex_listen_list[] =
{
&Start,
&Stop,
NULL
};

OneWire oneWireBus(PIN_BUS_SENSOR_TEMP);
DallasTemperature sensor(&oneWireBus);

void Inicia_sistema(void *ptr){
  sistemActive = true;
}

void Detiene_sistema(void *ptr){
  sistemActive = false;
}

void setup() {

  Serial.begin(BAUD_RATE_UART);
  pinMode(PIN_HOT_PWM, OUTPUT);
   
  delay(500);
  
  sensor.requestTemperatures();
 
  Temperatura = sensor.getTempCByIndex(0);

  delay(4000);
  
  Timer1.initialize(5000);                       //La interrupción se realizará cada 5 milisegundos
  Timer1.attachInterrupt(Control_Temperatura);   //Cada 5 milisegundos se ejecutará la función

  Tpwm = 0;     //Variable regulada por el PID que determinará el tiempo en ON del calefactor cada 500 ms

nexInit();
Start.attachPop(Inicia_sistema,&Start);
}

void loop() {
  sensor.requestTemperatures();
  Temperatura = sensor.getTempCByIndex(0);
}

void Control_Temperatura() { 
  if (sistemActive){ 
    if (Tpro >= *Tpwm) digitalWrite(6, LOW);   
    if (Tpro <= *Tpwm) digitalWrite(6, HIGH);  

    //Cada 100 ciclos de la interrución lo presentaremos en el monitor serie y nuevamente el PID realizará los cálculos
    if (Tpro == 100) {
      Tpro = 0;
      pid(Tp, Ti, Td, Tpwm);
    }
  }
}

/************************************************************************************************
 * Este es un algorítmo PID básico, suficiente para una regulación de temperatura
 * la sintonía es sencilla gracias a la lenta velocidad del cambio de temperatura
 ***********************************************************************************************/

void pid(float *Tp, float *Ti, float *Td, float *Tpwm) {

//Parte proporcional del algoritmo
  error  = (Tinicial + Offset) - (Temperatura);
  *Tp = (error * Kp);

//Parte integral del algoritmo
  *Ti = (*Ti + (error * Ki));
  if (*Ti > 100) *Ti = 100;
  if (*Ti < 0) *Ti = 0;

//Parte derivada del algoritmo
  *Td = ((error - error_ant) * Kd);
  error_ant = error;

//Suma de cada una de las partes del algoritmo PID
  *Tpwm = *Tp + *Ti + *Td;
}
