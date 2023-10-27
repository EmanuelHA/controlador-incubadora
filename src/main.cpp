/************************************
* AUTOR: EMANUEL HERNANDEZ ARAUZ    *
* FECHA: 14 JULIO, 2023             *
*************************************/

#include <Arduino.h>
#include <PID_v1.h>
#include <TinyI2CMaster.h>
#define  TINY4KOLED_QUICK_BEGIN // Reduccion de espacio al ser una OLED blanca
#include <Tiny4kOLED.h>
#include "ModernDos8.h"
#include <AHT10.h>
#define  SSR             0      // Pin Solid State Rele (SSR)
#define  ZERO_CROSS      2      // Pin paso por cero (ATMega328 -> 2, 3 pines interrupcion)
#define  AC_SEMICYCLE_uS 8333   // Duracion (en microsegundos) de un semiciclo de AC a 60Hz
#define  SAMPLING_RATE   2000   // Delay (en milisegundos) para evitar sobrecalentamiento al leer el sensor AHT10

/*   VARIABLES  */
// CRUCE POR CERO
volatile bool zcDetected = false;   // Bandera para indicar la deteccion del paso por cero
// INCUBACION
const unsigned short incubationPeriod[] = { 21, 28 };   // Config. de los dias max. de incubacion
enum Poultry { GALLINA, PATO, GANSO = 1 };              // Aves de corral disponibles en la config.
// PID
double setpoint = 37.5;                                             // Temperatura objetivo en grados Celsius
unsigned long timerNow = 0, timerPrev = 0;                          // Medicion del tiempo en ms
double input, output, kp = 2, ki = 5, kd = 1;                       // Variables de PID
PID PIDController(&input, &output, &setpoint, kp, ki, kd, DIRECT);  // Controlador PID
// E/S
AHT10 aht10Sensor = AHT10();    // Sensor de temperatura/humedad AHT10

// Funciones
void zeroCrossing();
void updateDisplay();
unsigned int customMap(unsigned int value, unsigned int from, unsigned int to);

void setup() {
    pinMode(SSR, OUTPUT);
    pinMode(ZERO_CROSS, INPUT);
    attachInterrupt(digitalPinToInterrupt(ZERO_CROSS), zeroCrossing, RISING);

    aht10Sensor.begin();
    input = aht10Sensor.readTemperature();
    digitalWrite(SSR, LOW);
    PIDController.SetSampleTime(SAMPLING_RATE);
    PIDController.SetMode(AUTOMATIC);
    oled.begin();
    oled.setFont(FONT8X8MDOS);
    oled.clear();
    oled.on();
    // oled.switchRenderFrame();
}

void loop() {
    timerNow = millis();

    if ((timerNow - timerPrev) >= SAMPLING_RATE) {
        input = aht10Sensor.readTemperature();
        PIDController.Compute();
        timerPrev = timerNow;
        updateDisplay();
    }
    // Implementacion si no se cuenta con puente rectificador en el optoacoplador
    if (zcDetected) {
        unsigned int ssrTriggerLapse = customMap(output, 255, AC_SEMICYCLE_uS);
        digitalWrite(SSR, HIGH);
        delayMicroseconds(ssrTriggerLapse);
        digitalWrite(SSR, LOW);
        delayMicroseconds(AC_SEMICYCLE_uS - ssrTriggerLapse);
        digitalWrite(SSR, HIGH);
        delayMicroseconds(ssrTriggerLapse);
        digitalWrite(SSR, LOW);
        zcDetected = false;
    }

    // Implementacion en caso de contar con puente rectificador en el optoacoplador
    //if (zcDetected) {
    //    unsigned int ssrTriggerLapse = customMap(output, 255, AC_SEMICYCLE_uS);
    //    digitalWrite(SSR, HIGH);
    //    delayMicroseconds(ssrTriggerLapse);
    //    digitalWrite(SSR, LOW);
    //    zcDetected = false;
    //}
}

// Funcion de interrupcion al detectar el paso por cero
void zeroCrossing() {
    zcDetected = true;
}
// Actualizacion de la OLED
void updateDisplay() {
    oled.clear();
    oled.setCursor(0, 0);
    oled.printf("TEMPERATURA: %f\n", input);
    // oled.switchFrame();
}
// Custom mapping con 0 como valor minimo (no funciona con datos negativos)
unsigned int customMap(unsigned int value, unsigned int from, unsigned int to) {
    return (value / from) * to;
}
