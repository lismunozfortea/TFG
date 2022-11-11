#include "analogWrite.h"

//*********VARIABLES GLOBALES********//
// Vectores de entradas y salidas //
//en la posicion 2 el valor actual, en la posicion 1 el valor un instante de muestreo atrás y en la 0 el valor dos instantes de muestreo atrás
float salidas_temp[2]; //vector de salidas de temperatura
float entradas_temp[2]; //vector de entradas de temperatura
float salidas_corriente[2]; //vector de salidas de corriente
float entradas_corriente[2]; //vector de entradas de corriente en valores analógicos
// Variables auxiliares //
float temp_seleccionada; //temperatura de consigna que es la que ha elegido el usuario
float corriente_deseada; //corriente de consigna para el regulador PID de corriente
float lectura_tempC; //entre 0 y 4095
float lectura_tempF;
float lectura_corriente;
float sensibilidadT= 0.01; //sensibilidad en voltios/ºC, 1ºC equivale a 10mV en el sensor de temperatura LM335Z (dada por el fabricante)
float sensibilidadC=0.185; //sensibilidad en Voltios/Amperio para sensor de corriente ACS712 de 5A (dada por el fabricante)
float valor_tempC;
float ciclo_trabajo; //error de la corriente que pasamos a través de la salida PWM
float q_temp[]= {9.12*pow(10,-26),1.87*pow(10,-27),8.93*pow(10,-26)}; //constantes PID temperatura
float q_corriente[]= {1.42,0.945,0.157}; //constantes PID corriente
bool leidos=false;
// Pines //
const int pin_tempF = 32;
const int pin_tempC = 33;
const int pin_corriente = 34;
int IN3 = 13;    // Input3 conectada al pin 13
int IN4 = 14;    // Input4 conectada al pin 14
int ENB = 12;    // ENB conectada al pin 12, PWM
//Variables para las interrupciones //
volatile int contador;
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
//------------------------------------------------------------------------------------------
//*********FUNCIONES GENERALES********//
//Funciones ISR //
void IRAM_ATTR onTimer() { //ISR para el manejo de la interrupción de los PID's
  portENTER_CRITICAL_ISR(&timerMux);
  contador++;
  portEXIT_CRITICAL_ISR(&timerMux);
 
}

void setup() {
   //*********SETUP GENERAL********//
Serial.begin(115200);
  //Setup pines puente H 
 pinMode (ENB, OUTPUT); 
 pinMode (IN3, OUTPUT);
 pinMode (IN4, OUTPUT);
//Inicializacion de los valores de las variables y de los pines
ciclo_trabajo=0; //Inicialmente apagada
digitalWrite(IN3,LOW);
digitalWrite(IN4,LOW);
//Inicializacion de los temporizadores
timer = timerBegin(0, 80, true); //la frecuencia base utilizada por los contadores en el ESP32 es de 80MHz
//Manejo de los temporizadores
timerAttachInterrupt(timer, &onTimer, true);
//Valor de los contadores
timerAlarmWrite(timer, 1000000, true); //el segundo parámetro nos indica cada cuanto se generará la interrupción, en este caso cada un segundo
//Habilitación de los contadores
timerAlarmEnable(timer);
}

//------------------------------------------------------------------------------------------

void loop(void) {
   //*********MANEJO DE INTERRUPCIONES********//
if (contador>0) {
 portENTER_CRITICAL(&timerMux);
    contador--;
    portEXIT_CRITICAL(&timerMux);

 //Código que se ejecuta durante la interrupción
salidas_temp[0]=salidas_temp[1];
salidas_temp[1]=salidas_temp[2];
salidas_temp[2]= PID(salidas_temp,entradas_temp,temp_seleccionada,q_temp); //obtenemos la salidad el PID de temperatura en valor digital
corriente_deseada=(salidas_temp[2]*sensibilidadT-2.5)/sensibilidadC; //lo pasamos a valores de corriente para pasarselo como consigna al PID de corriente
salidas_corriente[0]=salidas_corriente[1];
salidas_corriente[1]=salidas_corriente[2];
salidas_corriente[2]= PID(salidas_corriente,entradas_corriente,corriente_deseada,q_corriente); 
LecturaSensores();
//delay(1000);
ValorSensores();
//delay(1000);
ciclo_trabajo=entradas_corriente[2]-corriente_deseada; //le pasamos el error de la corriente como ciclo de trabajo 
ControlPuenteH(ciclo_trabajo);
    

}
}

//Funcion para calcular los reguladores PID //
float PID(float u[2], float e[2], float consigna, float q[2]){
//Ec en diferencias del PID: u(k)=u(k−1)+q0e(k)+q1e(k-1)+q2e(k-2)   (siendo "u" la salida del lazo y "e" la entrada)
 float e_0=consigna-e[2];
 float e_1=consigna-e[1];
 float e_2=consigna-e[0];
 
    // Control PID
      float u_s = u[1] + q[0]*e_0 + q[1]*e_1 + q[2]*e_2; //Ley del controlador PID discreto
 
     return u_s;
   
}

//Funciones para la lectura/escritura de los valores de los sensores //
void LecturaSensores(){ //lee de los pines ADC el valor de los sensores, estos pines tienen resolución de 12 bits, leen de 0 a 4095 donde 0 es 0V y 4095 3.3V
lectura_tempF= analogRead(pin_tempF)* (3.3 / 4095.0);
lectura_tempC= analogRead(pin_tempC)* (3.3 / 4095.0);
lectura_corriente= analogRead(pin_corriente)* (3.3 / 4095.0);
//activamos la variable auxiliar "leidos" para avisar a la otra funcion de que ya puede escribirlos
leidos=true;
  }

void ValorSensores(){ //calcula el valor de los sensores en su magnitud correcta y los escribe
if(leidos==true){
//Valor sensores de temperatura 
entradas_temp[0]=entradas_temp[1];
entradas_temp[1]=entradas_temp[2];
entradas_temp[2]= lectura_tempF/sensibilidadT;
valor_tempC= lectura_tempC/sensibilidadT;
//Valor sensor de corriente
entradas_corriente[0]=entradas_corriente[1];
entradas_corriente[1]=entradas_corriente[2];
entradas_corriente[2]= (lectura_corriente-2.5)/sensibilidadC; //formula desarrollada en la memoria
leidos=false;
}
}
//Funcion para el control del puente H //
void ControlPuenteH(float pwm){
  //Si la corriente de entrada es positiva se activa una diagonal y si es negativa, la otra
  if(pwm>0){
  digitalWrite (IN4, HIGH);
  digitalWrite (IN3, LOW);
  }
  else {
  digitalWrite (IN3, HIGH);
  digitalWrite (IN4, LOW);
  }
//Lo ideal sería en funcion de la señal de error (ciclo de trabajo) saber cuanto variar el ancho de PWM

  pwm=(pwm*sensibilidadC) + 2.5; //pasamos de valor de corriente a digital
  // Aplicamos PWM al pin ENB, modificando el ciclo de trabajo en funcion de la temperatura deseada
  analogWrite(ENB,pwm);
}
