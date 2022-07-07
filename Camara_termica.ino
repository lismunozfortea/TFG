
// VARIABLES GLOBALES //
//en la posicion 2 el valor actual, en la posicion 1 el valor un instante de muestreo atrás y en la 0 el valor dos instantes de muestreo atrás
// Vectores de entradas y salidas
float salidas_temp[2]; //vector de salidas de temperatura
float entradas_temp[2]; //vector de entradas de temperatura
float salidas_corriente[2]; //vector de salidas de corriente
float entradas_corriente[2]; //vector de entradas de corriente
// Variables auxiliares
float temp_seleccionada; //temperatura de consigna que es la que ha elegido el usuario
float corriente_deseada; //corriente de consigna para el regulador PID de corriente
float salida_temp_reg; //salida de temperatura después de la acción del regulador de temperatura
float salida_corriente_reg; //salida de corriente después de la acción del regulador de corriente
float lectura_tempC; //entre 0 y 4095
float lectura_tempF;
float lectura_corriente;
float valor_corriente;
float valor_tempF;
float valor_tempC;
bool leidos=false;
// Pines
const int pin_tempF = 32;
const int pin_tempC = 33;
const int pin_corriente = 34;
//Variables para las interrupciones
volatile int contador1;
volatile int contador2;
hw_timer_t * timer1 = NULL;
portMUX_TYPE timerMux1 = portMUX_INITIALIZER_UNLOCKED;
hw_timer_t * timer2 = NULL;
portMUX_TYPE timerMux2 = portMUX_INITIALIZER_UNLOCKED;
//Funciones ISR
void IRAM_ATTR onTimer1() { //ISR para el manejo de la interrupción de los PID's
  portENTER_CRITICAL_ISR(&timerMux1);
  contador1++;
  portEXIT_CRITICAL_ISR(&timerMux1);
 
}
void IRAM_ATTR onTimer2() { //ISR para el manejo de la interrupción de los sensores
  portENTER_CRITICAL_ISR(&timerMux2);
  contador2++;
  portEXIT_CRITICAL_ISR(&timerMux2);
 
}
//Funcion para calcular los reguladores PID
float PID(float u[2], float e[2], float consigna){
//Ec en diferencias del PID: u(k)=u(k−1)+q0e(k)+q1e(k-1)+q2e(k-2)   (siendo "u" la salida del lazo y "e" la entrada)
float q0,q1,q2;
q0=q1=q2=1;
 float e=consigna-e[2];
 float e_1=consigna-e[1];
 float e_2=consigna-e[0];
 
    // Control PID
      float u = u[1] + q0*e + q1*e_1 + q2*e_2; //Ley del controlador PID discreto
 
     return u;
   
}

//Funciones para la lectura/escritura de los valores de los sensores
void LecturaSensores(){ //lee de los pines ADC el valor de los sensores, estos pines tienen resolución de 12 bits, leen de 0 a 4095 donde 0 es 0V y 4095 3.3V
lectura_tempF= analogRead(pin_tempF)* (3.3 / 4095.0);
delay(1000);
lectura_tempC= analogRead(pin_tempC)* (3.3 / 4095.0);
delay(1000);
lectura_corriente= analogRead(pin_corriente)* (3.3 / 4095.0);
delay(1000);
//activamos la variable auxiliar "leidos" para avisar a la otra funcion de que ya puede escribirlos
leidos=true;
  }

void ValorSensores(){ //calcula el valor de los sensores en su magnitud correcta y los escribe
float sensibilidadT= 0.01; //sensibilidad en voltios/ºC, 1ºC equivale a 10mV en el sensor de temperatura LM335Z (dada por el fabricante)
float sensibilidadC=0.185; //sensibilidad en Voltios/Amperio para sensor de corriente ACS712 de 5A (dada por el fabricante)
if(leidos==true){
//Valor sensores de temperatura
valor_tempF= lectura_tempF/sensibilidadT;
valor_tempC= lectura_tempC/sensibilidadT;
//Valor sensor de corriente
valor_corriente= (lectura_corriente-2.5)/sensibilidadC; //formula desarrollada en la memoria
}

}
void setup() {
Serial.begin(115200);
//Inicializacion de los temporizadores
timer1 = timerBegin(0, 80, true); //la frecuencia base utilizada por los contadores en el ESP32 es de 80MHz
timer2 = timerBegin(0, 80, true);
//Manejo de los temporizadores
timerAttachInterrupt(timer1, &onTimer1, true);
timerAttachInterrupt(timer2, &onTimer2, true);
//Valor de los contadores
timerAlarmWrite(timer1, 1000000, true); //el segundo parámetro nos indica cada cuanto se generará la interrupción, en este caso cada un segundo
timerAlarmWrite(timer2, 1000000, true);
//Habilitación de los contadores
timerAlarmEnable(timer1);
timerAlarmEnable(timer2);

}

void loop() {

  //Interrupcion de los PID's
if (contador1>0) {
 portENTER_CRITICAL(&timerMux1);
    contador1--;
    
    portEXIT_CRITICAL(&timerMux1);

    //Código para el manejo de la interrupción

    salida_temp_reg= PID(salidas_temp,entradas_temp,temp_seleccionada);
    salida_corriente_reg= PID(salidas_corriente,entradas_corriente,corriente_deseada); //falta ver como se calcula la corriente deseada
    
}
 //Interrupcion de los sensores
if (contador2>0) {
 portENTER_CRITICAL(&timerMux2);
    contador2--;
    
    portEXIT_CRITICAL(&timerMux2);

    //Código para el manejo de la interrupción
LecturaSensores();
ValorSensores();
    
}
}
