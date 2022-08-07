// LIBRERIAS //
#include <SPI.h>
#include <FS.h>
// TFT_eSPI screen //
#define TFT_CS 15  //TFT CS to PIN 15
#include <TFT_eSPI.h>
TFT_eSPI tft = TFT_eSPI();
// TFT_eSPI touch //
//#define TOUCH_CS 21  //Touch CS to PIN 21
#define CALIBRATION_FILE "/TouchCalData2" // Calibration file stored in SPIFFS
#define REPEAT_CAL false // if true calibration is requested after reboot
#define totalButtonNumber 20

#define LABEL1_FONT &FreeSansOblique12pt7b // Key label font 1
#define LABEL2_FONT &FreeSansBold12pt7b    // Key label font 2
TFT_eSPI_Button keyN[totalButtonNumber];  // TFT_eSPI button class

//Funcion conversion de colores //
int ConvertRGBto565(byte rr, byte gg, byte bb)
{
  //reduz para 5 bits significativos
  byte r = (byte) (rr >> 3);
  //reduz para 6 bits significativos
  byte g = (byte)(gg >> 2);
  //reduz para 5 bits significativos
  byte b = (byte)(bb >> 3);

  //Junta
  return (int)((r << 11) | (g << 5) | b);
}
// VARIABLES GLOBALES //
//en la posicion 2 el valor actual, en la posicion 1 el valor un instante de muestreo atrás y en la 0 el valor dos instantes de muestreo atrás
// Vectores de entradas y salidas
float salidas_temp[2]; //vector de salidas de temperatura
float entradas_temp[2]; //vector de entradas de temperatura
float salidas_corriente[2]; //vector de salidas de corriente
float entradas_corriente[2]; //vector de entradas de corriente en valores analógicos
// Variables auxiliares
float temp_seleccionada; //temperatura de consigna que es la que ha elegido el usuario
float corriente_deseada; //corriente de consigna para el regulador PID de corriente
float salida_temp_reg; //salida de temperatura después de la acción del regulador de temperatura
float salida_corriente_reg; //salida de corriente después de la acción del regulador de corriente
float lectura_tempC; //entre 0 y 4095
float lectura_tempF;
float lectura_corriente;
float seleccion[1];
int start;
int defcolor = ConvertRGBto565(131, 131, 131);
//float valor_corriente;
//float valor_tempF;
float sensibilidadT= 0.01; //sensibilidad en voltios/ºC, 1ºC equivale a 10mV en el sensor de temperatura LM335Z (dada por el fabricante)
float sensibilidadC=0.185; //sensibilidad en Voltios/Amperio para sensor de corriente ACS712 de 5A (dada por el fabricante)
float valor_tempC;
float ciclo_trabajo; //error de la corriente que pasamos a través de la salida PWM
float q_temp[2];
float q_corriente[2];
bool leidos=false;
// Pines
const int pin_tempF = 32;
const int pin_tempC = 33;
const int pin_corriente = 34;
int IN3 = 13;    // Input3 conectada al pin 13
int IN4 = 14;    // Input4 conectada al pin 14
int ENB = 12;    // ENB conectada al pin 12, PWM
//Variables para las interrupciones
volatile int contador;
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
//Funciones ISR
void IRAM_ATTR onTimer() { //ISR para el manejo de la interrupción de los PID's
  portENTER_CRITICAL_ISR(&timerMux);
  contador++;
  portEXIT_CRITICAL_ISR(&timerMux);
 
}

//Funcion para calcular los reguladores PID
float PID(float u[2], float e[2], float consigna, float q[2]){
//Ec en diferencias del PID: u(k)=u(k−1)+q0e(k)+q1e(k-1)+q2e(k-2)   (siendo "u" la salida del lazo y "e" la entrada)
 float e_0=consigna-e[2];
 float e_1=consigna-e[1];
 float e_2=consigna-e[0];
 
    // Control PID
      float u_s = u[1] + q[0]*e_0 + q[1]*e_1 + q[2]*e_2; //Ley del controlador PID discreto
 
     return u_s;
   
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
}
}
//Funcion para el control del puente H
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
  pwm=abs(pwm); //nos quedamos con el valor absoluto
  // Aplicamos PWM al pin ENB, modificando el ciclo de trabajo en funcion de la temperatura deseada
  analogWrite(ENB,pwm);
}

  //Funcion control del teclado
 void Teclado()
  {
      uint16_t t_x = 0, t_y = 0; // coordenadas pulsacion
      bool pressed = tft.getTouch(&t_x, &t_y);  // true al pulsar

  // Comprueba si pulsas en zona de botón
  for (uint8_t b = 0; b < totalButtonNumber; b++) {
    if (pressed && keyN[b].contains(t_x, t_y)) {
      keyN[b].press(true);
      Serial.print(t_x);
      Serial.print(",");
      Serial.println(t_y);
    } else {
      keyN[b].press(false);
    }
  }

  // Accion si se pulsa boton
  for (uint8_t b = 0; b < totalButtonNumber; b++) {

    if (keyN[b].justReleased()) {
    keyN[b].drawButton(); // redibuja al soltar

      switch (b) {
        case 0: 
//Gestión de las pulsaciones del teclado numérico
        if(seleccion[0]!=0)
        {
          tft.setCursor(60,30);
          tft.print("0");
          seleccion[0]=0; }
          else
         { tft.setCursor(70,30);
          tft.print("0");
          seleccion[1]=0;
          }        
          break;
          
        case 1:
       
          if(seleccion[0]!=1)
        {
          tft.setCursor(60,30);
          tft.print("1");
          seleccion[0]=1;
            } 
          else
         { tft.setCursor(70,30);
          tft.print("1");
          seleccion[1]=1;
          }
          break;
          
           case 2: 
             if(seleccion[0]!=2)
        {
          tft.setCursor(60,30);
          tft.print("2");
          seleccion[0]=2;   } 
          else
         { tft.setCursor(70,30);
          tft.print("2");
          seleccion[1]=2;
          }
         break;

         case 3: 
          if(seleccion[0]!=3)
        {
          tft.setCursor(60,30);
          tft.print("3");
          seleccion[0]=3;  } 
          else
          {tft.setCursor(70,30);
          tft.print("3");
          seleccion[1]=3;
          }
         break;
         
         case 4:
       
          if(seleccion[0]!=4)
        {
          tft.setCursor(60,30);
          tft.print("4");
          seleccion[0]=4;   } 
          else
        {  tft.setCursor(70,30);
          tft.print("4");
          seleccion[1]=4;
          }
          break;
          
           case 5: 
         if(seleccion[0]!=5)
        {
          tft.setCursor(60,30);
          tft.print("5");
          seleccion[0]=5;   } 
          else
        {  tft.setCursor(70,30);
          tft.print("5");
          seleccion[1]=5;
          }
         break;

         case 6: 
          if(seleccion[0]!=6)
        {
          tft.setCursor(60,30);
          tft.print("6");
          seleccion[0]=6;  } 
          else
          {tft.setCursor(70,30);
          tft.print("6");
          seleccion[1]=6;
          }
         break;
         
         case 7:
         if(seleccion[0]!=7)
        {
          tft.setCursor(60,30);
          tft.print("7");
          seleccion[0]=7;  } 
          else
         { tft.setCursor(70,30);
          tft.print("7");
          seleccion[1]=7;
          }
          break;
          
           case 8: 
          if(seleccion[0]!=8)
        {
          tft.setCursor(60,30);
          tft.print("8");
          seleccion[0]=8;  } 
          else
         { tft.setCursor(70,30);
          tft.print("8");
          seleccion[1]=8;
          }
         break;

         case 9: 
       if(seleccion[0]!=9)
        {
          tft.setCursor(60,30);
          tft.print("9");
          seleccion[0]=9;  } 
          else
         { tft.setCursor(70,30);
          tft.print("9");
          seleccion[1]=9;
          }
         break;

          case 10: //botón de start
         { // se convierte el vector en un float
        temp_seleccionada= seleccion[0]*10 + seleccion[1];
      start=1;
         }
         break;

         case 11: //boton de off
         
       { ciclo_trabajo=0;
        digitalWrite (IN4, LOW);
        digitalWrite (IN3, LOW);
         }
        break;
        
        default:
          delay(1);
          
      }
    }
         
    if (keyN[b].justPressed()) {
      keyN[b].drawButton(true);  // cambia color del botón al pulsar
      delay(10); // evitar rebotes de pulsacion
    }
  } 
    }  
  

//Funcion para dibujar el teclado numérico 
void DibujarBotones(){
  int defcolor = ConvertRGBto565(131, 131, 131);
  tft.setRotation(1);
  tft.fillScreen(defcolor);
  tft.setTextColor(ILI9341_BLACK);
  tft.setCursor(60,20);
  tft.print("Introduzca la temperatura:");
  // Draw the keys
  tft.setFreeFont(LABEL1_FONT);
 
  keyN[0].initButton(&tft, 40, 40, 40, 40, TFT_BLACK, TFT_WHITE, TFT_BLUE, "0" , 1 ); // x, y, w, h, outline, fill, color, label, text_Size
  keyN[0].drawButton();
  keyN[1].initButton(&tft, 80, 40, 40, 40, TFT_BLACK, TFT_WHITE, TFT_BLUE, "1" , 1 );
  keyN[1].drawButton();
  keyN[2].initButton(&tft, 120, 40, 40, 40, TFT_BLACK, TFT_WHITE, TFT_BLUE, "2" , 1 ); // x, y, w, h, outline, fill, color, label, text_Size
  keyN[2].drawButton();
  keyN[3].initButton(&tft, 160, 80, 40, 40, TFT_BLACK, TFT_WHITE, TFT_BLUE, "3" , 1 );
  keyN[3].drawButton();
  keyN[4].initButton(&tft,40, 80, 40, 40, TFT_BLACK, TFT_WHITE, TFT_BLUE, "4" , 1 ); // x, y, w, h, outline, fill, color, label, text_Size
  keyN[4].drawButton();
  keyN[5].initButton(&tft, 80, 80, 40, 40, TFT_BLACK, TFT_WHITE, TFT_BLUE, "5" , 1 );
  keyN[5].drawButton();
  keyN[6].initButton(&tft, 120, 80, 40, 40, TFT_BLACK, TFT_WHITE, TFT_BLUE, "6" , 1 ); // x, y, w, h, outline, fill, color, label, text_Size
  keyN[6].drawButton();
  keyN[7].initButton(&tft, 160, 120, 40, 40, TFT_BLACK, TFT_WHITE, TFT_BLUE, "7" , 1 );
  keyN[7].drawButton();
  keyN[8].initButton(&tft, 180, 120, 40, 40, TFT_BLACK, TFT_WHITE, TFT_BLUE, "8" , 1 );
  keyN[8].drawButton();
  keyN[9].initButton(&tft, 40, 120, 40, 40, TFT_BLACK, TFT_WHITE, TFT_BLUE, "9" , 1 );
  keyN[9].drawButton();
  keyN[10].initButton(&tft, 20, 180, 60, 60, TFT_BLACK, TFT_WHITE, TFT_BLUE, "Start" , 1 );
  keyN[10].drawButton();
  keyN[11].initButton(&tft, 20, 180, 60, 60, TFT_BLACK, TFT_WHITE, TFT_BLUE, "Off" , 1 );
  keyN[11].drawButton();

}
void setup() {
  Serial.begin(115200);
//Setup pantalla TFT
  digitalWrite(TOUCH_CS, HIGH); // ********** TFT_eSPI touch **********
  digitalWrite(TFT_CS, HIGH); // ********** TFT_eSPI screen library **********
  delay(2000);  
 tft.begin();
 tft.invertDisplay(false); // I don't know why but it is required for my TFT to color correction
 tft.setRotation(1);  // Landscape
 DibujarBotones();
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

void loop() {
  //INTERFAZ DE USUARIO //
  // Función que controla las pulsaciones 
    Teclado();
 //Accion si se pulsa el boton de start
if(start==1)
{
 
        tft.fillScreen(defcolor);
         tft.setTextColor(ILI9341_BLACK);
          tft.setCursor(60,30);
          tft.print("Temperatura:");
        tft.setCursor(60,100);
        tft.print(entradas_temp[2]);
         tft.print("\337C");

         //funcion para que aparezca un aviso cuando la temperatura se estabilice
         if(entradas_temp[2]==temp_seleccionada) //mas bien seria cuando dejara de variar pero provisionalmente pongo esto
         {
           tft.setCursor(60,120);
          tft.print("Temperatura estable, apunte los resultados");
          delay(10000);
         start=0;
         DibujarBotones();
                 }       
    }
  //Interrupcion
if (contador>0) {
 portENTER_CRITICAL(&timerMux);
    contador--;
    portEXIT_CRITICAL(&timerMux);

    //Código que se ejecuta durante la interrupción
salidas_temp[0]=salidas_temp[1];
salidas_temp[1]=salidas_temp[2];
salidas_temp[2]= PID(salidas_temp,entradas_temp,temp_seleccionada,q_temp)*sensibilidadT; //obtenemos la salidad el PID de temperatura en valor digital
salidas_temp[2]=(salidas_temp[2]-2.5)/sensibilidadC; //lo pasamos a valores de corriente para pasarselo como entrada al PID de corriente
salidas_corriente[0]=salidas_corriente[1];
salidas_corriente[1]=salidas_corriente[2];
salidas_corriente[2]= PID(salidas_corriente,salidas_temp,corriente_deseada,q_corriente); //falta ver como se calcula la corriente deseada
LecturaSensores();
ValorSensores();
ciclo_trabajo=corriente_deseada-entradas_corriente[2]; //le pasamos el error de la corriente como ciclo de trabajo 
ControlPuenteH(ciclo_trabajo);
    
}

}
