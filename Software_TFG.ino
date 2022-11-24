#include <analogWrite.h>


/*
  The TFT_eSPI library incorporates an Adafruit_GFX compatible
  button handling class, this sketch is based on the Arduin-o-phone
  example.
  This example diplays a keypad where numbers can be entered and
  send to the Serial Monitor window.
  The sketch has been tested on the ESP8266 (which supports SPIFFS)
  The minimum screen size is 320 x 240 as that is the keypad size.
  TOUCH_CS and SPI_TOUCH_FREQUENCY must be defined in the User_Setup.h file
  for the touch functions to do anything.
*/

// The SPIFFS (FLASH filing system) is used to hold touch screen
// calibration data

#include "FS.h"

#include <SPI.h>
#include <TFT_eSPI.h>      // Hardware-specific library

TFT_eSPI tft = TFT_eSPI(); // Invoke custom library

// This is the file name used to store the calibration data
// You can change this to create new calibration files.
// The SPIFFS file name must start with "/".
#define CALIBRATION_FILE "/TouchCalData2"

// Set REPEAT_CAL to true instead of false to run calibration
// again, otherwise it will only be done once.
// Repeat calibration if you change the screen rotation.
#define REPEAT_CAL false

// Keypad start position, key sizes and spacing
#define KEY_X 40 // Centre of key
#define KEY_Y 96
#define KEY_W 62 // Width and height
#define KEY_H 30
#define KEY_SPACING_X 18 // X and Y gap
#define KEY_SPACING_Y 20
#define KEY_TEXTSIZE 1   // Font size multiplier

// Using two fonts since numbers are nice when bold
#define LABEL1_FONT &FreeSansOblique12pt7b // Key label font 1
#define LABEL2_FONT &FreeSansBold12pt7b    // Key label font 2

// Numeric display box size and location
#define DISP_X 1
#define DISP_Y 10
#define DISP_W 238
#define DISP_H 50
#define DISP_TSIZE 3
#define DISP_TCOLOR TFT_CYAN

// Number length, buffer for storing it and character index
#define NUM_LEN 12
char numberBuffer[NUM_LEN + 1] = "";
uint8_t numberIndex = 0;

// We have a status line for messages
#define STATUS_X 120 // Centred on this
#define STATUS_Y 65

// Cuadro donde aparece la temperatura lado frio
#define TEMP_X 250 // Centred on this
#define TEMP_Y 65

// Create 15 keys for the keypad
char keyLabel[15][5] = {"New", "Del", "Send", "1", "2", "3", "4", "5", "6", "7", "8", "9", ".", "0", "#" };
uint16_t keyColor[15] = {TFT_RED, TFT_DARKGREY, TFT_DARKGREEN,
                         TFT_BLUE, TFT_BLUE, TFT_BLUE,
                         TFT_BLUE, TFT_BLUE, TFT_BLUE,
                         TFT_BLUE, TFT_BLUE, TFT_BLUE,
                         TFT_BLUE, TFT_BLUE, TFT_BLUE
                        };

// Invoke the TFT_eSPI button class and create all the button objects
TFT_eSPI_Button key[15];
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
float sensibilidadT= 0.1; //sensibilidad en voltios/ºC, 1ºC equivale a 10mV en el sensor de temperatura LM335Z (dada por el fabricante)
float sensibilidadC=1.85; //sensibilidad en Voltios/Amperio para sensor de corriente ACS712 de 5A (dada por el fabricante)
float valor_tempC;
float ciclo_trabajo; //error de la corriente que pasamos a través de la salida PWM
float q_temp[]= {9.12*pow(10,-26),1.87*pow(10,-27),8.93*pow(10,-26)}; //constantes PID temperatura
float q_corriente[]= {1.42,0.945,0.157}; //constantes PID corriente
bool leidos=false;
// Pines //
#define pin_tempF 32
#define pin_tempC 33
#define pin_corriente 34
#define IN1 13  // Input1 conectada al pin 13
#define IN2 14    // Input2 conectada al pin 14
#define ENA 15    // ENA conectada al pin 15, PWM
//Variables para las interrupciones //
volatile int contador;
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

//Funciones ISR //
void IRAM_ATTR onTimer() { //ISR para el manejo de la interrupción de los PID's
  portENTER_CRITICAL_ISR(&timerMux);
  contador++;
  portEXIT_CRITICAL_ISR(&timerMux);
 
}
//------------------------------------------------------------------------------------------

void setup() {
  // Use serial port
  Serial.begin(115200);

  // Initialise the TFT screen
  tft.init();

  // Set the rotation before we calibrate
  tft.setRotation(3);

  // Calibrate the touch screen and retrieve the scaling factors
  //touch_calibrate();
  uint16_t calData[5] = {299, 3588, 348, 3474, 1};
  tft.setTouch(calData);

  // Clear the screen
  tft.fillScreen(TFT_BLACK);

  // Draw keypad background
  tft.fillRect(0, 0, 240, 320, TFT_DARKGREY);

  // Draw number display area and frame
  tft.fillRect(DISP_X, DISP_Y, DISP_W, DISP_H, TFT_BLACK);
  tft.drawRect(DISP_X, DISP_Y, DISP_W, DISP_H, TFT_WHITE);

  // Draw keypad
  drawKeypad();
  //Status inicial
  status("Introduzca temperatura"); 
   //*********SETUP GENERAL********//

  //Setup pines puente H 
 pinMode (ENA, OUTPUT); 
 pinMode (IN1, OUTPUT);
 pinMode (IN2, OUTPUT);
//Inicializacion de los valores de las variables y de los pines
ciclo_trabajo=0; //Inicialmente apagada
digitalWrite(IN1,LOW);
digitalWrite(IN2,LOW);
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
 

  uint16_t t_x = 0, t_y = 0; // To store the touch coordinates

  // Pressed will be set true is there is a valid touch on the screen
  boolean pressed = tft.getTouch(&t_x, &t_y);

 
  // / Check if any key coordinate boxes contain the touch coordinates
  for (uint8_t b = 0; b < 15; b++) {
    if (pressed && key[b].contains(t_x, t_y)) {
      key[b].press(true);  // tell the button it is pressed
    } else {
      key[b].press(false);  // tell the button it is NOT pressed
    }
  }

  // Check if any key has changed state
  for (uint8_t b = 0; b < 15; b++) {

    if (b < 3) tft.setFreeFont(LABEL1_FONT);
    else tft.setFreeFont(LABEL2_FONT);

    if (key[b].justReleased()) key[b].drawButton();     // draw normal

    if (key[b].justPressed()) {
      key[b].drawButton(true);  // draw invert

      // if a numberpad button, append the relevant # to the numberBuffer
      if (b >= 3) {
        if (numberIndex < NUM_LEN) {
          numberBuffer[numberIndex] = keyLabel[b][0];
          numberIndex++;
          numberBuffer[numberIndex] = 0; // zero terminate
        }
       status("Introduzca temperatura"); // Clear the old status
      }

      // Del button, so delete last char
      if (b == 1) {
        numberBuffer[numberIndex] = 0;
        if (numberIndex > 0) {
          numberIndex--;
          numberBuffer[numberIndex] = 0;//' ';
        }
        status("Introduzca temperatura"); // Clear the old status
      }

      if (b == 2) {
        status("Temperatura enviada, espere");
        Serial.println(numberBuffer);
       temp_seleccionada= atof(numberBuffer);

         //funcion para que aparezca un aviso cuando la temperatura se estabilice
         if(entradas_temp[2]==temp_seleccionada) //mas bien seria cuando dejara de variar pero provisionalmente pongo esto
         {
          status("Temperatura estable, apunte los resultados");
          delay(10000);
                 }    
      }
      // we dont really check that the text field makes sense
      // just try to call
      if (b == 0) {
        status("Introduzca temperatura");
        numberIndex = 0; // Reset index to 0
        numberBuffer[numberIndex] = 0; // Place null in buffer
      }

      // Update the number display field
      tft.setTextDatum(TL_DATUM);        // Use top left corner as text coord datum
      tft.setFreeFont(&FreeSans18pt7b);  // Choose a nicefont that fits box
      tft.setTextColor(DISP_TCOLOR);     // Set the font colour

      // Draw the string, the value returned is the width in pixels
      int xwidth = tft.drawString(numberBuffer, DISP_X + 4, DISP_Y + 12);

      // Now cover up the rest of the line up by drawing a black rectangle.  No flicker this way
      // but it will not work with italic or oblique fonts due to character overlap.
      tft.fillRect(DISP_X + 4 + xwidth, DISP_Y + 1, DISP_W - xwidth - 5, DISP_H - 2, TFT_BLACK);

      delay(10); // UI debouncing
    }
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
ValorSensores();
ciclo_trabajo=entradas_corriente[2]-corriente_deseada; //le pasamos el error de la corriente como ciclo de trabajo 
ControlPuenteH(ciclo_trabajo);
//Visualizacion temperatura
 PrintTemp();

}
  }
 }

//*********FUNCIONES PARA LA INTERFAZ********//
//------------------------------------------------------------------------------------------

void drawKeypad()
{
  // Draw the keys
  for (uint8_t row = 0; row < 5; row++) {
    for (uint8_t col = 0; col < 3; col++) {
      uint8_t b = col + row * 3;

      if (b < 3) tft.setFreeFont(LABEL1_FONT);
      else tft.setFreeFont(LABEL2_FONT);

      key[b].initButton(&tft, KEY_X + col * (KEY_W + KEY_SPACING_X),
                        KEY_Y + row * (KEY_H + KEY_SPACING_Y), // x, y, w, h, outline, fill, text
                        KEY_W, KEY_H, TFT_WHITE, keyColor[b], TFT_WHITE,
                        keyLabel[b], KEY_TEXTSIZE);
      key[b].drawButton();
    }
  }
}

//------------------------------------------------------------------------------------------

void touch_calibrate()
{
  uint16_t calData[5];
  uint8_t calDataOK = 0;

  // check file system exists
  if (!SPIFFS.begin()) {
    Serial.println("Formating file system");
    SPIFFS.format();
    SPIFFS.begin();
  }

  // check if calibration file exists and size is correct
  if (SPIFFS.exists(CALIBRATION_FILE)) {
    if (REPEAT_CAL)
    {
      // Delete if we want to re-calibrate
      SPIFFS.remove(CALIBRATION_FILE);
    }
    else
    {
      File f = SPIFFS.open(CALIBRATION_FILE, "r");
      if (f) {
        if (f.readBytes((char *)calData, 14) == 14)
          calDataOK = 1;
        f.close();
      }
    }
  }

  if (calDataOK && !REPEAT_CAL) {
    // calibration data valid
    tft.setTouch(calData);
  } else {
    // data not valid so recalibrate
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(20, 0);
    tft.setTextFont(2);
    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);

    tft.println("Touch corners as indicated");

    tft.setTextFont(1);
    tft.println();

    if (REPEAT_CAL) {
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.println("Set REPEAT_CAL to false to stop this running again!");
    }

    tft.calibrateTouch(calData, TFT_MAGENTA, TFT_BLACK, 15);

    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.println("Calibration complete!");

    // store data
    File f = SPIFFS.open(CALIBRATION_FILE, "w");
    if (f) {
      f.write((const unsigned char *)calData, 14);
      f.close();
    }
  }
}

//------------------------------------------------------------------------------------------

// Print something in the mini status bar
void status(const char *msg) {
  tft.setTextPadding(240);
  //tft.setCursor(STATUS_X, STATUS_Y);
  tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
  tft.setTextFont(0);
  tft.setTextDatum(TC_DATUM);
  tft.setTextSize(1);
  tft.drawString(msg, STATUS_X, STATUS_Y);
}
// Print temp
void PrintTemp() {
  tft.setCursor(TEMP_X, TEMP_Y);//Posición del texto en la pantalla
  tft.setTextColor(ILI9341_GREEN);//Setea el color del texto en verde
  tft.setTextSize(1);//Seteo del tamaño del texto
  tft.println("Temp:");// Se imprime en patalla la "Temp:"
   tft.setCursor(TEMP_X, (TEMP_Y+30));//Posición del texto en la pantalla
   tft.fillRect(TEMP_X,TEMP_Y+12, 100, 50, ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE);//Setea el color del texto en blanco
  tft.print(entradas_temp[2]);//Muestra la temperatura obtenida del sensor
}

//------------------------------------------------------------------------------------------
//*********FUNCIONES GENERALES********//

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
lectura_tempF= analogRead(pin_tempF)* (3.3 / 4096.0);
lectura_tempC= analogRead(pin_tempC)* (3.3 / 4096.0);
lectura_corriente= analogRead(pin_corriente)* (3.3 / 4096.0);
//activamos la variable auxiliar "leidos" para avisar a la otra funcion de que ya puede escribirlos
leidos=true;
  }

void ValorSensores(){ //calcula el valor de los sensores en su magnitud correcta y los escribe
if(leidos==true){
//Valor sensores de temperatura 
entradas_temp[0]=entradas_temp[1];
entradas_temp[1]=entradas_temp[2];
entradas_temp[2]= ((lectura_tempF/sensibilidadT)-4);
valor_tempC= ((lectura_tempC/sensibilidadT)-4);
//Valor sensor de corriente
entradas_corriente[0]=entradas_corriente[1];
entradas_corriente[1]=entradas_corriente[2];
entradas_corriente[2]= ((lectura_corriente-2.5)/sensibilidadC); //formula desarrollada en la memoria
leidos=false;
}
}
//Funcion para el control del puente H //
void ControlPuenteH(float pwm){
  //Si la corriente de entrada es positiva se activa una diagonal y si es negativa, la otra
  if(pwm>0){
  digitalWrite (IN2, LOW);
  digitalWrite (IN1, HIGH);
  }
  else {
  digitalWrite (IN1, LOW);
  digitalWrite (IN2, HIGH);
  }
  //Lo ideal sería en funcion de la señal de error (ciclo de trabajo) saber cuanto variar el ancho de PWM

  pwm=abs(pwm*sensibilidadC + 2.5); //pasamos de valor de corriente a analogico para pasarlo al PWM
  // Aplicamos PWM al pin ENB, modificando el ciclo de trabajo en funcion de la temperatura deseada
  analogWrite(ENA,pwm);
}
