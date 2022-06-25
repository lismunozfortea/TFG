// ***** LIBRERIAS *****
#include <TFT_eSPI.h>
#include <PIDController.hpp>
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include "SPI.h"
#include "FS.h"
/*Cosas que faltan:
 - Coordinar los pines y conexiones de los sensores y  de la Peltier
 */

//#include <OneWire.h> // Librería para la comunicación con un solo cable 


// ********** TFT_eSPI screen **********
#define TFT_CS 15


// ********** TFT_eSPI touch **********
#define TOUCH_CS 21  //Touch CS to PIN 21
#define CALIBRATION_FILE "/TouchCalData2" // Calibration file stored in SPIFFS
#define REPEAT_CAL false // if true calibration is requested after reboot
#define totalButtonNumber 20

TFT_eSPI tft = TFT_eSPI();

#define LABEL1_FONT &FreeSansOblique12pt7b // Key label font 1
#define LABEL2_FONT &FreeSansBold12pt7b    // Key label font 2
TFT_eSPI_Button key[totalButtonNumber];  // TFT_eSPI button class

// ********** PID **********
const int PIN_INPUT = 0; //revisar
const int PIN_OUTPUT = 3;

PID::PIDParameters<double> parameters(4.0, 0.2, 1);
PID::PIDController<double> pidController(parameters);

// ********** Sensor de corriente **********//
float sensibility= 0.185;
int num_muestras=100;

// *****Declaracion de variables globales *****
  float tempC; // Variable para almacenar el valor obtenido del sensor (0 a 1023)
  float tempF;
  int pinSensorF = 0; // Variable del pin de entrada del sensor frio (A0)
  int pinSensorC=1; // Variable del pin de entrada del sensor frio (A0)
  float temp_max = 60;
  float temp_min = -5;
  float temp_usuario; //temperatura definida por el usuario
 /*float temp_lim1; //temperatura inferior del límite
  float temp_lim2; //temperatura superior del límite*/
  int seleccion[1];
    

  int puente_H = 9; // Pin digital 9 para la señal de entrada del puente
    
  // ****** Variables internas para los pulsadores con enclavamiento *****
  int encender_Peltier = 0; //pulsador modo por defecto
  int encender_PID = 0; //pulsador para modo PID
  


  
  /*OneWire ourWire(pinSensorF); // Se establece el pin digital 0 para la comunicación OneWire (no entiendo muy bien si necesito esto)
  OneWire ourWire(pinSensorC); */
  
  
  void setup() 
  {
    pinMode(pinSensorF, INPUT); // Pin digital 0 como entrada
    pinMode(pinSensorC, INPUT); 
    pinMode(puente_H, OUTPUT); // Pin digital 9 como salida
    
     // Set all chip selects high to avoid bus contention during initialisation of each peripheral
  digitalWrite(TOUCH_CS, HIGH); // ********** TFT_eSPI touch **********
  digitalWrite(TFT_CS, HIGH); // ********** TFT_eSPI screen library **********
    Serial.begin(115200);
  delay (2000);
    
    // Cambiamos referencia de las entradas analógicas
  analogReference(INTERNAL);
  
    
    digitalWrite(Puente_H, LOW); // Puente_H inicialmente desconectado
    
    //sensor.begin(); // Se inicializan los sensores de temperatura 

     // ********** Inicializar PID ********** 
     pidController.Input = analogRead(PIN_INPUT);
  pidController.Setpoint = 100;
  pidController.TurnOn();

 // ********** TFT_eSPI screen library **********
  tft.begin();
 // tft.invertDisplay(false); // Solo requerido si los colores están invertidos
  touch_calibrate();  // Calibrate the touch screen and retrieve the scaling factors recorded in SD/SPIFFS

 // ********** General **********
  Serial.println("initialisation done...");


  // ********** First print **********
  int defcolor = ConvertRGBto565(131, 131, 131);
  Pulsaciones_ModoTempUnica();
   

 
  }
  
  void loop() 
  {

    //Funciones que lee la temperatura del sensor 
    Lectura_Temperatura_fria();
    Lectura_Temperatura_caliente();
    // Función que controla las pulsaciones 
    Pulsaciones_ModoTempUnica();
     // Función que controla el estado (ON/OFF) de la célula Peltier
    Celula_Peltier();
    //Funcion que controla el boton de apagado
     Apagar();
     // Valores sensor de corriente
     float corriente = getCorriente(num_muestras);
   float corrienteRMS = 0.707 * corriente;
   float power = 230.0 * corrienteRMS; 
    
//****MODO TEMP.UNICA ******
If(encender_PID==1)
{
    encender_Peltier=1;
        tft.fillScreen(defcolor);
         tft.setTextColor(ILI9341_BLACK);
          tft.setCursor(60,30);
          tft.print("Temperatura:");
        tft.setCursor(60,100);
        tft.print(tempF);
         tft.print("\337C");
     pidController.Input = analogRead(PIN_INPUT);
      pidController.Update();
  
  analogWrite(PIN_OUTPUT, pidController.Output);
  
    //apagado de emergencia en caso de llegar a los límites establecidos
           if (tempC>=temp_max || tempF<=temp_min){
         encender_PID=0;
         encender_Peltier=0;
 }
         //funcion para que aparezca un aviso cuando la temperatura se estabilice

         If(tempF==temp_usuario) //mas bien seria cuando dejara de variar pero provisionalmente pongo esto
         {
           tft.setCursor(60,120);
          tft.print("Temperatura estable, apunte los resultados");
          encender_Peltier=0;
          encender_PID=0;
          
         }

          //incluir botón que permita volver atrás para introducir otra temperatura y volver a medir

          
    }
 
    
  }
 

  //####################################################################################################
  //Funcion control pulsaciones de los botones en la opción PID (elige temperatura el usuario)
//####################################################################################################
 void Pulsaciones_ModoTempUnica()
  {
         tft.fillScreen(defcolor);
         tft.setTextColor(ILI9341_BLACK);
          tft.setCursor(60,20);
          tft.print("Introduzca la temperatura:");
        botones_Numerico();
        

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
        if(seleccion[0]=/0)
        {
          tft.setCursor(60,30);
          tft.print("0");
          seleccion[0]=0;
          else
          tft.setCursor(70,30);
          tft.print("0");
          seleccion[1]=0;
          }

         
         
          break;
        case 1:
       
          if(seleccion[0]=/1)
        {
          tft.setCursor(60,30);
          tft.print("1");
          seleccion[0]=1;
          else
          tft.setCursor(70,30);
          tft.print("1");
          seleccion[1]=1;
          }

        
          break;
           case 2: 
             if(seleccion[0]=/2)
        {
          tft.setCursor(60,30);
          tft.print("2");
          seleccion[0]=2;
          else
          tft.setCursor(70,30);
          tft.print("2");
          seleccion[1]=2;
          }

        
         break;

         case 3: 
          if(seleccion[0]=/3)
        {
          tft.setCursor(60,30);
          tft.print("3");
          seleccion[0]=3;
          else
          tft.setCursor(70,30);
          tft.print("3");
          seleccion[1]=3;
          }

      
         break;
         case 4:
       
          if(seleccion[0]=/4)
        {
          tft.setCursor(60,30);
          tft.print("4");
          seleccion[0]=4;
          else
          tft.setCursor(70,30);
          tft.print("4");
          seleccion[1]=4;
          }

        
          break;
           case 5: 
         if(seleccion[0]=/5)
        {
          tft.setCursor(60,30);
          tft.print("5");
          seleccion[0]=5;
          else
          tft.setCursor(70,30);
          tft.print("5");
          seleccion[1]=5;
          }

         break;

         case 6: 
          if(seleccion[0]=/6)
        {
          tft.setCursor(60,30);
          tft.print("6");
          seleccion[0]=6;
          else
          tft.setCursor(70,30);
          tft.print("6");
          seleccion[1]=6;
          }

      
         break;
         case 7:
         if(seleccion[0]=/7)
        {
          tft.setCursor(60,30);
          tft.print("7");
          seleccion[0]=7;
          else
          tft.setCursor(70,30);
          tft.print("7");
          seleccion[1]=7;
          }

        
          break;
           case 8: 
          if(seleccion[0]=/8)
        {
          tft.setCursor(60,30);
          tft.print("8");
          seleccion[0]=8;
          else
          tft.setCursor(70,30);
          tft.print("8");
          seleccion[1]=8;
          }

         break;

         case 9: 
       if(seleccion[0]=/9)
        {
          tft.setCursor(60,30);
          tft.print("9");
          seleccion[0]=9;
          else
          tft.setCursor(70,30);
          tft.print("9");
          seleccion[1]=9;
          }

         break;

          case 10: //botón de start
         { // se convierte el vector en un float
        temp_usuario= atof(seleccion); //no estoy segura de esto
        //decimos al PID que se active
        encender_PID=1;
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



//####################################################################################################
  //Funcion para el apagado de emergencia
//####################################################################################################

void Apagar(){
  
 apagado.initButton(&tft, 200, 200, 110, 60, TFT_BLACK, TFT_WHITE, TFT_BLUE, "Apagar" , 1 );
 apagado.drawButton();

 uint16_t a_x = 0, a_y = 0; // coordenadas pulsacion
  bool pressed = tft.getTouch(a_x, &a_y);  // true al pulsar

  // Comprueba si pulsas en zona de botón
 
    if (pressed && apagado.contains(a_x, a_y)) {
      apagado.press(true);
      Serial.print(a_x);
      Serial.print(",");
      Serial.println(a_y);
    } else {
      apagado.press(false);
    }
  }

  // Accion si se pulsa boton
    if (apagado.justReleased()) {
    apagado.drawButton(); // redibuja al soltar
 
     encender_Peltier=0;
     encender_PID=0;
} 

  } 

  
//####################################################################################################
// Funcion para dibujar el teclado numérico ********** TFT_eSPI touch **********
//####################################################################################################
void botones_Numerico()
{


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

}
//####################################################################################################
  //Función que lee la temperatura del sensor cara fria
//####################################################################################################
   void Lectura_temperatura_fria()
  {
       // Con analogRead leemos el sensor
  tempF = analogRead(pinSensorF); 
   
  // Calculamos la temperatura con la fórmula
  tempF = (1.1 * tempF * 100.0)/1024.0; 
 
  // Envia el dato al puerto serial
  Serial.print(tempF);
  // Salto de línea
  Serial.print("\n");
  
  // Esperamos un tiempo para repetir el loop
  delay(1000);
  }

 //####################################################################################################
 //Función que lee la temperatura del sensor de la cara caliente
//####################################################################################################
   void Lectura_temperatura_caliente()
  {
       // Con analogRead leemos el sensor
  tempC = analogRead(pinSensorC); 
   
  // Calculamos la temperatura con la fórmula
  tempC = (1.1 * tempC * 100.0)/1024.0; 
 
  // Envia el dato al puerto serial
  Serial.print(tempC);
  // Salto de línea
  Serial.print("\n");
  
  // Esperamos un tiempo para repetir el loop
  delay(1000);
  }
//####################################################################################################
 // Función que obtiene el valor de la corriente del sensor de corriente
//####################################################################################################

float getCorriente(int nmuestras)
{
   float voltaje;
   float corrienteSum = 0;
   for (int i = 0; i < nmuestras; i++)
   {
      voltaje = analogRead(A0) * 5.0 / 1023.0; //CAMBIAR EL PIN A POR EL QUE SEA EL CORRECTO
      corrienteSum += (voltaje - 2.5) / sensibility;
   }
   return(corrienteSum / nmuestras);
}
  //####################################################################################################
 // Función que controla el estado (ON/OFF) de la célula Peltier
//####################################################################################################
  void Celula_Peltier()
  {
      
      
      // Si se enciende el pulsador Peltier o la Tª es mayor o igual que TEMP_MAX se activa el Puente_H
      if(encender_Peltier == 1)
      {
        digitalWrite(puente_H, HIGH);
  
      }
      
      // Si se vuelve a pulsar el pulsador o la Tª es menor o igual que TEMP_MIN se desactiva el Puente_H
      if(encender_Peltier == 0)
      {
        digitalWrite(puente_H, LOW);
      
      }   

  }
  
  
  
//####################################################################################################
// RGB 24 bits to RGB565 (16bits) conversion
//####################################################################################################
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
  //####################################################################################################
// screen calibration ********** TFT_eSPI touch **********
//####################################################################################################
void touch_calibrate()
{
  uint16_t calData[5];
  uint8_t calDataOK = 0;

  if (existSD) {
    // check if calibration file exists and size is correct
    if (SD.exists(CALIBRATION_FILE)) {
      if (REPEAT_CAL)
      {
        // Delete if we want to re-calibrate
        SD.remove(CALIBRATION_FILE);
      }
      else
      {
        File f = SD.open(CALIBRATION_FILE, "r");
        if (f) {
          if (f.readBytes((char *)calData, 14) == 14)
            calDataOK = 1;
          f.close();
        }
      }
    }
  }
  else  // SPIFFS uses
  {
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
    Serial.print(calData[1]);
    Serial.print(",");
    Serial.print(calData[2]);
    Serial.print(",  ");
    Serial.print(calData[3]);
    Serial.print(",");
    Serial.print(calData[4]);
    Serial.print(",  ");
    Serial.print(calData[5]);
    Serial.print(",");
    Serial.print(calData[6]);
    Serial.print(",  ");
    Serial.print(calData[7]);
    Serial.print(",");
    Serial.println(calData[8]);

    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.println("Calibration complete!");

    // store data
    if (existSD) {
      File f = SD.open(CALIBRATION_FILE, "w");
      if (f) {
        f.write((const unsigned char *)calData, 14);
        f.close();
      }
    }
    else {
      File f = SPIFFS.open(CALIBRATION_FILE, "w");
      if (f) {
        f.write((const unsigned char *)calData, 14);
        f.close();
      }
    }

  }
}
