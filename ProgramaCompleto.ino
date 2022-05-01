#include <TFT_eSPI.h>

#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include "SPI.h"
#include "FS.h"
/*Cosas que faltan:
 - Añadir opcion de apagar en cualquier momento la Peltier
 - Programar el menu que permite personalizar la medida
 - Coordinar los pines y conexiones de los sensores y  de la Peltier
 */

// Incluimos las librerías
#include <OneWire.h> // Librería para la comunicación con un solo cable 
//#include <URTouch.h>
//#include <URTouchCD.h>

// ********** TFT_eSPI screen **********
#define TFT_CS 15


// ********** TFT_eSPI touch **********
#define TOUCH_CS 21  //Touch CS to PIN 21
#define CALIBRATION_FILE "/TouchCalData2" // Calibration file stored in SPIFFS
#define REPEAT_CAL false // if true calibration is requested after reboot
#define totalButtonNumber 3

TFT_eSPI tft = TFT_eSPI();

#define LABEL1_FONT &FreeSansOblique12pt7b // Key label font 1
#define LABEL2_FONT &FreeSansBold12pt7b    // Key label font 2
TFT_eSPI_Button key[totalButtonNumber];  // TFT_eSPI button class



// Declaracion de variables globales
  float tempC; // Variable para almacenar el valor obtenido del sensor (0 a 1023)
  float tempF;
  int pinSensorF = 0; // Variable del pin de entrada del sensor frio (A0)
  int pinSensorC=1; // Variable del pin de entrada del sensor frio (A0)
  float temp_max = 60;
  float temp_min = -5;
    

  int puente_H = 9; // Pin digital 9 para la señal de entrada del puente
    
  // Variables internas para los pulsadores con enclavamiento
  int encender_Peltier = 0;
  int anterior_Peltier = 0;
  int estado_Peltier = 0;
  
  int Peltier = 0;

  
  OneWire ourWire(pinSensorF); // Se establece el pin digital 0 para la comunicación OneWire (no entiendo muy bien si necesito esto)
  OneWire ourWire(pinSensorC); 
  
  
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
    
    sensor.begin(); // Se inicializa el sensor de temperatura 

 // ********** TFT_eSPI screen library **********
  tft.begin();
 // tft.invertDisplay(false); // Solo requerido si los colores están invertidos
  touch_calibrate();  // Calibrate the touch screen and retrieve the scaling factors recorded in SD/SPIFFS

 // ********** General **********
  Serial.println("initialisation done...");


  // ********** First print **********
  int defcolor = ConvertRGBto565(131, 131, 131);
  botones_MenuInicio();
   

 
  }
  
  void loop() 
  {

    //Funciones que lee la temperatura del sensor 
    Lectura_Temperatura_fria();
    Lectura_Temperatura_caliente();
    // Función que controla el estado (ON/OFF) de la célula Peltier
    Celula_Peltier();
    
 
    
  }
//####################################################################################################
  //Funcion control pulsaciones de los botones iniciales TFT
//####################################################################################################
  void Pulsaciones_TFT()
  {

      uint16_t t_x = 0, t_y = 0; // coordenadas pulsacion
  bool pressed = tft.getTouch(&t_x, &t_y);  // true al pulsar

  // Comprueba si pulsas en zona de botón
  for (uint8_t b = 0; b < totalButtonNumber; b++) {
    if (pressed && key[b].contains(t_x, t_y)) {
      key[b].press(true);
      Serial.print(t_x);
      Serial.print(",");
      Serial.println(t_y);
    } else {
      key[b].press(false);
    }
  }

  // Accion si se pulsa boton
  for (uint8_t b = 0; b < totalButtonNumber; b++) {

    if (key[b].justReleased()) {
      //key[b].drawButton(); // redibuja al soltar

      switch (b) {
        case 0: //modo por defecto
         endender_Peltier=1;
         tft.fillScreen(defcolor);
         tft.setTextColor(ILI9341_BLACK);
          tft.setCursor(60,30);
          tft.print("Temperatura:");
        tft.setCursor(60,100);
        tft.print(tempF);
         tft.print("\337C");

         if (tempC>=temp_max || tempF>=temp_min)
         encender_Peltier=0;
         
         }
         
          break;
        case 1: //ir a temperatura definida por el usuario
          status("system Disabled");
         
          break;
           case 2: //definir limites de la curva de temperatura
          status("system Disabled");
         break;

         case 3: //apagado de emergencia
          status("system Disabled");
         break;
        default:
          delay(1);
          // statements
      }
    }
    if (key[b].justPressed()) {
      key[b].drawButton(true);  // cambia color del botón al pulsar
      delay(10); // UI debouncing
    }
  }

  
   

    delay(10); // evitar rebotes de pulsacion
  }
  }
//####################################################################################################
// Funcion para el menu de inicio ********** TFT_eSPI touch **********
//####################################################################################################
void botones_MenuInicio()
{
   int defcolor = ConvertRGBto565(131, 131, 131);
   tft.setRotation(1); //Horizontal
 tft.fillScreen(defcolor);
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(4);
  tft.setCursor(60,30);
  tft.print("¿Como desea medir?");

  // Draw the keys
  tft.setFreeFont(LABEL1_FONT);
 
  key[0].initButton(&tft, 80, 40, 110, 60, TFT_BLACK, TFT_WHITE, TFT_BLUE, "Mod.Defecto" , 1 ); // x, y, w, h, outline, fill, color, label, text_Size
  key[0].drawButton();
  key[1].initButton(&tft, 80, 115, 110, 60, TFT_BLACK, TFT_WHITE, TFT_BLUE, "Mod.Temp.Unica" , 1 );
  key[1].drawButton();
  key[2].initButton(&tft, 150, 40, 110, 60, TFT_BLACK, TFT_WHITE, TFT_BLUE, "Mod.Limites" , 1 ); // x, y, w, h, outline, fill, color, label, text_Size
  key[2].drawButton();
  key[3].initButton(&tft, 200, 200, 110, 60, TFT_BLACK, TFT_WHITE, TFT_BLUE, "Apagar" , 1 );
  key[3].drawButton();

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
  
  
  
  //Función que obtiene la posición presionada en la pantalla

 /* void Coordenadas_pulsador()
  {
    long x, y;
    
  //Comprueba si se ha presionado la pantalla 
  while(ts.dataAvailable())
  {
    ts.read();
    
    //Obtiene la posicion presionada
    x = ts.getX()+15;
    y = ts.getY()+5;

  }   
  }*/
//Función que controla las opciones y menus asi como su visualizacion

  /*void Menu_opciones()
  {
    //Si se pulsa comenzar se nos abre el menu para seleccionar el modo de medida
    
if((x=250) && (y=100))  //modificar esto para que coja todo el texto
   {
    tft.fillScreen(ILI9341_WHITE); 
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(4);
   tft.setCursor(60,30);
  tft.print("¿Como desea medir?");

   tft.setCursor(60,60);
  tft.print("Medida por defecto");
//Si seleccionamos medida por defecto se enciende la peltier y visualizamos la temperatura que mide el sensor
  if((x=60) && (y=60))
  {  
    encender_Peltier=1;
    tft.setCursor(60,30);
  tft.print("Temperatura:");
  
  tft.setCursor(60,100);
  tft.print(tempC);
  tft.print("\337C");

      if (tempC<= temp_min)
      {
        encender_Peltier =0;
         }

  }

   tft.setCursor(60,90);
  tft.print("Medida personalizada");
  
  //Si seleccionamos medida personalizada se nos abrirá un menú para elegir los límites que deseamos
  if((x=60) && (y=90))
  {
    //rellenar esto 
  }
     }
  }*/

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
