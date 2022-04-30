
#include <TouchScreen.h>
#include <Adafruit_TFTLCD.h>
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include "SPI.h"
/*Cosas que faltan:
 - Añadir opcion de apagar en cualquier momento la Peltier
 - Programar el menu que permite personalizar la medida
 - Coordinar los pines y conexiones con las reales
 */
#include <pin_magic.h>
#include <registers.h>
// Incluimos las librerías
#include <OneWire.h> // Librería para la comunicación con un solo cable 
#include <URTouch.h>
#include <URTouchCD.h>

  //Pins   (revisar)
#define TFT_DC 9
#define TFT_CS 10
//Pines Touch
#define t_SCK  3
#define t_CS   4
#define t_MOSI 5
#define t_MISO 6
#define t_IRQ  7

//Instanciamos TFT
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

//Instancimos el Touch
UTouch ts(t_SCK, t_CS, t_MOSI, t_MISO, t_IRQ);

// Declaracion de variables globales
  float tempC; // Variable para almacenar el valor obtenido del sensor (0 a 1023)
  float tempF;
  int pinSensor = 0; // Variable del pin de entrada del sensor (A0)
  int pinSensor2=1;
  float temp_max = 60;
  float temp_min = -40;
    

  int Puente_H = 9; // Pin digital 9 para la señal de entrada del puente
    
  // Variables internas para los pulsadores con enclavamiento
  int encender_Peltier = 0;
  int anterior_Peltier = 0;
  int estado_Peltier = 0;
  
  int Peltier = 0;

  
  OneWire ourWire(PIN_sensor); // Se establece el pin digital 0 para la comunicación OneWire (no entiendo muy bien si necesito esto)
  
  
  void setup() 
  {
    pinMode(PIN_sensor, INPUT); // Pin digital 0 como entrada
    pinMode(Puente_H, OUTPUT); // Pin digital 9 como salida
    
    // Cambiamos referencia de las entradas analógicas
  analogReference(INTERNAL);
  
  // Configuramos el puerto serial a 9600 bps (el puerto serial no será necesario seguramente)
  Serial.begin(9600);
    
    digitalWrite(Puente_H, LOW); // Puente_H inicialmente desconectado
    
    sensor.begin(); // Se inicializa el sensor de temperatura 

    //Configuracion de la TFT
    
  tft.begin();
  tft.setRotation(1); //Horizontal
  tft.fillScreen(ILI9341_WHITE); 
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(4);
  //Escribimos el texto:
  tft.setCursor(60,100);
  tft.print("Pulse para comenzar");

  //Configuracion del táctil
  ts.InitTouch();
  ts.setPrecision(PREC_MEDIUM); 
  }
  
  void loop() 
  {

    //Función que lee la temperatura del sensor 
    Lectura_Temperatura_fria();
    Lectura_Temperatura_calor();
    // Función que controla el estado (ON/OFF) de la célula Peltier
    Celula_Peltier();
    
    //Función que obtiene la posición presionada en la pantalla
    Coordenadas_pulsador();

    //Función que controla las opciones y menus asi como su visualizacion

    Menu_opciones();
    
  }

    //Función que lee la temperatura del sensor 
   void Lectura_temperatura_fria()
  {
       // Con analogRead leemos el sensor
  tempF = analogRead(pinSensor); 
   
  // Calculamos la temperatura con la fórmula
  tempF = (1.1 * tempF * 100.0)/1024.0; 
 
  // Envia el dato al puerto serial
  Serial.print(tempF);
  // Salto de línea
  Serial.print("\n");
  
  // Esperamos un tiempo para repetir el loop
  delay(1000);
  }
   //Función que lee la temperatura del sensor de la cara caliente
   void Lectura_temperatura_calor()
  {
       // Con analogRead leemos el sensor
  tempC = analogRead(pinSensor2); 
   
  // Calculamos la temperatura con la fórmula
  tempC = (1.1 * tempC * 100.0)/1024.0; 
 
  // Envia el dato al puerto serial
  Serial.print(tempC);
  // Salto de línea
  Serial.print("\n");
  
  // Esperamos un tiempo para repetir el loop
  delay(1000);
  }
  
  // Función que controla el estado (ON/OFF) de la célula Peltier
  void Celula_Peltier()
  {
      // Función que evalúa el estado del pulsador Peltier
     // Estado_Pulsador_Peltier();
      
      // Si se enciende el pulsador Peltier o la Tª es mayor o igual que TEMP_MAX se activa el Puente_H
      if(encender_Peltier == 1)
      {
        digitalWrite(Puente_H, HIGH);
  
      }
      
      // Si se vuelve a pulsar el pulsador o la Tª es menor o igual que TEMP_MIN se desactiva el Puente_H
      if(encender_Peltier == 0)
      {
        digitalWrite(Puente_H, LOW);
      
      }   

  }
  
  
  // Función que evalúa el estado del pulsador ON/OFF de la Célula Peltier (el pulsador será la TFT)
  /*void Estado_Pulsador_Peltier()
  {
    estado_Peltier = digitalRead(Pulsador_Peltier); // Comprobamos el estado actual del pulsador Peltier
    
    // Si el pulsador Peltier está presionado y su estado anterior es desactivado
    if(estado_Peltier && anterior_Peltier == 0)
    { 
      encender_Peltier = 1 - encender_Peltier;
    }
    
    anterior_Peltier = estado_Peltier; // Se actualiza el estado anterior del pulsador Peltier    
  }*/
  
  //Función que obtiene la posición presionada en la pantalla

  void Coordenadas_pulsador()
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
  }
//Función que controla las opciones y menus asi como su visualizacion

  void Menu_opciones()
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
  }
