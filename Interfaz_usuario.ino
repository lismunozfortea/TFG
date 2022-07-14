// ***** LIBRERIAS *****
#include <SPI.h>
#include <FS.h>

// ********** TFT_eSPI screen **********
#define TFT_CS 15  //TFT CS to PIN 15
#include <TFT_eSPI.h>
TFT_eSPI tft = TFT_eSPI();

// ********** TFT_eSPI touch **********
//#define TOUCH_CS 21  //Touch CS to PIN 21
#define CALIBRATION_FILE "/TouchCalData2" // Calibration file stored in SPIFFS
#define REPEAT_CAL false // if true calibration is requested after reboot
#define totalButtonNumber 20


#define LABEL1_FONT &FreeSansOblique12pt7b // Key label font 1
#define LABEL2_FONT &FreeSansBold12pt7b    // Key label font 2
TFT_eSPI_Button keyN[totalButtonNumber];  // TFT_eSPI button class

float temp_seleccionada;
int start;
float seleccion[1];

//Funcion conversion de colores
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
        //temp_seleccionada= atof(seleccion); 
        temp_seleccionada= seleccion[0]*10 + seleccion[1];
      start=1;
         }
         break;

         case 11:
         //se apaga todo
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
  void setup(){
    
  digitalWrite(TOUCH_CS, HIGH); // ********** TFT_eSPI touch **********
  digitalWrite(TFT_CS, HIGH); // ********** TFT_eSPI screen library **********
    Serial.begin(115200);
    delay(2000);
    
     
  tft.begin();
  tft.invertDisplay(false); // I don't know why but it is required for my TFT to color correction
  tft.setRotation(1);  // Landscape
  //touch_calibrate();  // Calibrate the touch screen and retrieve the scaling factors recorded in SD/SPIFFS


  // ********** General **********
  Serial.println("initialisation done...");


  // ********** First print **********
  int defcolor = ConvertRGBto565(131, 131, 131);
    DibujarBotones();
  }
  
  void loop(){
    // Función que controla las pulsaciones 
    Teclado();
    
//****MODO TEMP.UNICA ******
if(start==1)
{
 
        tft.fillScreen(defcolor);
         tft.setTextColor(ILI9341_BLACK);
          tft.setCursor(60,30);
          tft.print("Temperatura:");
        tft.setCursor(60,100);
        tft.print(salidas_temp[2]);
         tft.print("\337C");

         //funcion para que aparezca un aviso cuando la temperatura se estabilice

       /*  if(tempF==temp_usuario) //mas bien seria cuando dejara de variar pero provisionalmente pongo esto
         {
           tft.setCursor(60,120);
          tft.print("Temperatura estable, apunte los resultados");
         start=0;
          
         }
         */

          //incluir botón que permita volver atrás para introducir otra temperatura y volver a medir

          
    }
  }
  }
