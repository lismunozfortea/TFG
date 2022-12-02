#include <analogWrite.h>
#include <lvgl.h>
#include <TFT_eSPI.h>
#include <driver/adc.h>

//*********VARIABLES DE LA INTERFAZ********//

/*Change to your screen resolution*/
static const uint16_t screenWidth  = 320;
static const uint16_t screenHeight = 240;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[screenWidth * 10];

TFT_eSPI tft(screenWidth, screenHeight); /* TFT instance */

lv_obj_t *spinbox;
lv_obj_t *led1;
lv_obj_t *chart;


#if LV_USE_LOG != 0
/* Serial debugging */
void dbg_print(const char * buf)
{
    Serial.printf(buf);
    Serial.flush();
}
#endif


           
//------------------------------------------------------------

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
/*#define pin_tempF 32
#define pin_tempC 33
#define pin_corriente 34*/
#define IN1 13  // Input1 conectada al pin 13
#define IN2 14    // Input2 conectada al pin 14
#define ENA 15    // ENA conectada al pin 15, PWM
//Variables para las interrupciones //
volatile int contador;
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;


//*********FUNCIONES INTERFAZ********//

/* Display flushing */
void my_disp_flush( lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p )
{
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);
    tft.pushColors((uint16_t *)&color_p->full, w * h, true);
    tft.endWrite();

    lv_disp_flush_ready(disp);
}

/*Read the touchpad*/
void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
    uint16_t touchX, touchY;

    //bool touched = tft.getTouch(&touchX, &touchY, 600);
    bool touched = tft.getTouch(&touchX, &touchY);
    if(!touched)
        data->state = LV_INDEV_STATE_REL;
    else
    {
        data->state = LV_INDEV_STATE_PR;

        /*Set the coordinates*/
        data->point.x = touchX+45;
        data->point.y = touchY;

        Serial.print("Data x ");
        Serial.println(touchX);

        Serial.print("Data y ");
        Serial.println(touchY);
    }
}

//-------------/*Callbacks de eventos*/------------------------------------
static void lv_spinbox_increment_event_cb(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_SHORT_CLICKED || code  == LV_EVENT_LONG_PRESSED_REPEAT) {
        lv_spinbox_increment(spinbox);
    }
}

static void lv_spinbox_decrement_event_cb(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_SHORT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
        lv_spinbox_decrement(spinbox);
    }
}
//Cada vez que se modifique el valor del objeto ocurrirá el evento
static void lv_spinbox_value_event_cb(lv_event_t * e)
{
     
 temp_seleccionada = lv_spinbox_get_value(spinbox);
 
 }

////////////*Funciones ISR* //////////////////////////////
void IRAM_ATTR onTimer() { //ISR para el manejo de la interrupción de los PID's
  portENTER_CRITICAL_ISR(&timerMux);
  contador++;
  portEXIT_CRITICAL_ISR(&timerMux);
 
}
//------------------------------------------------------------------------------------------

void setup() {
  // Use serial port
  Serial.begin(115200);

   //*********SETUP INTERFAZ LVGL********//

    String LVGL_Arduino = "Hello Arduino! ";
    LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

    Serial.println(LVGL_Arduino);
    Serial.println("I am LVGL_Arduino");

    lv_init();

#if LV_USE_LOG != 0
    lv_log_register_print_cb(dbg_print); /* register print function for debugging */
#endif

    tft.begin();         /* TFT init */
    tft.setRotation(3);  /* Landscape orientation, flipped */

    /*Set the touchscreen calibration data,
     the actual data for your display can be acquired using
     the Generic -> Touch_calibrate example from the TFT_eSPI library*/
    uint16_t calData[5] = { 444, 3356, 366, 3336, 1};
    tft.setTouch(calData);

    lv_disp_draw_buf_init(&draw_buf, buf, NULL, screenWidth * 10);

    /*Initialize the display*/
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    /*Change the following line to your display resolution*/
    disp_drv.hor_res  = screenWidth;
    disp_drv.ver_res  = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    /*Initialize the (dummy) input device driver*/
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);

    /* Create simple label */
    spinbox = lv_spinbox_create(lv_scr_act());
    lv_spinbox_set_range(spinbox, -400, 700);
    lv_spinbox_set_digit_format(spinbox, 3, 2);
    lv_spinbox_step_prev(spinbox);
    lv_obj_set_width(spinbox, 55);
    lv_obj_align(spinbox, LV_ALIGN_TOP_MID, 0, 5);
    lv_obj_add_event_cb(spinbox,lv_spinbox_value_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    lv_coord_t h = lv_obj_get_height(spinbox);

    lv_obj_t *btn = lv_btn_create(lv_scr_act());
    lv_obj_set_size(btn, 3*h/4, h);
    lv_obj_align_to(btn, spinbox, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
    lv_obj_set_style_bg_img_src(btn, LV_SYMBOL_PLUS, 0);
    lv_obj_add_event_cb(btn, lv_spinbox_increment_event_cb, LV_EVENT_ALL,  NULL);

    btn = lv_btn_create(lv_scr_act());
    lv_obj_set_size(btn, 3*h/4, h);
    lv_obj_align_to(btn, spinbox, LV_ALIGN_OUT_LEFT_MID, -5, 0);
    lv_obj_set_style_bg_img_src(btn, LV_SYMBOL_MINUS, 0);
    lv_obj_add_event_cb(btn, lv_spinbox_decrement_event_cb, LV_EVENT_ALL, NULL);

    /*Create a LED and switch it OFF*/
    led1 = lv_led_create(lv_scr_act());
    lv_obj_align_to(led1, btn, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);
    lv_led_set_brightness(led1, 150);
    lv_led_set_color(led1, lv_palette_main(LV_PALETTE_RED));
    lv_led_off(led1);

    lv_obj_t *label = lv_label_create(lv_scr_act());
    lv_label_set_text(label, "Overheat");
    lv_obj_align_to(label, led1, LV_ALIGN_OUT_RIGHT_MID, 5, 0);

    /*Create a chart*/
    chart = lv_chart_create(lv_scr_act());
    lv_obj_set_size(chart, 90, 140);
    lv_obj_align_to(chart, spinbox, LV_ALIGN_OUT_BOTTOM_MID, 15, 45);
    lv_chart_set_type(chart, LV_CHART_TYPE_LINE);   /*Show lines and points too*/
    lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, -40, 80); 
    lv_chart_set_axis_tick(chart, LV_CHART_AXIS_PRIMARY_Y, 10, 5, 7, 2, true, 35);

    /*Add two data series*/
    lv_chart_series_t *ser1 = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_RED), LV_CHART_AXIS_PRIMARY_Y);
    lv_chart_series_t *ser2 = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_GREEN), LV_CHART_AXIS_SECONDARY_Y);

/////////////////////////////////////////////////////////////////////////ACTUALIZAR CON TEMPERATURA EN LOOP//////////////////////
    /*Set the next points on 'ser1'*/
    lv_chart_set_next_value(chart, ser1, 10);
    lv_chart_set_next_value(chart, ser1, 10);
    lv_chart_set_next_value(chart, ser1, 10);
    lv_chart_set_next_value(chart, ser1, 10);
    lv_chart_set_next_value(chart, ser1, 10);
    lv_chart_set_next_value(chart, ser1, 10);
    lv_chart_set_next_value(chart, ser1, 10);
    lv_chart_set_next_value(chart, ser1, 30);
    lv_chart_set_next_value(chart, ser1, 70);
    lv_chart_set_next_value(chart, ser1, 80);

    /*Directly set points on 'ser2'*/
    ser2->y_points[0] = 80;
    ser2->y_points[1] = 70;
    ser2->y_points[2] = 65;
    ser2->y_points[3] = 65;
    ser2->y_points[4] = 65;
    ser2->y_points[5] = 65;
    ser2->y_points[6] = 65;
    ser2->y_points[7] = 65;
    ser2->y_points[8] = 65;
    ser2->y_points[9] = 65;

    lv_chart_refresh(chart); /*Required after direct set*/
 //////////////////////////////////////////////////////////////////////////////ACTUALIZAR CON TEMPERATURA EN LOOP/////////////////7
   //*********SETUP GENERAL********//

  //Setup adc channels
  adc1_config_width(ADC_WIDTH_12Bit);
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);// using GPIO 34 corriente (ajustar DB en función de lo que necesitemos)
  adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11);// using GPIO 32 temp fria 
  adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11);// using GPIO 33 temp caliente

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
 
  lv_timer_handler(); /* let the GUI do its work */

    delay(5); //Igual hay que quitarlo!!!

 //Accion si se sobrepasasa determinada corriente
  if (entradas_corriente[2]=3.0)
{ciclo_trabajo=0; 
digitalWrite(IN1,LOW);
digitalWrite(IN2,LOW);
}

//Accion si se sobrepasa determinado valor de temperatura
 if(valor_tempC=40.0){
    lv_led_on(led1);
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
}
  
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
lectura_tempF= float(adc1_get_raw(ADC1_CHANNEL_4))* (3.3 / 4096.0);
lectura_tempC= float(adc1_get_raw(ADC1_CHANNEL_5))* (3.3 / 4096.0);
lectura_corriente= float(adc1_get_raw(ADC1_CHANNEL_6))* (3.3 / 4096.0);

/*lectura_tempF= analogRead(pin_tempF)* (3.3 / 4096.0);
lectura_tempC= analogRead(pin_tempC)* (3.3 / 4096.0);
lectura_corriente= analogRead(pin_corriente)* (3.3 / 4096.0);*/

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
