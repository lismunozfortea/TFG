#include "fw_gui.hpp"
#include "fw_pid.hpp"
#include "fw_powerstage.hpp"
#include "fw_sensorblock.hpp"

/*** #defines *******************************************************/

// Pins
#define IN1_PIN GPIO_NUM_13  // H bridge IN1 on GPIO13
#define IN2_PIN GPIO_NUM_14  // H bridge IN2 on GPIO14
#define ENA_PIN GPIO_NUM_15  // H bridge ENA on GPIO15

/******************************************************* #defines ***/

/*** Global constants ***********************************************/

// Fault conditions
const double t_hot_max    = 70.0;     // [ºC] Maximum hot plate temperature 
const double t_hot_resume = 40.0;     // [ºC] Post overheating safe hot plate temperature 
const double i_tem_max    =  3.0;     // [A]  Maximum TE module current

// PID controller parameters
const unsigned long t_sampling = 10000ul;  // Sampling time [us]

const struct FW::PidParameters pid_t_pams = { // Temperature
  .Kp = 9.12e-26,
  .Ki = 1.87e-27,
  .Kd = 8.93e-26,
  .T  = t_sampling * 1e-6
};

const struct FW::PidParameters pid_i_pams = { // Current
  .Kp = 1.420,
  .Ki = 0.945,
  .Kd = 0.157,
  .T  = t_sampling * 1e-6
};

/*********************************************** Global constants ***/

/*** Global variables ***********************************************/

FW::PidController t_pid   {25.0,   3.0, pid_t_pams};    // Tcold PID controller
FW::PidController i_pid   { 0.0, 100.0, pid_i_pams};    // Item  PID controller
FW::PowerStage    pwrstg  {IN1_PIN, IN2_PIN, ENA_PIN};  // Power stage
FW::SensorBlock   snsrblk;                              // Analog sensor block

/*********************************************** Global variables ***/

/*** Function prototypes ********************************************/

static void ctrl_loop_update(void);

/******************************************** Function prototypes ***/

void setup(void)
{
    Serial.begin(115200);
    gui_begin();
    snsrblk.begin();
    pwrstg.begin();

}

void loop(void) {
    static unsigned long nxt_run;

    if (micros() >= nxt_run)
    {
        snsrblk.update();       // Sample analogic sensors
        ctrl_loop_update();     // Update control signals

        nxt_run += t_sampling;  // Schedule update at now + t_sampling
    }
    gui_update();
}

static void ctrl_loop_update(void)
{
   
    static op_status_t op_status {OS_NORMAL};
    switch (op_status)
    {
        OS_NORMAL:
            // Fault condition management
            // Sensor malfunction
            if (snsrblk.bad_sensor())
            {
                pwrstg.shut_down(true);     // Safe mode: shut down bridge
                op_status = OS_BADSNSR;     // Register fault
                gui_notify(op_status);      // Signal user fault condition
                break;
            }

            // Over-current
            if (snsrblk.i_tem() >= i_tem_max)
            {
                pwrstg.shut_down(true);     // Safe mode: shut down bridge
                op_status = OS_OVRCURRENT;  // Register fault
                gui_notify(op_status);      // Signal user fault condition
                break;
            }

            // Over-temperature
            if (snsrblk.t_hot() >= t_hot_max)
            {
                pwrstg.shut_down(true);     // Safe mode: shut down bridge
                op_status = OS_OVRHEATED;   // Register fault
                gui_notify(op_status);      // Signal user fault condition
                break;
            }

            {
                // Update regulator outputs
                double current_setting = t_pid.output(snsrblk.t_cold());
                i_pid.reference(current_setting);
                double duty_cycle = i_pid.output(snsrblk.i_tem());

                // Update bridge duty cycle
                pwrstg.duty(duty_cycle);
            }
            break;

      OS_OVRHEATED:
          if (snsrblk.t_hot() < t_hot_resume)  // Safe t_hot
          {
              op_status = OS_NORMAL;    // Clear fault
              gui_notify(op_status);    // Signal user fault cleared
              t_pid.reset();
              i_pid.reset();
              pwrstg.shut_down(false);  // Resume bridge operation
              break;
          }

      OS_BADSNSR:
      OS_OVRCURRENT:    
      default:
          break;  // Shut down bridge forever
    }
    // Don't write code past this line
}
