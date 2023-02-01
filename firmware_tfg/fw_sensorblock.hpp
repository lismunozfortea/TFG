#if !defined(FW_SENSORBLOCK_HPP)
#define FW_SENSORBLOCK_HPP

#include <driver/adc.h>
#include <esp_adc_cal.h>




// ADC1 channels
#define ADC1_TCOLD_CHANNEL ADC1_GPIO32_CHANNEL  // Cold plate temperature on GPIO32
#define ADC1_THOT_CHANNEL ADC1_GPIO33_CHANNEL   // Hot plate temperature on GPIO33
#define ADC1_ITEM_CHANNEL ADC1_GPIO34_CHANNEL   // TE module average current on GPIO34

#define DEFAULT_VREF 1100  // [mV] Use adc1_vref_to_gpio() to obtain a better estimate

namespace FW {
class SensorBlock {
  // LM335 (from datasheet)
  const double t_gain = 0.0647;   // Gain [V/K]
  const double t_offset25 = 1.986306566;  // Offset@25ºC [V]

  // ACS712 5A (from datasheet)
  const double i_gain = 0.185;     // Gain [V/A]
  const double i_offset0 = 2.46163303;  // Offset@0A [V]
  /*
  const double i_offset0 = 2.41459615;  // Offset@0A [V]*/

  double m_t_cold;
  double m_t_hot;
  double m_i_tem;
  esp_adc_cal_characteristics_t m_adc_chars;  // ADC calibration
  bool m_bad_sensor;
  //

  uint32_t result_i = 0;
  uint32_t result_tc = 0;
  uint32_t result_th = 0;
  int counter = 0;


public:
  SensorBlock(void)
    : m_bad_sensor{ false } {};
  double bad_sensor() const {
    return m_bad_sensor;
  }                  // Get sensor status
  void begin(void);  // Initialization
  double t_cold() const {
    return m_t_cold;
  }  // Cold plate temperature [ºC]
  double t_hot() const {
    return m_t_hot;
  }  // Hot plate temperature  [ºC]
  double i_tem() const {
    return m_i_tem;
  }  // Thermoelectric module current [A]
  void update(void);
};


inline void SensorBlock::update(void) {


  const double t_offset0 = t_offset25 - 25.0 * t_gain;  // Offset@0ºC [V]

  // Sample analogic sensors
  uint32_t t_cold_raw = adc1_get_raw(ADC1_TCOLD_CHANNEL);
  uint32_t t_hot_raw = adc1_get_raw(ADC1_THOT_CHANNEL);
  uint32_t i_tem_raw = adc1_get_raw(ADC1_ITEM_CHANNEL);


  // Get calibrated voltages
  uint32_t t_cold_mv = esp_adc_cal_raw_to_voltage(t_cold_raw, &m_adc_chars);
  uint32_t t_hot_mv = esp_adc_cal_raw_to_voltage(t_hot_raw, &m_adc_chars);
  uint32_t i_tem_mv = esp_adc_cal_raw_to_voltage(i_tem_raw, &m_adc_chars);

  // Save calibrated voltages in the variable "result"
  result_i += i_tem_mv;
  result_tc += t_cold_mv;
  result_th += t_hot_mv;

  //Translation to process physical units every 16 samples
  if (++counter >= 16) {

    counter = 0;

    m_i_tem = ((double)result_i * 6.25e-5 - i_offset0) / i_gain;    // Item  [A]
    m_t_cold = ((double)result_tc * 6.25e-5 - t_offset0) / t_gain;  // Tcold [ºC]
    m_t_hot = ((double)result_th * 6.25e-5 - t_offset0) / t_gain;   // Thot  [ºC]


    result_i = 0;
    result_tc = 0;
    result_th = 0;
  }

}
}

#endif