#include "fw_sensorblock.hpp"

using namespace FW;

void SensorBlock::begin(void)
{
    // Configure ADC
    adc1_config_width(ADC_WIDTH_12Bit);
    adc1_config_channel_atten(ADC1_TCOLD_CHANNEL, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_THOT_CHANNEL , ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_ITEM_CHANNEL , ADC_ATTEN_DB_11);

    // Characterize ADC
    esp_adc_cal_characterize(
        ADC_UNIT_1,
        ADC_ATTEN_DB_11,
        ADC_WIDTH_12Bit,
        DEFAULT_VREF,
        &m_adc_chars
    );
}
