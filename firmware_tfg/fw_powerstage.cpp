#include "fw_powerstage.hpp"

using namespace FW;

void PowerStage::begin(void)
{
    const mcpwm_pin_config_t pin_config {
        .mcpwm0a_out_num = m_in1,
        .mcpwm0b_out_num = m_in2
    };

    const mcpwm_config_t mcpwm_conf {
        .frequency    = 25000u,  // Hz
        .cmpr_a       = 0.0f,    // %
        .cmpr_b       = 0.0f,    // %
        .duty_mode    = MCPWM_DUTY_MODE_0,
        .counter_mode = MCPWM_UP_COUNTER
    };

    mcpwm_set_pin(MCPWM_UNIT_0, &pin_config);
    gpio_set_level(m_en, 0);
    gpio_set_direction(m_en, GPIO_MODE_OUTPUT); 
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &mcpwm_conf);
    mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
    gpio_set_level(m_en, 1);
}

void PowerStage::shut_down(bool shut)
{
    if (shut)
    {
        gpio_set_level(m_en, 0);
        mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0.0f);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0.0f);
    }
    else
    {
        mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
        gpio_set_level(m_en, 1);
    }
}
