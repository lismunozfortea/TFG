#if !defined(FW_POWERSTAGE_HPP)
#define FW_POWERSTAGE_HPP

#include <driver/gpio.h>
#include <driver/mcpwm.h>

namespace FW {
    class PowerStage
    {
        const gpio_num_t m_in1, m_in2, m_en;
    public:
        PowerStage(gpio_num_t in1_pin, gpio_num_t in2_pin, gpio_num_t en_pin) :
            m_in1 {in1_pin}, m_in2 {in2_pin}, m_en {en_pin} {};
        void begin(void);
        void duty(double duty);
        void shut_down(bool on);
    };

    inline void PowerStage::duty(double duty)
    {
        if (duty >= 0.0)
        {
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0.0f);
        }
        else
        {
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, -duty);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0.0f);
        }
    }
}

#endif
