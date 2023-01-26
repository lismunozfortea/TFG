#if !defined(FW_PID_HPP)
#define FW_PID_HPP

#include <cmath>

namespace FW {
    struct PidParameters
    {
        double Kp, Ki, Kd, T;
    };

    class PidController
    {
        double m_coef[3];
        double m_csat[2];
        double m_e[3];
        double m_y[1];
        double m_ref;
        double m_sat;
        bool   m_saturated = false;
    public:
        PidController(double ref, double sat, const struct PidParameters& params);
        double output(double input);
        double reference() const { return m_ref; }
        void   reference(double ref) { m_ref = ref; }
        double saturateAt() const { return m_sat; }
        void   reset(void)
        {
            m_e[0] = 0.0, m_e[1] = 0.0, m_e[2] = 0.0;
            m_y[0] = 0.0;
        }        
    };

    inline double PidController::output(double input)
    {
        // Discrete PID controller difference equation
        m_e[0] = input - m_ref;
        double y;
        if (!m_saturated)
            y = m_y[0]
                + m_coef[0] * m_e[0]
                + m_coef[1] * m_e[1]
                + m_coef[2] * m_e[2];
        else
            y = m_csat[0] * m_e[0]
                + m_csat[1] * m_e[1];

        if (std::abs(y) <= m_sat)
            m_saturated = false;
        else
        {
            y = copysign(m_sat, y);
            m_saturated = true;
        }

        m_e[2] = m_e[1];
        m_e[1] = m_e[0];
        m_y[0] = y;

        return y;  
    };

}

#endif
