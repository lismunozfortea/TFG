#if !defined(FW_PID_HPP)
#define FW_PID_HPP

namespace FW {
    struct PidParameters
    {
        double Kp, Ki, Kd, T;
    };

    class PidController
    {
        double m_coef[3];
        double m_e[3];
        double m_y[1];
        double m_ref;
    public:
        PidController(double ref, const struct PidParameters& params);
        double output(double input);
        double reference() const { return m_ref; }
        void   reference(double ref) { m_ref = ref; }
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
        double y = m_y[0]
                + m_coef[0] * m_e[0]
                + m_coef[1] * m_e[1]
                + m_coef[2] * m_e[2];
        m_e[2] = m_e[1];
        m_e[1] = m_e[0];
        m_y[0] = y;

        return y;  
    };

}

#endif
