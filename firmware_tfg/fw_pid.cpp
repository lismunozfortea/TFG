#include "fw_pid.hpp"

#include <cassert>

using namespace FW;

PidController::PidController(double ref, double sat, const struct PidParameters& params) :
    m_coef {
       params.Kp + 0.5 * params.Ki * params.T + params.Kd / params.T,
      -params.Kp + 0.5 * params.Ki * params.T - 2.0 * params.Kd / params.T,
       params.Kd / params.T
    },
    m_csat {
       params.Kp + params.Kd / params.T,
       params.Kd / params.T
    },
    m_e {0.0, 0.0, 0.0},
    m_y {0.0f},
    m_ref {ref},
    m_sat {sat}
{
    assert(!std::signbit(m_sat));
};
