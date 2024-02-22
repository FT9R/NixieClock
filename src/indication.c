#include "indication.h"

void AnodeSwitch(uint8_t anode)
{
    if ((anode >= 1) && (anode <= 6))
        MODIFY_REG(PORTD, ANODE_MASK, anode << ANODE_OFFSET);
}

void CathodeSwitch(uint8_t cathode)
{
    if ((cathode >= 0) && (cathode <= 9))
        MODIFY_REG(PORTD, CATHODE_MASK, cathode << CATHODE_OFFSET);
}

void Display_DeadTime(void)
{
    MODIFY_REG(PORTD, CATHODE_MASK | ANODE_MASK, 12u << CATHODE_OFFSET);
}

void SoftStart(struct Indication_s *indication, struct Voltage_s *voltage)
{
    if ((!indication->pause) && (!indication->isTurnedOff) && (voltage->pid.setPoint <= VOUT_TASK))
        voltage->pid.setPoint += 0.01;
}

void SoftTurnoff(struct Indication_s *indication, struct Voltage_s *voltage)
{
    if ((indication->isTurnedOff) && (voltage->pid.setPoint > 0.0))
        voltage->pid.setPoint -= 0.005;
}