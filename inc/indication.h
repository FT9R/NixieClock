#pragma once

#include "dataTypes.h"
#include "definitions.h"
#include <avr/io.h>

void AnodeSwitch(uint8_t anode);
void CathodeSwitch(uint8_t cathode);
void Display_DeadTime(void);
void SoftStart(struct Indication_s *indication, struct Voltage_s *voltage);
void SoftTurnoff(struct Indication_s *indication, struct Voltage_s *voltage);