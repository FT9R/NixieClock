#pragma once

#include <stdbool.h>

typedef enum displayModes_e { DISPLAY_TIME, DISPLAY_TEMPERATURE, DISPLAY_CAD } displayModes_t;

struct Indication_s
{
    uint8_t digit1;
    uint8_t digit2;
    uint8_t digit3;
    uint8_t digit4;
    uint8_t counter;
    displayModes_t dispMode;
    bool pause;
    bool isTurnedOff;
    bool pwmOutputStatus;
} indication;

struct Time_s
{
    uint8_t sec;
    uint8_t min;
    uint8_t hour;
} time;

struct CAD_s
{
    uint8_t digit1;
    uint8_t digit2;
    uint8_t digit3;
    uint8_t digit4;
    uint8_t digit5;
    uint8_t digit6;
    uint32_t counter;
    uint8_t updateStage;
    bool update;
} cad;

struct Temperature_s
{
    uint16_t value;
    uint16_t valueRef;
    int16_t compensationFactor;
    uint16_t compensationCounter;
    uint8_t msb;
    uint8_t lsb;
    bool isCompensationAllowed;
    bool isCompensated;
} temperature;

struct Voltage_s
{
    struct
    {
        float value;
        float valueScaled;
        float mean;
        uint32_t sum;
        uint8_t counter;
    } adc;
    struct
    {
        arm_pid_instance_f32 pidData;
        float setPoint;
        float processValue;
        bool run;
    } pid;
} voltage;