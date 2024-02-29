#pragma once

#define K_P                 0.01f
#define K_I                 0.00005f
#define K_D                 0.0f
#define BOOST_DUTY_MIN      0.0f
#define BOOST_DUTY_MAX      0.7f
#define VREF                1.1f
#define ADC_SAMPLES         100
#define VOUT_TASK           180
#define VOUT_REGL_SLOPE     1.026408451f
#define VOUT_REGL_INTERCEPT 1.076760563f
#define R1_VAL              210000
#define R2_VAL              1000
#define CONNECTED           true
#define DISCONNECTED        false
#if (ADC_SAMPLES > 255)
#error "Too much ADC samples"
#endif

#define CATHODE_MASK   0x1E
#define ANODE_MASK     0xE0
#define CATHODE_OFFSET 1u
#define ANODE_OFFSET   5u

#define USER_LED_ON  SET_BIT(PORTD, 1 << 0)
#define USER_LED_OFF CLEAR_BIT(PORTD, 1 << 0)

/* Macro */
#define SET_BIT(REG, BIT)                   ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)                 ((REG) &= ~(BIT))
#define TOGGLE_BIT(REG, BIT)                ((REG) ^= (BIT))
#define READ_BIT(REG, BIT)                  ((REG) & (BIT))
#define CLEAR_REG(REG)                      ((REG) = (0x0))
#define WRITE_REG(REG, VAL)                 ((REG) = (VAL))
#define READ_REG(REG)                       ((REG))
#define MODIFY_REG(REG, CLEARMASK, SETMASK) WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))