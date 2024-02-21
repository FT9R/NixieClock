#pragma once

#define FIRST_START

#define VREF         (float) 1.1
#define ADC_SAMPLES  20
#define VOUT_TASK    180
#define R1_VAL       210000
#define R2_VAL       1000
#define CONNECTED    true
#define DISCONNECTED false
#if (ADC_SAMPLES > 255)
#error "Too much ADC samples"
#endif

#define K_P            (float) 0.01
#define K_I            (float) 0.00005
#define K_D            (float) 0.0
#define BOOST_DUTY_MIN (float) 0.0
#define BOOST_DUTY_MAX (float) 0.7

#define CATHODE_MASK   0x1E
#define ANODE_MASK     0xE0
#define CATHODE_OFFSET 1u
#define ANODE_OFFSET   5u

#define SET_BIT(REG, BIT)                   ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)                 ((REG) &= ~(BIT))
#define TOGGLE_BIT(REG, BIT)                ((REG) ^= (BIT))
#define READ_BIT(REG, BIT)                  ((REG) & (BIT))
#define CLEAR_REG(REG)                      ((REG) = (0x0))
#define WRITE_REG(REG, VAL)                 ((REG) = (VAL))
#define READ_REG(REG)                       ((REG))
#define MODIFY_REG(REG, CLEARMASK, SETMASK) WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))