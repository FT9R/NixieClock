# General description
This is a nixie clock project based on IN-16 gas-discharge tubes and an inexpensive ATtiny88 microcontroller.  
It is simplified and has no control buttons except for one reed switch, which is used to reset the time.
![Clock common view](images/Clock.jpg)

## Features
* Full time info display: `hh:mm:ss`.
* Non-isolated boost DC-DC converter with feedback, that raise voltage from 15 to 180V.  
It helps to keep brightness at constant level. PID controller engaged in to reach precise voltage value with fast feedback response.
* Soft-start and soft-turnoff techniques to reduce electrical load of included components.
* Cathodes anti-degradation technology like in other nixie devices.
* Embedded temperature sensor with auto calibration.
* The supercapacitor helps to keep the RTC in working order for a long time without an input voltage and does not require replacement like a battery cell.
* A nighttime display pause to conserve tube resource.

## Remarks
During the clock adjustment, a parasitic glow of the cathodes was detected. Zener diodes D10-D21 with breakdown voltage 75V were used to eliminate this effect. But the PCB has not changed since the first prototype, so the zener diodes must be installed as shown in the figure below
![Zener diodes placement](images/Zener_diodes.jpg)