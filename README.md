# General description
This is a nixie clock project based on IN-16 gas-discharge tubes and ATtiny88 microcontroller.  
It is simplified and has no control buttons except for one reed switch, which is used to reset the time.
![Clock common view](Images/Clock.JPG)

## Features
* Full time info display: `hh:mm:ss`.
* Non-isolated boost DC-DC converter with feedback, that raise voltage from 15 to 180V
and helps to keep brightness at constant level.
* PID controller engaged in to reach precise voltage value with fast feedback response.
* Soft-start and soft-turnoff techniques to reduce electrical load of included components.
* Cathodes anti-degradation technology like in other nixie devices.
* Embedded temperature sensor with auto calibration.
* The supercapacitor helps to keep the RTC in working order for a long time without an input voltage and does not require replacement.
* A nighttime display pause to conserve tube resource.
