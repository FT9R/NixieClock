# Nixie-Clock
This is the project of nixie clock based on IN-16 tubes and low-cost ATtiny88 Microcontroller. They are simplified, so they don't have any control buttons, except one reed switch. Reed switch is used to reset time to 20:00.
If you have any issues with time synchronization, you can trim the value of C17.

This project contain DC-DC converter, that raise voltage from 15 to 180V. PID regulator ingaged in to reach precise voltage level with fast response on feedback. It also use soft start and soft turn off techiques to reduce electrical load of included components and power supply.
Like other nixie devices, this one uses cathode anti-degradation technology.

Also, this device contains a temperature sensor, the readings from which will be displayed on the indicators from 10 to 15 seconds. But it doesn't work very well. The reason for this is its location right on the board, which heats up from other components. Attention was paid to this and the sensor was moved further from the components, but this is not enough. Therefore, within one hour after the start, the sensor will be calibrated depending on the environment.
Principle: the sensor receives one temperature value at start-up and a second value one hour later. The difference between these values is used in the future until the clock is turned off.

At the current moment this project is deprecated. The reasons for this are addiction on value of timing capacitor of RTC and temperature sensor calibration necessity.
The solution to the first problem is the use of synchronization with satellites through GPS interface. The solution to the second problem is the removal of the sensor on a separate board, excluding its heating. I will take care about in future projects.

WARNING!!!
It must be taken into attention that tubes have a parasitic glow. The introduced dead time between switching of anodes does not sufficiently help to get rid of it. Therefore, in the future, zener diodes D11-D21 were added to the circuit. But the board has not been updated anymore. Based on this, you should additionally solder the zener diodes on the underside of the main board as shown in the attached images.
