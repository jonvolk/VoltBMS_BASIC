# Ampera/Volt specific modifications
### BMS

  - Balance Hysteresis (config example: 10mV)
    - balancing will start at 2 x hysteresis (eg. 20mV)
    - balancing will continue until hysteresis value is reached (eg. 10mV)
    - the end result will be somewhere around the hysteresis value +/- 5mV
> This is requred as the voltage sampling fluctuates highly (+/- 5mV) on individual cells, and will result in indefinite cycling of the charger on/off.

  - Will only work with the latest display software: [SimpDisplay] 


### Todos

 - Add 'sleep' mode for HV Voltage sampling (to stop HV drain while sitting)

[SimpDisplay]: <https://github.com/bogdan-toma/SimpDisplay>