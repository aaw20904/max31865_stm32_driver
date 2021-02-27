# max31865_stm32_driver - 
It`s a device driver of IC MAX31865. These sofware was written in C++. I tried to follow principles of SOLID architecture - there are a class 'HAL_plug_MAX31865' :this class depends on hardware 
and has a variable-flag ('busy_flag') that use also in a Interrupt Service Routine to provide correct time intervals during transmission/reception through a SPI hardware interface. This class using HAL STM32 library -
but you can create your own implementation of this class for another hardware and platforms.
