# Fluid_control_system_Arduino
This is a set of Arduino code to control our customized pressure control system.

"Without interval" means the pneumatic system will kept running once it is turned on. If the switch of channel 1/2 is on, the rheostat can be used to tune the output pressure. If the switch is off, the pneumatic system will generate a constant output that is set in the code.

"With interval" means the pneumatic system will rest for 5 seconds after running for 5 seconds. If the switch of channel 1/2 is on, the rheostat can be used to tune the output pressure. If the switch is off, the pneumatic system will generate a constant output that is set in the code.

The sinewave code will generate a sin wave with a 20-second period (could be tuned in the code). If the switch of channel 1 is on, the rheostat can be used to tune the amplitude. If the switch is off, the amplitude will be constant that is set in the code.

The square wave code will generate a square wave with a 5-second period and 50% duty cycle (could be tuned in the code). If the switch of channel 1 is on, the rheostat can be used to tune the amplitude. If the switch is off, the amplitude will be constant that is set in the code.
