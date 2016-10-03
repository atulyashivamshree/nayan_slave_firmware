# nayan_slave_firmware
Implements Loiter Code from ArduCopter on the nayan HLP

## Organization: 

*System files contain important functions for running RTOS and other functionalities and should not be changed*

### System Files
* `intercomm` -  System file. Handles communication between HLP and LLP
* `mcuconf.h` - System file
* `odroid_comm` - handles communication with an odroid system
* `Setup` - System file
* `stubs` - System file

### Ported Arducopter Code
* `autopilot_math.h` - implements key mathematical structures and operations
* `APM_config.h` - contains all parameters
* `inertial_nav.h` - implements INS code on the FCU
* `params.h` - implements saving and loading parameters
* `position_controller.h` - implements position controller
* `wp_nav.h` - Handles loiter and waypoint navigation

## Compatibility:
* `v0.1` - works with v0.1 of cv_sens and present in nayan_v2015 branch Does not work with mavros version after Mar 2016
* `v0.2` - Last tested version of code on Nayan-NUC, Inforce, Odroid boards. Works with `v0.2.2` of cv_sens and the master branch

###Flashing on the Slave Firmware
[Slave_Processor_Setup](http://aus.co.in/wiki/Slave_Processor_Setup)


###Nayan Wiki
Visit [http://aus.co.in/wiki](http://aus.co.in/wiki)

###Chibios Documentation
Visit [http://chibios.sourceforge.net/html/](http://chibios.sourceforge.net/html/) 

###MAVLink
Visit [http://qgroundcontrol.org/mavlink/start](http://qgroundcontrol.org/mavlink/start)
