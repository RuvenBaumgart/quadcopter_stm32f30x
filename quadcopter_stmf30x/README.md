# Flightcontroller software quadcopter_stm32f30x
**flight controller software (firmware) used to fly quadrocopter crafts**

## Important Notes
This firmware is in a development stage still and should only be used as an reference. It is not for a broad variatiy of flight controllers but only for those with a stm32f303 mcu and only for those with a pinout same as the OMNIBUSF3. 

Nevertheless it is a working release and can be used by advanced users only. There is no GUI available at this point to change important settings like matching of the remote controll channel. Also the PID values, Filter- and Motorsetting need to be changed manually in the code itself. You can take a look at the 4Inch.h or 5Inch.h files to get an impression how to change some the variables. Maybe there will be a GUI in the future to allow making theses configurations easy. However, for now it is not the case.

As  as  summary. I'm publishing this codebase only for the purpose of reference and hope it is providing valid information for someone who is interessted and is searching some answers. You can also feel free to contact me.


