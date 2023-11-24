# esp32-flight-data-recorder
a simple flight data recorder for use with the esp32 board. using lilygo amoled t-display esp32-s3, a kx134 accelerometer, and a MPL3115A2  for barometric presure

This was born out of wanting to know the particulars of a recent model rocket launch with my daughter. I'm not sure about the component g-force capabilites and a quick google search did not reveal any easy information.

The code implements a simple state machine, boot->init->ground hold->armed->in flight->landed. 

to move from hold to armed, you press the left/lower button.
to clear the ground level/pressure and min/max accounting, press the right/upper button.
to reset back to hold, press and hold the right button, then press the left button. (note, this will dump the last flight statistics to serial (saved via littleFS)

the project utilizes an amoled 240x536 display for output.

Hardware used:
(esp3-s3 microcontroller w/ screen) https://www.lilygo.cc/products/t-display-s3-amoled
(64G 3 axis accelerometer - i2c) https://www.sparkfun.com/products/17589
(precision barometer - i2c) https://www.adafruit.com/product/1893