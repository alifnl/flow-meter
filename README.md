This code is available to be built in arduino IDE or platformIO.
Most of the framework is ESP-idf with a little arduino syntax.

This flow meter consists:
1. http web server to control the flow meter without entering the site.
2. ota web server to upload spiffs or firmware without direct contact to the device.
3. capture and monitor volume passed the flow meter and save and log it to csv file.
4. there is also direct UI (using lcd 16x2 and 4 buttons) to set gain and set other parameters.

This the web display

![image](https://github.com/user-attachments/assets/f696eeb7-df71-49bb-b33e-13e3bc40a5be)
