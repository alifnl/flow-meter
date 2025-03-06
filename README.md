This code is available to be built in arduino IDE or platformIO.
Most of the framework is ESP-idf with a little arduino syntax.

This flow meter consists:
1. http web server to control the flow meter without entering the site.
2. ota web server to upload spiffs or firmware without direct contact to the device.
3. capture and monitor volume passed the flow meter and save and log it to csv file.
4. there is also direct UI (using lcd 16x2 and 4 buttons) to set gain and set other parameters.

This the web display

![image](https://github.com/user-attachments/assets/f696eeb7-df71-49bb-b33e-13e3bc40a5be)
![image](https://github.com/user-attachments/assets/f704973a-0631-4dea-a0f0-42b825acb3ce)

I was using flowmeter FY-B10 or YF-B10. 
To get accurate value, I was using high flow gain and low flow gain, since the sensors minimum flow is 1L/min which is around 33mL/s.

Volume measurement scenario:
1. I collect how many pulses collected in one second.
2. If the pulse is exceeding 33mL/s it will be calculated using low flow gain.
3. in my case, low flow gain is 18.000 and high flow gain: 2.08
4. ofcourse there will be error (tolerance), I got 13% error which is quite enough for my application.
5. You can tune the gain by getting how many pulses is gathered in certain time and then give a fix amount of water.
6. after that you can use multivariable equation and solve it using chatGPT.
7. i.e if I got high flow pulse 800 pulse and low flow pulse 10 and the fixed volume is 2L. Then, you make this equation 800.x + 10.y = 2000. then gathered another equation by conducting same experiment in point 5.
8. after you solve the equation, keep them as parameter.
