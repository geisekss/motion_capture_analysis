# Source code for _"A multi-sensor human gait dataset captured through an optical system and inertial measurement units"_

* The setup adopted to the motion capture system Qualisys, with 18 infra-red cameras, is presented in the picture `setup.png`

* The Arduino firmware source code, used in the ESP8266 to acquire accelerometer data from an InvenSense MPU-9250 six-axis IMU and send them by OSC protocol, is located in the `Arduino_firmware` folder.

* The Android application source code, used to acquire accelerometer data from a Nexus 5 smartphone, is located in the repository [**data_collection_app**](https://github.com/geisekss/data_collection_app.git).

* The integration API, used to synchronize data from the Qualisys system and the ESP8266, is located in the `integration_API` folder.

* The source code to perform the data processing, described in the paper _"{A multi-sensor human gait dataset captured through an optical system and inertial measurement units"_, is located in the `data_analysis` folder.

	* First, it is necessary to run the Matlab code `data_analysis\extract_walkSection_trajectories.matlab` to extract the walking sections from the trajectories, and store the first and last frames of these sections.

	* Through these first and last frames stored by the previous code, it is possible to run the Python code `data_analysis\extract_walkSection_accelerations.py` to extract the walking sections from the accelerations.  

