# Source code for _"A multi-sensor human gait dataset captured through an optical system and inertial measurement units"_

* The setup adopted to the motion capture system Qualisys, with 18 infra-red cameras, is presented in the picture `setup.png`

* The Arduino firmware source code, used in the ESP8266 to acquire accelerometer data from an InvenSense MPU-9250 six-axis IMU and send them by OSC protocol, is located in the `Arduino_firmware` folder.

* The Android application source code, used to acquire accelerometer data from a Nexus 5 smartphone, is located in the repository [**data_collection_app**](https://github.com/geisekss/data_collection_app.git).

* The integration API, used to synchronize data from the Qualisys system and the ESP8266, is located in the `integration_API` folder.

* The source code to perform the data processing, described in the paper _"{A multi-sensor human gait dataset captured through an optical system and inertial measurement units"_, is located in the `data_analysis` folder.

	* First, it is necessary to run the Matlab code `data_analysis\extract_walkSection_trajectories.matlab` to extract the walking sections from the trajectories, and store the first and last frames of these sections.

	* Through these first and last frames stored by the previous code, it is possible to run the Python code `data_analysis\extract_walkSection_accelerations.py` to extract the walking sections from the accelerations.  

	* Processed data (folder `processed_data`) can be loaded using Mocap Toolbox by downloading it in the [official website](https://www.jyu.fi/hytk/fi/laitokset/mutku/en/research/materials/mocaptoolbox), and executing the following sample code (remember to include the Mocap Toolbox folder and its subfolders in Matlab path):

	```matlab
	load mcdemodata
	folder = 'processed_data/';
	files_qtm = dir(strcat(folder, user, '/*_qtm_walk.mat'));
	for i=1:length(files_qtm)
		capture_name = files_qtm(i).name
		capture_data = load(strcat(files_qtm(i).folder, '/', capture_name));
		capture_name = fieldnames(capture_data);	
		markers = capture_data.(capture_name{1}).markerName;
	```
