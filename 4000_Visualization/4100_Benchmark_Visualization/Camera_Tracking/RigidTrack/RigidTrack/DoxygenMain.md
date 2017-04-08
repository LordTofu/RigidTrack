 \mainpage Rigid Track Doxygen Documentation
 
 \section intro_sec Introduction

 Rigid Track is a software that provides, combined with an OptiTrack camera, the pose 
 estimation of one object in three dimensional space. This is achieved with only one camera 
 in combination with reflective markers. Those are attached to the object ought to be tracked. 
 The accuracy in the range of millimeters and the high update rate of 100 Hz enable use cases for 
 fast and agile objects. The main application is navigation for drones that rely on high precision 
 position data. Where GPS is not available, e.g. indoors or due to a lacking GPS receiver, this 
 setup substitutes for it. Another use case is the pure pose logging when the drone does not 
 depend on the position, e.g. when it is remote piloted by hand. While this setup contains one 
 OptiTrack Flex 3 camera, every other model of OptiTrack should work, despite not tested. With 
 better camera models, e.g. the Prime Series, even outdoor usage is possible. When the capabilities 
 are not sufficient please refer to OptiTracks Software Motive. But keep in mind that this solution
 needs at least 3 cameras as Rigid Track works with only one. 


 \section softInstall_sec Rigid Track Installation
  
  Start the RigidTrack_setup.exe from the enclosed SD card and follow the instructions given in the 
  installation assistant. Default parameters like installation directory or shortcuts to be created 
  can be chosen. But normally clicking Next and keeping the default values should be sufficient. When 
  the installation is completed a shortcut in the start menu and the desktop can be used to start 
  Rigid Track. The program is then successfully installed in C:/Program Files (x86)/TU Munich FSD/Rigid Track.
 

 \section source_code Source Code
 The most interesting file for you is main.cpp. It contains the relevant functions for pose estimation.
 Camera calibration and other functional aspects are also implemented there.
 The GUI program code is found in RigidTrack.cpp. communication.cpp deals only with communication from main.cpp to 
 the GUI. 
 
