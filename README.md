# PiCamera_ROS_MultiThred
This Class has been written around the PiCamera library it provides easy access to camera parameters and enables the camera to run as a seperate thread. This provides uninterrupted video streaming at over 200fps **resolution dependant**. 

This Class also provides the option to enable a ROS publisher that will provide the stream to rostopic /image
and subcribes to /camera_parameters to change: awb mode, exposure mode, gain, iso and brightnes. 

<info/>
rostopic pub --once /camera_parameters picam_node/Parameters  '"auto"' '"auto"' '1.0' '1.0' '800' '50'
<info>

Please see the PiCamera Documention for possible settings.
