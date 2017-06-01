# PiCamera_ROS_MultiThread
This Class has been written around the PiCamera library it provides easy access to camera parameters and enables the camera to run as a seperate thread. This provides uninterrupted video streaming at over 200fps **resolution dependant**. 

This Class also provides the option to enable a ROS publisher that will provide the stream to rostopic /image
and subcribes to /camera_parameters to change: awb mode, exposure mode, gain, iso and brightnes. 

```bash
rostopic pub --once /camera_parameters picam_node/Parameters  '"auto"' '"auto"' '1.0' '1.0' '800' '50'
```

Please see the PiCamera Documention for possible settings.

# To use this class from another script

Enable the class
```bash
vs = picam.PiVideoStream(resolution=(320, 240), framerate=20, ROS=False)
```

Start the stream:
```bash
vs.start()
```

**The camera will take 10 seconds to warm up, this allows the inaccesable analog and digital gains to rise to nice level before we change parameters that lock these values, the camera will let you know when it's read to use.**


To get the most current image we call :
```bash
vs.read()
```

To stop the steam we call:
```bash
vs.stop()
```
