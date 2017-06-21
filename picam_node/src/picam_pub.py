#!/usr/bin/env python

import sys, time, os
ROS_FLAG = 0
try:
    import rospy
    from sensor_msgs.msg import Image
    from picam_node.msg import Parameters
    ROS_FLAG = 1
except ImportError:
    os.system('clear')
    print("\n\n\tROS is not installed! - functionality disabled")
    time.sleep(3)

try:
    from picamera.array import PiRGBArray
    from picamera import PiCamera
    from threading import Thread
    import numpy as np
    import cv2
except ImportError:
    os.system('clear')
    print( "\n\n\t\t\t[WARNING!!!]\n\n\tYou must install PiCamera and Numpy for bare operations\n\t    NOTE: this package is designed for use with a Pi!\n\n")
    time.sleep(6)
    os.system('clear')
    print( "\n\n\t\tSort your shit GoodBye!!")
    time.sleep(2)
    os.system('clear')
    sys.exit()


class PiVideoStream:
    def __init__(self, resolution=(320, 240), framerate=20, ROS=False, cal=False):
        # initialize the camera and stream
        self.camera = PiCamera()
        self.camera.resolution = resolution
        self.camera.framerate = framerate
        self.camera.awb_mode = 'auto'
        self.camera.exposure_mode = 'auto'

        # As a safey in case the awb_mode is set to off before these values are set will break the camera driver
        self.camera.awb_gains = (1.0, 1.0) 
        
        #self.camera.rotation = 90
        self.camera.vflip = False
        self.camera.hflip = False
        self.rawCapture = PiRGBArray(self.camera, size=resolution)
        self.stream = self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True)

        # initialize the frame and the variable used to indicate
        # if the thread should be stopped
        self.frame = None
        self.stopped = False

        # Set the vaiables used to control the White balance
        self.cali = cal
        self.rGain = 1.0
        self.bGain = 1.0
        self.str = 'EMPTY'

        # ROS
        self.ROS = ROS
        if self.ROS == True and ROS_FLAG:
            rospy.init_node('PiCamera', anonymous=True)
            PiVideoStream.start_ros_node(self)
            if(self.cali):
                rospy.loginfo("Custom AWB mode being used!")
                self.camera.awb_mode = 'off' # make sure the camera.awb_gains has been set


    def custom_awb(self):
        # get r,g,b values from the image
        b,g,r = cv2.split(self.frame)
        b = np.mean(b)
        g = np.mean(g)
        r = np.mean(r)

        # Adjust R and B relative to G, but only if they're significantly
        # different (delta +/- 2)
        if( abs(r - g) > 4):
            if (r > g):
                if (self.rGain > 0.5):
                    self.rGain -= 0.01
            else:
                if (self.rGain < 7.98):
                    self.rGain += 0.01
        if (abs(b - g) > 4):
            if (b > g):
                if (self.bGain > 0.5):
                    self.bGain -= 0.01
            else:
                if (self.bGain < 7.98):
                    self.bGain += 0.01
        if g < 95:
            if(self.camera.brightness <= 99):
                self.camera.brightness += 1
        elif g > 105:
            if(self.camera.brightness >= 2):
                self.camera.brightness -= 1

        camera.awb_gains = (self.rGain, self.bGain)
        self.str = 'rGain: %f\tbGain: %f\tBrightness %i R: %i, G: %i, B: %i\n' % (self.rGain, self.bGain, self.camera.brightness, r, g, b)
        rospy.loginfo(self.str)


    def debug(self):
        # debug info
       return self.str


    def start(self):
        # start the thread to read frames from the video stream
        Thread(target=self.update).start()
        if self.ROS == True:
            rospy.loginfo("Waiting 10s for camera to warm up.")
        else:
            print("Waiting 10s for camera to warm up.")
        time.sleep(10) ## wait for camera to warm up !!

        if self.ROS == True:
            rospy.loginfo("Done! - Camera is read to use")
        else:
            print("Done! - Camera is read to use")
        return self ## do we need this?


    def update(self):
        # keep looping infinitely until the thread is stopped
        for f in self.stream:

            # grab the frame from the stream and clear the stream in
            # preparation for the next frame
            self.frame = f.array

            # if we init with options act on them
            if(self.ROS == True):
                PiVideoStream.pub_image(self)

            if(self.cali == True):
                PiVideoStream.custom_awb(self)

            # if the thread indicator variable is set, stop the thread
            # and resource camera resources
            if self.stopped:
                self.stream.close()
                self.rawCapture.close()
                self.camera.close()
                return
            #time.sleep(1/(self.camera.framerate - 2)) ## time to sleep dependant on framrate
            self.rawCapture.truncate(0)


    def read(self):
        # return the frame most recently read
        return self.frame


    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True
        if self.ROS == True:
            rospy.signal_shutdown("PiCam_thread stopped!")
        else:
            print("PiCam_thread stopped!")


    def set_params(self, data):

        if(data.red_gain >= 1.0 and data.red_gain <= 8.0):
            self.rGain = data.red_gain

        if(data.blue_gain >= 1.0 and data.blue_gain <= 8.0):
            self.bGain = data.blue_gain

        self.camera.brightness = data.brightness
        self.camera.iso =  data.iso
        self.camera.awb_mode = data.awb_mode
        self.camera.exposure_mode = data.exposure_mode
        self.camera.awb_gains = (self.rGain, self.bGain)
        self.str = '\n\tawb_mode: %s\n\texposure_mode: %s\n\trGain: %d  bGain: %d\n\tiso: %i\n\tBrightness %i' % (self.camera.awb_mode, self.camera.exposure_mode, self.rGain, self.bGain, self.camera.iso, self.camera.brightness)
        rospy.loginfo(self.str)


    def start_ros_node(self):

        self.videoRaw = rospy.Publisher('/image', Image, queue_size=1)
        self.cam_params = rospy.Subscriber("camera_parameters", Parameters, self.set_params)
        str1 = "Camera Initalized"
        str2 = "Camera Frame: %i x %i" % (self.camera.resolution[0], self.camera.resolution[1])
        str3 = "Camera framerate: %i" % (self.camera.framerate)
        rospy.loginfo(str1)
        rospy.loginfo(str2)
        rospy.loginfo(str3)

        try:
            # try to load the yaml file
            FileName = "NoIR_PiCamera_V2.yaml"
            with open(fname, "w") as f:
                yaml.dump(data, f)
        except NameError:
            str0 = "No yaml file found...\nThe file should be in the same directory as picam_pub.py\nThe file must be called 'NoIR_PiCamera_V2.yaml' "
            rospy.logwarn(str0)


    def pub_image(self):

        if rospy.is_shutdown():
            str0 = "ROS not initialised"
            rospy.logwarn(str0)
            PiVideoStream.start_ros_node(self)

        else:
            # Create image object
            msg = Image()
            msg.header.frame_id = 'base_link'
            msg.header.stamp = rospy.Time.now()

            # Encode image 
            msg.data = np.array(cv2.imencode('.jpg', self.frame)[1]).tostring()

            # Publish new image
            self.videoRaw.publish(msg)


if __name__ == '__main__':

    vs = PiVideoStream(framerate=20, ROS=False, cal=False).start() # start picamera using defaults with ROS node

    while not rospy.is_shutdown():
        time.sleep(1) ## sleep for 1 second
    vs.stop()
