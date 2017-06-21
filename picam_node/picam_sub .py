#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD)
#
#  file      @picam_pub.py
#  authors   Ben Fogarty <ben.fogarty@live.com>
#            Better Robotics <ben.fogarty@live.com>
#  copyright Copyright (c) 2017, Better Robotics, Inc., All rights reserved.
#            Copyright (c) 2017, Better Robotics Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that
# the following conditions are met:
#  * Redistributions of source code must retain the above copyright notice, this list of conditions and the
#    following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
#    following disclaimer in the documentation and/or other materials provided with the distribution.
#  * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
#    products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
# RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
# DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
# OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import numpy as np
import cv2, rospy
from sensor_msgs.msg import Image
 

def callback(data):
    
    # create int values from sting
    np_arr = np.fromstring(data.data, np.uint8)

    # convert data from .jpg encoded image to cv2 format (np.uint8)
    image = cv2.imdecode(np_arr, 1)
    cv2.imshow('image', image)

    # Split image into Blue Green Red inputs 
    b, g, r = cv2.split(image)

    # create horizontal stack
    stack_raw = np.hstack((b, g, r))
    cv2.imshow("RAW BGR", stack_raw)

    # show images and delay for 10ms
    cv2.waitKey(10)
    

if __name__ == '__main__':
    rospy.init_node('image_viewer', anonymous=True)
    rospy.Subscriber("/image", Image, callback)
    rospy.spin()
