import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
import cv2 as cv
import time
import sys



class FaceRecognition(Node):

    def __init__(self):
        super().__init__('face_recognition')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.cap = cv.VideoCapture(int(sys.argv[1]))
        self.sift = cv.SIFT_create()

        self.start = time.time()
        self.frames = 0
        self.num_recognitions = 0

        self.img1 = cv.imread('Resources/Persona/photo_1.png')
        self.img2 = cv.imread('Resources/Persona/photo_2.png')
        self.img3 = cv.imread('Resources/Persona/photo_3.png')
        self.img4 = cv.imread('Resources/Persona/photo_4.png')
        self.img5 = cv.imread('Resources/Persona/photo_5.png')

        self.kp1, self.des1 = self.sift.detectAndCompute(self.img1, None)
        self.kp2, self.des2 = self.sift.detectAndCompute(self.img2, None)
        self.kp3, self.des3 = self.sift.detectAndCompute(self.img3, None)
        self.kp4, self.des4 = self.sift.detectAndCompute(self.img4, None)
        self.kp5, self.des5 = self.sift.detectAndCompute(self.img5, None)

        #Flann matcher
        FLANN_INDEX_KDTREE = 1
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=50)
        self.flann = cv.FlannBasedMatcher(index_params, search_params)

    def timer_callback(self):

        ret, frame = self.cap.read()

        if ret:

            self.frames = self.frames + 1

            kp6, des6 = self.sift.detectAndCompute(frame, None)

            matches_photo1 = self.flann.knnMatch(self.des1, des6, k=2)
            matches_photo2 = self.flann.knnMatch(self.des2, des6, k=2)
            matches_photo3 = self.flann.knnMatch(self.des3, des6, k=2)
            matches_photo4 = self.flann.knnMatch(self.des4, des6, k=2)
            matches_photo5 = self.flann.knnMatch(self.des5, des6, k=2)

            good_photo1 = []
            good_photo2 = []
            good_photo3 = []
            good_photo4 = []
            good_photo5 = []

            for i, (m, n) in enumerate(matches_photo1):
                if m.distance < 0.75*n.distance:
                    #matchesMask[i] = [1, 0]
                    good_photo1.append(m)

            for i, (m, n) in enumerate(matches_photo2):
                if m.distance < 0.75*n.distance:
                    good_photo2.append(m)

            for i, (m, n) in enumerate(matches_photo3):
                if m.distance < 0.75*n.distance:
                    good_photo3.append(m)
                    
            for i, (m, n) in enumerate(matches_photo4):
                if m.distance < 0.75*n.distance:
                    good_photo4.append(m)

            for i, (m, n) in enumerate(matches_photo5):
                if m.distance < 0.75*n.distance:
                    good_photo5.append(m)              

            if len(good_photo1) > 10:
                print("Se encontr?? coincidencia foto 1")
                #cv.putText(frame, "Match", (225, 50),
                        #cv.FONT_HERSHEY_TRIPLEX, 2, (0, 255, 0), 2)
                self.num_recognitions = self.num_recognitions + 1

            elif len(good_photo2) > 10:
                print("Se encontr?? coincidencia foto 2")
                #cv.putText(frame, "Match", (225, 50),
                        #cv.FONT_HERSHEY_TRIPLEX, 2, (0, 255, 0), 2)
                self.num_recognitions = self.num_recognitions + 1

            elif len(good_photo3) > 10:
                print("Se encontr?? coincidencia foto 3")
                #cv.putText(frame, "Match", (225, 50),
                        #cv.FONT_HERSHEY_TRIPLEX, 2, (0, 255, 0), 2)
                self.num_recognitions = self.num_recognitions + 1
                
            elif len(good_photo4) > 10:
                print("Se encontr?? coincidencia foto 4")
                #cv.putText(frame, "Match", (225, 50),
                        #cv.FONT_HERSHEY_TRIPLEX, 2, (0, 255, 0), 2)
                self.num_recognitions = self.num_recognitions + 1
                
            elif len(good_photo5) > 10:
                print("Se encontr?? coincidencia foto 5")
                #cv.putText(frame, "Match", (225, 50),
                        #cv.FONT_HERSHEY_TRIPLEX, 2, (0, 255, 0), 2)
                self.num_recognitions = self.num_recognitions + 1

            current_time = time.time()
            if current_time - self.start >= 3:
                self.start = current_time
                p = self.num_recognitions / self.frames
                print(self.num_recognitions, self.frames)
                msg = String()
                if p >= 0.33:
                    msg.data = 'YES'
                else:
                    msg.data = 'NO'
                self.publisher_.publish(msg)
                self.get_logger().info('Publishing: "%s"' % msg.data)

                self.frames = 0
                self.num_recognitions = 0

            #cv.imshow("camera", frame)
            #cv.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    publisher = FaceRecognition()

    rclpy.spin(publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
