from __future__ import print_function

import numpy as np
import cv2

import cv2
import threading
import numpy as np


class CSI_Camera:

    def __init__(self):
        self.video_capture = None
        self.frame = None
        self.grabbed = False
        self.read_thread = None
        self.read_lock = threading.Lock()
        self.running = False

    def open(self, gstreamer_pipeline_string):
        try:
            self.video_capture = cv2.VideoCapture(
                gstreamer_pipeline_string, cv2.CAP_GSTREAMER
            )
            self.grabbed, self.frame = self.video_capture.read()

        except RuntimeError:
            self.video_capture = None
            print("Unable to open camera")
            print("Pipeline: " + gstreamer_pipeline_string)

    def start(self):
        if self.running:
            print('Video capturing is already running')
            return None
        # create a thread to read the camera image
        if self.video_capture != None:
            self.running = True
            self.read_thread = threading.Thread(target=self.updateCamera)
            self.read_thread.start()
        return self

    def stop(self):
        self.running = False
        # Kill the thread
        self.read_thread.join()
        self.read_thread = None

    def updateCamera(self):
        # This is the thread to read images from the camera
        while self.running:
            try:
                grabbed, frame = self.video_capture.read()
                with self.read_lock:
                    self.grabbed = grabbed
                    self.frame = frame
            except RuntimeError:
                print("Could not read image from camera")
        # FIX ME - stop and cleanup thread
        # Something bad happened

    def read(self):
        with self.read_lock:
            frame = self.frame.copy()
            grabbed = self.grabbed
        return grabbed, frame

    def release(self):
        if self.video_capture != None:
            self.video_capture.release()
            self.video_capture = None
        # Now kill the thread
        if self.read_thread != None:
            self.read_thread.join()


def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1920,
    capture_height=1080,
    display_width=1920,
    display_height=1080,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )


def run_cameras():
    window_title = "Dual CSI Cameras"
    left_camera = CSI_Camera()
    left_camera.open(
        gstreamer_pipeline(
            sensor_id=0,
            capture_width=640,
            capture_height=480,
            flip_method=0,
            display_width=640,
            display_height=480,
        )
    )
    left_camera.start()

    right_camera = CSI_Camera()
    right_camera.open(
        gstreamer_pipeline(
            sensor_id=1,
            capture_width=640,
            capture_height=480,
            flip_method=0,
            display_width=640,
            display_height=480,
        )
    )
    right_camera.start()

    if left_camera.video_capture.isOpened() and right_camera.video_capture.isOpened():

        cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)

        try:
            while True:
                _, left_image = left_camera.read()
                _, right_image = right_camera.read()
                camera_images = np.hstack((left_image, right_image))
                if cv2.getWindowProperty(window_title, cv2.WND_PROP_AUTOSIZE) >= 0:
                    cv2.imshow(window_title, camera_images)
                    print('taq')
                else:
                    break
                keyCode = cv2.waitKey(30) & 0xFF
                if keyCode == 27:
                    break
                    
                cv_file = cv2.FileStorage()
                cv_file.open('EpipolarGeometryAndStereoVision/EpipolarGeometryAndStereoVision\stereoMap.xml', cv2.FILE_STORAGE_READ)
                stereoMapL_x = cv_file.getNode('stereoMapL_x').mat()
                stereoMapL_y = cv_file.getNode('stereoMapL_y').mat()
                stereoMapR_x = cv_file.getNode('stereoMapR_x').mat()
                stereoMapR_y = cv_file.getNode('stereoMapR_y').mat()

                imgL = left_image
                imgL = cv2.remap(imgL, stereoMapL_x, stereoMapL_y, 0, 0, 0)
                imgR = right_image
                imgR = cv2.remap(imgR, stereoMapR_x, stereoMapR_y,  0, 0, 0)

                # Setting parameters for StereoSGBM algorithm
                minDisparity = 0
                numDisparities = 21
                blockSize = 12
                disp12MaxDiff = 1
                uniquenessRatio = 10
                speckleWindowSize = 10
                speckleRange = 10

                # Creating an object of StereoSGBM algorithm
                stereo = cv2.StereoSGBM_create(minDisparity=minDisparity,
                                            numDisparities=numDisparities,
                                            blockSize=blockSize,
                                            disp12MaxDiff=disp12MaxDiff,
                                            uniquenessRatio=uniquenessRatio,
                                            speckleWindowSize=speckleWindowSize,
                                            speckleRange=speckleRange
                                            )

                # Calculating disparith using the StereoSGBM algorithm
                disp = stereo.compute(imgL, imgR).astype(np.float32)

                # Calculating disparith using the StereoSGBM algorithm
                disp = cv2.normalize(disp, 0, 255, cv2.NORM_MINMAX)

                # Displaying the disparity map
                cv2.imshow("disparity", disp)
                cv2.imshow(window_title, camera_images)
                cv2.waitKey(0)


        finally:

            left_camera.stop()
            left_camera.release()
            right_camera.stop()
            right_camera.release()
        cv2.destroyAllWindows()
    else:
        print("Error: Unable to open both cameras")
        left_camera.stop()
        left_camera.release()
        right_camera.stop()
        right_camera.release()

def main():
    run_cameras()
    
if __name__ == '__main__':
    main()
    cv2.destroyAllWindows()
