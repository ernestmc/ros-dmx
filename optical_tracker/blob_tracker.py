import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Point


class BlobTracker(object):

    def __init__(self, video_id):
        self.capture = cv2.VideoCapture(video_id)
        self.key_color = None
        self.latest_mask = None
        self.latest_image = None
        # Set up the detector.
        self.detector = self.create_detector()
        self.MAIN_WINDOW = "Main window"
        self.MASK_WINDOW = "Mask window"
        cv2.namedWindow(self.MAIN_WINDOW)
        cv2.namedWindow(self.MASK_WINDOW)
        cv2.setMouseCallback(self.MAIN_WINDOW, self.on_mouse_event)
        self.rate = rospy.Rate(30)
        self.publisher = rospy.Publisher("~/position", Point, queue_size=10)

    def run(self):
        running = True
        while not rospy.is_shutdown():
            ret, frame = self.capture.read()
            image = cv2.GaussianBlur(frame,(5,5),0)
            self.update_image(image)
            blob = None
            if self.key_color is not None:
                mask = self.filter_image(self.latest_image_hsv, self.key_color, (30, 50, 50))
                mask = cv2.dilate(mask, None, 18)
                mask = cv2.erode(mask, None, 10)
                blob = self.detect_blob(mask)
                #self.update_mask(mask)
            if blob is not None:
                self.publish_position(blob[0], blob[1], blob[2])
            self.rate.sleep()
            # Listen for ESC key
            c = cv2.waitKey(10) % 0x100
            if c == 27:
                running = False

    def update_image(self, image):
        self.latest_image = image
        self.latest_image_hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        cv2.imshow(self.MAIN_WINDOW, image)

    def update_mask(self, mask_image):
        self.latest_mask = mask_image
        cv2.imshow(self.MASK_WINDOW, mask_image)

    def on_mouse_event(self, event, x, y, w, h):
        if event == cv2.EVENT_LBUTTONUP:
            #print event, x, y, w, h
            color = self.latest_image_hsv[y, x]
            self.key_color = color

    def filter_image(self, image, color, similarity):
        low_color = cv2.addWeighted(color, 1, similarity, -1, 0, dtype=cv2.CV_8U)
        high_color = cv2.addWeighted(color, 1, similarity, 1, 0, dtype=cv2.CV_8U)
        mask = cv2.inRange(image, low_color, high_color)
        mask = cv2.addWeighted(255, 1, mask, -1, 0, dtype=cv2.CV_8U)
        return mask

    def detect_blob(self, image):
        # Detect blobs.
        pimage = image
        dy, dx = pimage.shape
        cv2.rectangle(pimage, (0, 0), (dx-1, 10), (255, 255, 255), -1)
        cv2.rectangle(pimage, (0, dy-10), (dx-1, dy-1), (255, 255, 255), -1)
        cv2.rectangle(pimage, (dx-10, 0), (dx-1, dy-1), (255, 255, 255), -1)
        cv2.rectangle(pimage, (0, 0), (10, dy-1), (255, 255, 255), -1)
        keypoints = self.detector.detect(pimage)
        blob = None
        for k in keypoints:
            if blob is None or k.size > blob.size:
                blob = k
        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        #im_with_keypoints = cv2.drawKeypoints(image, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        center = None
        radius = None
        if blob is not None:
            center = (int(blob.pt[0]), int(blob.pt[1]))
            radius = int(blob.size / 2.0)
            cv2.circle(pimage, center, radius, (255, 255, 255), 3)
        im_with_keypoints = pimage
        cv2.imshow(self.MASK_WINDOW, im_with_keypoints)
        if blob is None:
            return None
        return [blob.pt[0], blob.pt[1], blob.size]

    def create_detector(self):
        # Setup SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()
        # Change thresholds
        #params.minThreshold = 10
        #params.maxThreshold = 200
        # Filter by Area.
        params.filterByArea = True
        params.minArea = 500
        params.maxArea = 200000
        # Filter by Circularity
        params.filterByCircularity = False
        params.minCircularity = 0.001
        # Filter by Convexity
        params.filterByConvexity = False
        params.minConvexity = 0.7
        # Filter by Inertia
        params.filterByInertia = False
        params.minInertiaRatio = 0.01
        return cv2.SimpleBlobDetector(params)

    def publish_position(self, x, y, z):
        self.publisher.publish(Point(x, y, z))


if __name__ == '__main__':
  rospy.init_node("blob_tracker")
  node = BlobTracker(0)
  node.run()