import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from matplotlib import pyplot as plt


def nothing(x):
    pass

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.declare_parameter('image_file', 'image.PNG')
        self.timer = self.create_timer(0.2, self.timer_callback)
        self.pub = self.create_publisher(Image, 'processed_image', 10)
        file_loc = self.get_parameter('image_file').get_parameter_value().string_value
        print("Loading image from " + file_loc)
        self.image = cv2.imread(file_loc)
        self.br = CvBridge()

        cv2.namedWindow('Original image', cv2.WINDOW_AUTOSIZE)


        cv2.createTrackbar('HMin','Original image',0,179,nothing)
        cv2.createTrackbar('SMin','Original image',0,255,nothing)
        cv2.createTrackbar('VMin','Original image',0,255,nothing)
        cv2.createTrackbar('HMax','Original image',0,179,nothing)
        cv2.createTrackbar('SMax','Original image',0,255,nothing)
        cv2.createTrackbar('VMax','Original image',0,255,nothing)



        # Set default value for MAX HSV trackbars.
        cv2.setTrackbarPos('HMin', 'Original image', 0)
        cv2.setTrackbarPos('SMin', 'Original image', 0)
        cv2.setTrackbarPos('VMin', 'Original image', 0)
        cv2.setTrackbarPos('HMax', 'Original image', 179)
        cv2.setTrackbarPos('SMax', 'Original image', 255)
        cv2.setTrackbarPos('VMax', 'Original image', 255)


        self.hMin = self.sMin = self.vMin = self.hMax = self.sMax = self.vMax = 0
        self.phMin = self.psMin = self.pvMin = self.phMax = self.psMax = self.pvMax = 0

    def timer_callback(self):
        ## process stuff
        print('Doing stuff')
        cv2.imshow("Original image", self.image)
        cv2.waitKey(1)
        # cv2.imshow("Processed image", result)
        self.color_segment(self.image)
        result = self.image
        try:
            image_msg = self.br.cv2_to_imgmsg(cv2.cvtColor(result, cv2.COLOR_BGR2RGB), "rgb8")
        except CvBridgeError:
            print('Cannot convert image to msg')
        self.pub.publish(image_msg)

    def image_filters(self, cv_image):
        kernel_avg = np.ones((9,9),np.float32)/81
        img_average = cv2.filter2D(cv_image,-1,kernel_avg)
        cv2.imshow("Average filter", img_average)
        img_blur = cv2.blur(cv_image, (9,9))
        img_gauss = cv2.GaussianBlur(cv_image, (9,9), 0)
        img_median = cv2.medianBlur(cv_image, 9)
        cv2.imshow("Blur",img_blur)
        cv2.imshow("Gaussian filter",img_gauss)
        cv2.imshow("Median filter",img_median)
        cv2.waitKey(1)

    def thresholding(self, cv_image):
        img_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        ret,thresh1 = cv2.threshold(img_gray,127,255,cv2.THRESH_BINARY)
        ret,thresh2 = cv2.threshold(img_gray,127,255,cv2.THRESH_BINARY_INV)
        ret,thresh3 = cv2.threshold(img_gray,127,255,cv2.THRESH_TRUNC)
        ret,thresh4 = cv2.threshold(img_gray,127,255,cv2.THRESH_TOZERO)
        ret,thresh5 = cv2.threshold(img_gray,127,255,cv2.THRESH_TOZERO_INV)
        
        titles = ['Original Image','BINARY','BINARY_INV','TRUNC','TOZERO','TOZERO_INV']
        images = [cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB), thresh1, thresh2, thresh3, thresh4, thresh5]
        
        for i in range(6):
            plt.subplot(2,3,i+1),plt.imshow(images[i],'gray',vmin=0,vmax=255)
            plt.title(titles[i])
            plt.xticks([]),plt.yticks([])

        plt.show()

    def color_segment(self, cv_image):
        hMin = cv2.getTrackbarPos('HMin','Original image')
        sMin = cv2.getTrackbarPos('SMin','Original image')
        vMin = cv2.getTrackbarPos('VMin','Original image')

        hMax = cv2.getTrackbarPos('HMax','Original image')
        sMax = cv2.getTrackbarPos('SMax','Original image')
        vMax = cv2.getTrackbarPos('VMax','Original image')


        lower = np.array([hMin, sMin, vMin])
        upper = np.array([hMax, sMax, vMax])


        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        cv2.imshow("Color segmentation", mask)
        cv2.waitKey(1)


    def edges(self, cv_image):
        img_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        grad_x = cv2.Sobel(img_gray, cv2.CV_64F, 1, 0)
        grad_y = cv2.Sobel(img_gray, cv2.CV_64F, 0, 1)
        grad = np.sqrt(grad_x**2 + grad_y**2)
        grad_norm = (grad * 255 / grad.max()).astype(np.uint8)
        cv2.imshow('Sobel gradient', grad_norm)
        ret,thresh1 = cv2.threshold(grad_norm,50,255,cv2.THRESH_BINARY)
        cv2.imshow('Gradient thresholded - Edges', thresh1)
        edges = cv2.Canny(img_gray,100,200)
        cv2.imshow('Canny edge detection', edges)

    def lines(self, cv_image):
        cdst = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
        cdstP = np.copy(cdst)
        
        lines = cv2.HoughLines(cv_image, 1, np.pi / 180, 100, None, 0, 0)
        
        if lines is not None:
            for i in range(0, len(lines)):
                rho = lines[i][0][0]
                theta = lines[i][0][1]
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
                pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
                cv2.line(cdst, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)
        
        
        linesP = cv2.HoughLinesP(cv_image, 1, np.pi / 180, 50, None, 50, 10)
        
        if linesP is not None:
            for i in range(0, len(linesP)):
                l = linesP[i][0]
                cv2.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)
        
        cv2.imshow("Source", cv_image)
        cv2.imshow("Detected Lines (in red) - Standard Hough Line Transform", cdst)
        cv2.imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP)
        cv2.waitKey(1)

    def circles(self, cv_image):
        img_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)   
        rows = img_gray.shape[0]
        circles = cv2.HoughCircles(img_gray, cv2.HOUGH_GRADIENT, 1, 20,
                                param1=150, param2=20,
                                minRadius=1, maxRadius=30)
        
        
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                center = (i[0], i[1])
                # circle center
                cv2.circle(cv_image, center, 1, (0, 100, 100), 3)
                # circle outline
                radius = i[2]
                cv2.circle(cv_image, center, radius, (0, 255, 0), 3)
        
        
        cv2.imshow("Detected circles", cv_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    image_processor = ImageProcessor()
    rclpy.spin(image_processor)
    image_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



