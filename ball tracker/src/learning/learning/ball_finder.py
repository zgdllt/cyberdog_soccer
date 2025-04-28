import rclpy
import ball_tracker
import barrier_move
import camera
import cv2
def judge(img):
    hsv= cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    thresh= cv2.inRange(hsv, (30, 70, 0), (70, 255, 255))
    _, contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    try:
        max_contour = max(contours, key=cv2.contourArea)
        return cv2.contourArea(max_contour)>2500
    except:
        return False
def main(args=None):
    rclpy.init(args=args)
    AvoidNode=barrier_move.DogAvoidanceController("AvoidNode")
    camera_node = camera.CameraSubscriber('camera_subscriber')
    rclpy.spin_once(camera_node)
    img=camera_node.get_image()
    while not judge(img):
        rclpy.spin_once(camera_node)
        rclpy.spin_once(AvoidNode)
    ball_tracker.main()
    AvoidNode.destroy_node()
    camera_node.destroy_node()
    rclpy.shutdown()
if __name__=="__main__":
    main()