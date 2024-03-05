import rclpy
from rclpy.node import Node
from ultralytics import YOLO
from std_srvs.srv import Empty
import cv2
from std_msgs.msg import String

class SignDetector(Node):
    def __init__(self):
        super().__init__('sign_detection')
        # The publisher that the sign name will publish on
        self.signPub = self.create_publisher(String, "sign", 10);
        
        # The message that will be published to the sign topic
        self.signMessage = String();
        
        # Running yolo stuff
        # Load a pretrained YOLOv8n model
        model = YOLO('/home/alves/Documents/Classes/Winter24/Project/code/MLPipeline/yolo/yolov7/runs/detect/train10/weights/best.pt')

        # Single stream with batch-size 1 inference
        # source = "udp://127.0.1.1:12345"  # RTSP, RTMP, TCP or IP streaming address
        source = 0

        # Run inference on the source
        model.predict(source, stream=True, verbose=False)  # generator of Results objects

        # Open the video file
        # video_path = "udp://127.0.1.1:12345"
        video_path = 0
        cap = cv2.VideoCapture(video_path)

        # Loop through the video frames
        while cap.isOpened():
            # Read a frame from the vide
            success, frame = cap.read()

            if success:
                # Run YOLOv8 inference on the frame
                results = model(frame)

                # Visualize the results on the frame
                annotated_frame = results[0].plot()
                
                # Print out which signs are seen
                for r in results:
                    
                    detection_count = r.boxes.shape[0]

                    # If results is empty publish an empty message
                    if (detection_count == 0):
                        self.signMessage.data = ""
                        self.signPub.publish(self.signMessage)

                    for i in range(detection_count):
                        cls = int(r.boxes.cls[i].item())
                        self.signMessage.data = str(r.names[cls])
                        self.signPub.publish(self.signMessage)
                    
                # Display the annotated frame
                cv2.imshow("YOLOv8 Inference", annotated_frame)

                # Break the loop if 'q' is pressed
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            else:
                # Break the loop if the end of the video is reached
                break
        # Release the video capture object and close the display window
        cap.release()
        cv2.destroyAllWindows()
        

def main(args=None):
    rclpy.init(args=args)
    node = SignDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
