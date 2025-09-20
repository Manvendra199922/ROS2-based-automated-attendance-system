import rclpy
from rclpy.node import Node
import cv2
import pandas as pd
import time
from datetime import datetime
import os

class AnnotatedImageSubscriber(Node):
    def __init__(self):
        super().__init__('student_display')
        
        # Paths to the image and Excel file
        self.image_path = '/home/manvendra/ros2_ws/face_detected/face_detected_image.png'
        self.excel_path = '/home/manvendra/ros2_ws/attendance.xlsx'
        self.save_dir = '/home/manvendra/ros2_ws/attendance_record/'

        # Wait for 18 seconds before processing
        time.sleep(18)
        
        # Fetch the latest data from the Excel file
        self.fetch_latest_data()

    def fetch_latest_data(self):
        try:
            # Read the Excel file
            df = pd.read_excel(self.excel_path)

            # Get the latest entry
            latest_entry = df.iloc[-1]
            self.timestamp = latest_entry['Timestamp']
            self.date = latest_entry['Date']
            self.student_name = latest_entry['Name']

            # Load and display the image with the fetched data
            self.display_image_with_annotations()

        except Exception as e:
            self.get_logger().error(f"Failed to fetch data from Excel file: {e}")

    def display_image_with_annotations(self):
        # Load the image from the specified path
        frame = cv2.imread(self.image_path)

        if frame is None:
            self.get_logger().error(f"Failed to load image from {self.image_path}")
            return

        # Get the dimensions of the image for text positioning
        height, width, _ = frame.shape

        # Define the font and positions for the annotations
        font = cv2.FONT_HERSHEY_SIMPLEX
        name_position = (10, 30)
        date_position = (width - 250, height - 30)
        timestamp_position = (width - 250, height - 10)
        font_scale = 0.6
        font_color = (0, 0, 0)  # Black color
        line_type = 2

        # Add the annotations to the frame
        cv2.putText(frame, self.student_name, name_position, font, font_scale, font_color, line_type)
        cv2.putText(frame, f"Date: {self.date}", date_position, font, font_scale, font_color, line_type)
        cv2.putText(frame, f"Timestamp: {self.timestamp}", timestamp_position, font, font_scale, font_color, line_type)

        # Display the frame with the annotations
        cv2.imshow('Attendance_monitor', frame)

        # Generate a unique filename using the current timestamp
        current_time = datetime.now().strftime('%Y%m%d_%H%M%S')
        save_path = os.path.join(self.save_dir, f'{current_time}.png')

        # Save the annotated image
        cv2.imwrite(save_path, frame)
        self.get_logger().info(f"Student record updated")

        # Handle key events to close the window
        while True:
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # Close the OpenCV window
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    annotated_image_subscriber = AnnotatedImageSubscriber()

    try:
        rclpy.spin(annotated_image_subscriber)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    annotated_image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
