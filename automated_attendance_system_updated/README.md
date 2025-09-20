# Automated Attendance System – ROS2 Package  

This folder contains the complete ROS2 package implementation of the **Automated Attendance System**.  
It includes the source nodes, launch files, configuration, and dependencies needed to run the project.  

---

## Folder Structure  

automated_attendance_system_updated/  
│── launch/  
│   └── launch_file.py              # Launches all nodes together  
│── src/automated_attendance_system/  
│   ├── node1.py                    # Camera + face detection + anti-spoofing  
│   ├── node2.py                    # Face recognition + attendance update  
│   ├── node3.py                    # Display recognized student + save annotated records  
│   ├── print_and_launch.py         # Entry point to run full system  
│── resource/  
│   └── automated_attendance_system # Package resource file  
│── setup.py  
│── setup.cfg  
│── package.xml  
│── requirements.txt  
│── README.md                       # This file  

---

## How to Use  

### 1. Build the package  
colcon build  
source install/setup.bash  

### 2. Run the system  
ros2 run automated_attendance_system print_and_launch  

### 3. Run with launch file  
ros2 launch automated_attendance_system launch_file.py  

### 4. Stop execution  
Press **`q`** in the display window to exit.  

---

## Outputs  

- **attendance.xlsx** → Excel sheet with  
  - Student Name  
  - Date  
  - Timestamp  
  - Match Score  

- **face_detected/** → Cropped face images captured from camera.  

- **attendance_record/** → Annotated images of recognized students (with name + score).  

---

## Notes  

- You can run nodes individually for debugging:  
  - `node1` → Camera & detection  
  - `node2` → Recognition & attendance update  
  - `node3` → Display & record keeping  
- The launch file and `print_and_launch.py` provide a **single entry point** for running the entire system.  

---

## Author  
- **Manvendra Pratap Singh**
