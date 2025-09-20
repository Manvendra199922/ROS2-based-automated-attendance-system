# Automated Attendance System using ROS2  

An intelligent **ROS2-based Automated Attendance System** that leverages **computer vision and face recognition** to automate student attendance recording.  
This system captures faces in real time, performs **anti-spoofing checks**, matches them against a pre-stored database, and updates attendance records in an Excel sheet.  

---

## 🖼️ System Overview  
The project is structured into **ROS2 nodes** that communicate through topics:  

- **Node1 (Camera & Face Detection)**  
  - Captures live camera feed  
  - Detects faces using Haar Cascade Classifier  
  - Performs basic anti-spoofing  
  - Publishes detected images  

- **Node2 (Face Recognition & Attendance Update)**  
  - Receives detected faces  
  - Compares against known student dataset using `face_recognition`  
  - Updates attendance in `attendance.xlsx` with timestamp, date, name, and match score  

- **Node3 (Display & Record Keeping)**  
  - Annotates recognized face image with student details  
  - Displays result on screen  
  - Saves annotated images in `attendance_record/`  

- **Launch File**  
  - Orchestrates all nodes to run together  

---

## ✨ Features  
✔️ Live **camera-based face detection**  
✔️ **Anti-spoofing** mechanism (basic image variance check)  
✔️ **Face recognition** using deep feature embeddings (`face_recognition`)  
✔️ Automatic **attendance logging** in Excel (`openpyxl`)  
✔️ **Annotated visual feedback** with student details  
✔️ Modular ROS2 architecture – nodes can run independently or via launch file  

---

## 📂 Project Structure  
```
automated_attendance_system/
│── launch/
│   └── launch_file.py
│── src/automated_attendance_system/
│   ├── node1.py              # Camera, face detection, anti-spoofing, image publisher
│   ├── node2.py              # Face recognition, attendance logging
│   ├── node3.py              # Display recognized student, save annotated record
│   ├── print_and_launch.py   # Entry script to launch system
│── resource/
│   └── automated_attendance_system
│── setup.py
│── setup.cfg
│── package.xml
│── requirements.txt
│── README.md
```

---

## ⚙️ Installation  

### 1. Prerequisites  
- **Ubuntu 20.04 / 22.04** with **ROS2 Foxy/Humble**  
- Python ≥ 3.8  
- Camera/webcam  

### 2. Install dependencies  
```bash
sudo apt update
sudo apt install python3-colcon-common-extensions                  ros-${ROS_DISTRO}-cv-bridge                  ros-${ROS_DISTRO}-image-transport

pip install -r requirements.txt
```

`requirements.txt` includes:  
- `opencv-python`  
- `face_recognition`  
- `pandas`  
- `openpyxl`  
- `numpy`  
- `rclpy`  
- `cv_bridge`  

---

## ▶️ Usage  

### 1. Clone and build the package  
```bash
git clone https://github.com/<your-username>/automated_attendance_system.git
cd automated_attendance_system
colcon build
source install/setup.bash
```

### 2. Run the system  
```bash
ros2 run automated_attendance_system print_and_launch
```

### 3. Stop execution  
Press **`q`** in the image window to quit.  

---

## 📊 Output  

- `attendance.xlsx` → Logs attendance with  
  - Timestamp  
  - Date  
  - Student Name  
  - Match Score  

- `face_detected/` → Saves detected face snapshots  

- `attendance_record/` → Stores annotated images with details  

---

## 🔍 How It Works (Pipeline)  
1. **Capture** → Camera feed captured via OpenCV.  
2. **Detection** → Haar cascade detects faces.  
3. **Anti-Spoofing** → Color variance check filters spoof attempts.  
4. **Recognition** → Matches face encodings with student dataset.  
5. **Attendance Marking** → Updates `attendance.xlsx`.  
6. **Annotation & Storage** → Saves/display annotated image.  

---

## 📸 Demo (Conceptual)  
- **Face Detected:** Captured via webcam.  
- **Recognition:** Matches against known dataset.  
- **Attendance Record:** Automatically updated in Excel.  
- **Annotated Image:** Displayed & saved with details.  

---

## 👨‍💻 Authors  
- **Manvendra Pratap Singh**  

--- 
