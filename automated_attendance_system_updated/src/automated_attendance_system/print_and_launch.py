import os
import subprocess

def main():
    # Print the desired text
    print("<------------AUTOMATED ATTENDANCE SYSTEM------------>")
    print("<------------by BHAVYA and MANVENDRA---------------->")

    # Launch ROS2 nodes
    subprocess.run(['ros2', 'launch', 'project', 'launch_file.py'])

if __name__ == '__main__':
    main()
