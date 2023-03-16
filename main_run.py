import subprocess
import os
import sys
import roslaunch

def run_homing():
    """runs homing protocol"""
    subprocess.run("src/roboclaw_driver/")

def load_roslaunch():
    """loads roslaunch object to run
    Nuada main roslaunch"""
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, ["rat_control/launch/rat_hw_main.launch"])
    return launch

def load_ros_commands():
    """runs roslaunch for hardware main,
    runs rosrun for roboclaw driver and vision interface"""
    launch = load_roslaunch()
    launch.start()  # start hardware interface and roscore
    subprocess.run("rosrun roboclaw_driver subscriber.py", shell=True) # start roboclaw driver
    subprocess.run("rosrun rat_control vision_communication.py", shell=True)   # start vision interface

if __name__ == "__main__":
    while True:
        if input("run homing? [Y/n] > ") == "Y":
            run_homing()
        if input("run ros? [Y/n] >") == "Y":
            load_ros_commands()