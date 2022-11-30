# Roboclaw Driver
Directory containing the individual software components for Roboclaws and ROS. All things Roboclaw are located in ```./scripts/roboclaw_python```; in particular, ```roboclaw_3.py``` contains the main ```Roboclaw``` class used by the rest of the software. The ROS nodes are located in ```./scripts```, and are indicated by name as either a _publisher_ or _subscriber_.

### Manually operating individual motors
This can be done using the encoder script with the __relative__ path ```./scripts/roboclaw_python/encoders.py```. Run from the command line using ```sudo python3 encoders.py``` and follow the prompts to input your test values.
##### Relevant ranges:
```
position/encoder counts:
accel:
speed:
deccel:
```
