
import evdev
import rospy

from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Int16, Bool
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped




rospy.init_node("JOY")
cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)


devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
dddd = "/dev/input/event26"
for device in devices:
    if "Flysky" in device.name:
        dddd = device.path
        print(device.path, device.name, device.phys)
device = evdev.InputDevice(dddd)


forward_val = 0
right_val = 0

SPEED = 0.2
SPEED_A = 0.5

for event in device.read_loop():
    if event.type == evdev.ecodes.EV_KEY:
        print(evdev.categorize(event))
        if event.code == evdev.ecodes.BTN_BASE2:
            if event.value == 1:
                exit()
    if event.type == evdev.ecodes.EV_ABS:
        # print(evdev.categorize(event))
        # cat = evdev.categorize(event)
        if event.code == evdev.ecodes.ABS_Y:
            # print()
            forward_val = event.value/90.5
        elif event.code == evdev.ecodes.ABS_X:
            right_val = event.value/90.5
        print("f", forward_val, "r", right_val)
        tw = Twist()
        # rospy.sleep(2)
        # print("A")
        tw.linear.x = forward_val*SPEED
        tw.angular.z = right_val*SPEED_A
        cmd_vel.publish(tw)
        
        # if  == evdev.ecodes.ABS_Y:
        #     print("dd")
        #     print()
