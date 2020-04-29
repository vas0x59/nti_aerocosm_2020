
import evdev
import rospy

from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Int16, Int32
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped




rospy.init_node("JOY")
cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

servo1 = rospy.Publisher("/arduino/servo1", Int32, queue_size=10)
servo2 = rospy.Publisher("/arduino/servo2", Int32, queue_size=10)
slider = rospy.Publisher("/arduino/slider", Int32, queue_size=10)
motor = rospy.Publisher("/arduino/motor1", Int32, queue_size=10)


devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
dddd = "/dev/input/event26"
for device in devices:
    if "Flysky" in device.name:
        dddd = device.path
        print(device.path, device.name, device.phys)
device = evdev.InputDevice(dddd)


forward_val = 0
right_val = 0
thru_val = 0
yaw_val = 0

r1_val = 0
r2_val = 0
MAXMOTOR=80
motor_speed = 0

SPEED = 0.2
SPEED_A = 0.5

revers_btn = 1

for event in device.read_loop():
    if event.type == evdev.ecodes.EV_KEY:
        print(evdev.categorize(event))
        if event.code == evdev.ecodes.BTN_BASE2:
            if event.value == 1:
                exit()
        elif event.code == evdev.ecodes.BTN_TOP:
            if event.value == 1:
                revers_btn = -1
            else:
                revers_btn = 1
        elif event.code == evdev.ecodes.BTN_THUMB:
            if event.value == 1:
                motor_speed = MAXMOTOR
            else:
                motor_speed = 0
    if event.type == evdev.ecodes.EV_ABS:
        # print(evdev.categorize(event))
        # cat = evdev.categorize(event)
        if event.code == evdev.ecodes.ABS_Y:
            # print()
            forward_val = event.value/90.5
        elif event.code == evdev.ecodes.ABS_X:
            right_val = event.value/90.5
        elif event.code == evdev.ecodes.ABS_Z:
            thru_val = event.value/90.5
        elif event.code == evdev.ecodes.ABS_RX:
            yaw_val  = event.value/90.5
        elif event.code == evdev.ecodes.ABS_RX:
            yaw_val  = event.value/90.5
        elif event.code == evdev.ecodes.ABS_RY:
            r1_val  = event.value/90.5
        elif event.code == evdev.ecodes.ABS_RZ:
            r2_val  = event.value/90.5
        print("f", forward_val, "r", right_val, "T", thru_val, "y", yaw_val, "1", r1_val, "2", r2_val)
        tw = Twist()
        # rospy.sleep(2)
        # print("A")
        tw.linear.x = forward_val*SPEED
        tw.angular.z = -right_val*SPEED_A
        cmd_vel.publish(tw)
    servo1.publish(int((-r1_val+1) * 90))
    servo2.publish(int((r2_val+1) * 90))
    slider.publish(int(yaw_val*10))
    print(revers_btn*motor_speed)
    motor.publish(revers_btn*motor_speed)
        # if  == evdev.ecodes.ABS_Y:
        #     print("dd")
        #     print()
