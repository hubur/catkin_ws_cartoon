import rospy
from nav_msgs.msg import Odometry
from autominy_msgs.msg import SpeedCommand
import tf.transformations
import numpy


class PID:

    def __init__(self):
        rospy.init_node("speed_pid")
        self.speed_pub = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=10)
        self.localization_sub = rospy.Subscriber("/sensors/speed", SpeedCommand, self.on_speed_sense, queue_size=1)
        self.speed_sub = rospy.Subscriber("/control/speed", SpeedCommand, self.on_speed_control, queue_size=1)

        self.speed = SpeedCommand()
        self.rate = rospy.Rate(100)
        self.timer = rospy.Timer(rospy.Duration.from_sec(0.01), self.on_control)

        self.kp = 1.0
        self.ki = 0.0
        self.kd = 0.0
        self.min_i = -1.0
        self.max_i = 1.0

        self.integral_error = 0.0
        self.last_error = 0.0

        # this should be changed from a topic for future tasks
        self.desired_speed = 0

        rospy.on_shutdown(self.on_shutdown)

        while not rospy.is_shutdown():
            self.rate.sleep()

    def on_speed_sense(self, msg):
        self.speed = msg

    def on_speed_control(self, msg):
        self.desired_speed = msg.value

    def on_control(self, tmr):

        if tmr.last_duration is None:
            dt = 0.01
        else:
            dt = (tmr.current_expected - tmr.last_expected).to_sec()

        print dt
        error = self.desired_speed - self.speed.value

        self.integral_error += error * dt
        self.integral_error = max(self.min_i, self.integral_error)
        self.integral_error = min(self.max_i, self.integral_error)

        derivative_error = (error - self.last_error) / dt
        self.last_error = error

        pid_output = self.kp * error + self.kd * derivative_error + self.ki * self.integral_error
        print("x", pid_output, self.desired_speed, self.speed.value)
        pid_output += self.speed.value
        pid_output = min(pid_output, 0.5)

        #speed_msg = SpeedCommand()
        #speed_msg.value = 0.2
        #self.speed_pub.publish(speed_msg)

        speed_msg = SpeedCommand()
        speed_msg.value = pid_output
        self.speed_pub.publish(speed_msg)

    def on_shutdown(self):
        speed_msg = SpeedCommand()
        speed_msg.value = 0.0
        self.speed_pub.publish(speed_msg)


if __name__ == "__main__":
    PID()
