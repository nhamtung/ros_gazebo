#!/usr/bin/env python
import Tkinter as tk
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class Slider:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("GUI Slider")

        self.lift = tk.Scale(self.root, label="Lift", length=200, from_=100, to=0, orient=tk.VERTICAL)
        self.lift.pack()
        self.lift.bind("<ButtonRelease-1>", self.updateValue_lift)

        self.b1 = tk.Button(text='Reset', command=self.reset_model).pack()

        self.root.mainloop()

    def updateValue_lift(self, event):
        print("join_state_publisher - lift:{}".format(self.lift.get()))
        self.join_state_publish("lift", (self.lift.get()*0.25)/100)

    def reset_model(self):
        print("join_state_publisher - reset")
        self.lift.set(0)
        self.join_state_publish("lift", 0)

    def join_state_publish(self, joint_name, value):
        print("join_state_publisher - pub:{}, value:{}".format(joint_name, value))

        pub = rospy.Publisher('/joint_states_lift_module', JointState, queue_size=10)
        rospy.init_node('joint_states_lift_module')
        rate = rospy.Rate(10) # 10hz
        hello_str = JointState()
        hello_str.header = Header()
        hello_str.header.stamp = rospy.Time.now()
        hello_str.name = [joint_name]
        hello_str.position = [value]
        hello_str.velocity = [0]
        hello_str.effort = [0]
        pub.publish(hello_str)
        rate.sleep()

if __name__ == "__main__":
    slider = Slider()