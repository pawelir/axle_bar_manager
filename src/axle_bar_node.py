#!/usr/bin/env python3

from control_msgs.msg import JointControllerState
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
import rospy

'''Module responsible for controlling the Rotrac's axle bars'''

class AxleBarManager:
    def __init__(self):
        self.pub_front_axle_position = rospy.Publisher('/rotrac_e2/front_axle_position_controller/command',
                                                       Float64,
                                                       queue_size=1)
        self.pub_rear_axle_position = rospy.Publisher('/rotrac_e2/rear_axle_position_controller/command',
                                                       Float64,
                                                       queue_size=1)
        self.sub_front_axle_state = rospy.Subscriber('/rotrac_e2/front_axle_position_controller/state', 
                                                    JointControllerState,
                                                    self.update_front_axle_state)
        self.sub_rear_axle_state = rospy.Subscriber('/rotrac_e2/rear_axle_position_controller/state', 
                                                    JointControllerState,
                                                    self.update_rear_axle_state)
        self.srv_raise_bars = rospy.Service("~/raise_axle_bars",
                                            Trigger,
                                            self.raise_axle_bars_cb)
        self.srv_raise_bars = rospy.Service("~/lower_axle_bars",
                                            Trigger,
                                            self.lower_axle_bars_cb)
        self.axle_position = Float64()
        self.front_axle_state = JointControllerState()
        self.rear_axle_state = JointControllerState()
        rospy.sleep(0.5)

    def update_front_axle_state(self,msg) -> None:
        self.front_axle_state = msg
    
    def update_rear_axle_state(self,msg) -> None:
        self.rear_axle_state = msg
    
    def raise_axle_bars_cb (self) -> None:
        self.axle_position.data = 0.1
        self.pub_front_axle_position.publish(self.axle_position)
        self.pub_rear_axle_position.publish(self.axle_position)

    def lower_axle_bars_cb (self) -> None:
        self.axle_position.data = 0.0
        self.pub_front_axle_position.publish(self.axle_position)
        self.pub_rear_axle_position.publish(self.axle_position)

    @property
    def lowered(self) -> bool:
        return (self.front_axle_state.process_value < 0.01 and
                self.rear_axle_state.process_value < 0.01)

if __name__=="__main__":
    rospy.init_node('axle_bar_manager')
    try:
        axle_bar_manager = AxleBarManager()
        rospy.loginfo('[axle_bar_manager] Node initialized!')
    except rospy.ROSInterruptException:
        pass
    rospy.spin()