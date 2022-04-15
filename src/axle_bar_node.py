#!/usr/bin/env python3

from control_msgs.msg import JointControllerState
from std_msgs.msg import Float64
from std_srvs.srv import Trigger, TriggerResponse
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
                                                    self._update_front_axle_state)
        self.sub_rear_axle_state = rospy.Subscriber('/rotrac_e2/rear_axle_position_controller/state', 
                                                    JointControllerState,
                                                    self._update_rear_axle_state)
        self.srv_raise_bars = rospy.Service("axle_bar_manager/raise_axle_bars",
                                            Trigger,
                                            self._raise_axle_bars_cb)
        self.srv_raise_bars = rospy.Service("axle_bar_manager/lower_axle_bars",
                                            Trigger,
                                            self._lower_axle_bars_cb)
        self._front_axle_state = JointControllerState()
        self._rear_axle_state = JointControllerState()
        while not self._front_axle_state or not self._rear_axle_state:
            rospy.loginfo("Waiting for axle position controller")

    def _update_front_axle_state(self, msg: JointControllerState) -> None:
        self._front_axle_state = msg
    
    def _update_rear_axle_state(self, msg: JointControllerState) -> None:
        self._rear_axle_state = msg
    
    def _raise_axle_bars_cb (self) -> None:
        rospy.logdebug("Raise axle bars service called")
        axle_position = Float64(data=0.1)
        self.pub_front_axle_position.publish(axle_position)
        self.pub_rear_axle_position.publish(axle_position)
        start_time = rospy.Time.now().to_sec()
        while not self.raised and not self._timeout(start_time, duration_limit=4):
            rospy.sleep(0.1)
        
        rospy.logdebug("Raise axle bars service finished")
        return TriggerResponse(success=self.raised)

    def _lower_axle_bars_cb (self) -> None:
        rospy.logdebug("Lower axle bars service called")
        axle_position = Float64(data=0.0)
        self.pub_front_axle_position.publish(axle_position)
        self.pub_rear_axle_position.publish(axle_position)
        start_time = rospy.Time.now().to_sec()
        while not self.lowered and not self._timeout(start_time, duration_limit=4):
            rospy.sleep(0.1)

        rospy.logdebug("Lower axle bars service finished")
        return TriggerResponse(success=self.lowered)

    @staticmethod
    def _timeout(start_time: float, duration_limit: int) -> bool:
        return (rospy.Time.now().to_sec() - start_time) >= duration_limit

    @property
    def lowered(self) -> bool:
        return (self._front_axle_state.process_value < 0.01 and
                self._rear_axle_state.process_value < 0.01)
    
    @property
    def raised(self) -> bool:
        return (self._front_axle_state.process_value > 0.09 and
                self._rear_axle_state.process_value > 0.09)

if __name__=="__main__":
    rospy.init_node('axle_bar_manager')
    try:
        axle_bar_manager = AxleBarManager()
        rospy.loginfo('[axle_bar_manager] Node initialized!')
    except rospy.ROSInterruptException:
        pass
    rospy.spin()