# pmManager
# Power mode manager. Handles boot/deep sleep scheduling for each power cycle. 
# Also ensures safe kill of services.
# Description:
# Takes the /{namespace}/rtc_time data and schedules an 'rtcalarm' for each boot. 
# Each boot will schedule a wake time and a sleep time.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess

class pmManager(Node):
    def __init__(self):
        super().__init__("pm_manager")
        
        self.dc_status_, self.cm_status_, self.pm_status_ = None;
        self.pm_status_options_ = ["OK", "FAILSAFE", "RTC_SLEEP", "UPS_POWER"]
        self.pm_status_publisher_ = self.create_publisher(String, '/pm_status', 10)
        self.pm_timer_ = self.create_timer(1, self.pm_status_callback)

        self.dc_status_subscriber_ = self.create_subscription(String, \
                                                              'dc_manager/dc_status', \
                                                                self.dc_status_check, 10)
        self.cm_status_subscriber_ = self.create_subscription(String, \
                                                              '/cm_manager/cm_status', \
                                                                self.cm_status_check, 10)

    def dc_status_check(self, msg):
        self.dc_status_ = msg.data
    
    def cm_status_check(self, msg):
        self.cm_status_ =  msg.data
    
    def pm_status_callback(self):
        pm_status = self.__get_pm_status()
        msg = String()
        msg.data = pm_status
        self.pm_status_publisher_.publish(msg)
        
        if(pm_status != self.pm_status_):
            self.__trigger_pm_actor(msg)
            self.pm_status_ = pm_status

    def __get_pm_status(self):
        # check if power mode is via mains or battery
            # if battery, move to safe kill event.
            # if mains, then pass and set rtc sleep time and next wake time.
        if(self.__power_mode()):
        # check if rtc sleep time has been set for the same day already.
            # if set for same day already, then pass. Otherwise set rtc sleep time.
        # check if rtc wake time has been set for next day already.
            # if set for same day already, then pass. Otherwise set rtc wake time.
        # __self.trigger_rtc_actor()
            pass
        else:
            self.__trigger_failsafe_status()
        # log boot timings into local sqlite implementation as device power logs.
        pass

    def __power_mode(self):
        # if power from mains then true otherwise false.
        pass

    def __trigger_failsafe_status(self):
        # (optional) add arbitrary condition to trigger "UPS_POWER" instead.
        # example: UPS battery level non-critical, then return self.pm_status_options[3].
        return self.pm_status_options_[1]
    
    def __trigger_pm_actor(self, msg):
        if msg.data == self.pm_status_options_[0]:
            pass

        elif msg.data == self.pm_status_options_[1] or msg.data == self.pm_status_options_[2] \
            and self.dc_status_ != msg.data:
            self.get_logger().info(f'pm_manager wants to shutdown due to: {str(self.pm_status_options_[1])}. \
                                   Waiting for dc_manager. dc_status: {str(self.dc_status_)}')
        
        elif msg.data == self.dc_status_ == self.pm_status_options_[1] \
              or msg.data == self.dc_status_ == self.pm_status_options_[2]:
            self.get_logger().info(f'Triggered shutdown due to pm_status: {str(self.pm_status_options_[1])}')
            subprocess.run(["sudo", "shutdown", "now"], check=True)
    
    def __trigger_rtc_actor(self):
        pass

def main():
    rclpy.init()
    node = pmManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()