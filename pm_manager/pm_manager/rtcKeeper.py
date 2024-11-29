# rtcKeeper
# Generates the rtc_keeper node to populate /{namespace}/rtc_time topic
# Description:
# Publishes time from on-board hardware clock using native linux tool "hwclock" or
# from an external rtc source. Also provides rtc-ntp sync when internet is available.

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String
import subprocess, requests

class rtcKeeper(Node):
    def __init__(self):
        super().__init__("rtc_keeper")
        self.declare_parameter("rtc_source", "hwc",
                               ParameterDescriptor(description="Set source of rtc. \
                                                   'hwc' if on-board, 'path/to/driver' if external"))

        self.rtc_publisher_ = self.create_publisher(String, '/rtc_time', 10)
        self.rtc_keeper_timer_ = self.create_timer(30, self.pub_rtc_time)

    def pub_rtc_time(self):
        # call __get_rtc_time, package message in String format and publish
        rtc_time = self.__get_rtc_time()
        msg = String()
        msg.data = rtc_time
        self.rtc_publisher_.publish(msg)

    def __get_rtc_time(self):
        # check rtc_source for type implentation
        rtc_source = self.get_parameter('rtc_source').get_parameter_value().string_value
        rtc_time = None
        # check if internet is available
            # if available, update hardware clock
        # update and return rtc_time
        try:
            if self.__is_internet_there():
                self.get_logger().info("Internet access detected. Updating RTC source...")
                self.__update_rtc_clock(rtc_source)
            
            if(rtc_source == "hwc"):
                rtc_time = subprocess.run(
                    ['sudo', 'hwclock', '--show'],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    check=True
                ).stdout.strip()
            else:
                # custom implentation for external rtc
                pass

        except Exception as e:
            self.get_logger().info(f'Failed to get RTC time: {str(e)}')
        return rtc_time
    
    def __is_internet_there(self):
        try:
            requests.get("https://www.google.com", timeout=3)
            return True
        except requests.ConnectionError:
            return False

    def __update_rtc_clock(self, rtc_source):
        try:
            subprocess.run(['sudo', 'timedatectl', 'set-ntp', 'true'], check=True)
            if(rtc_source == "hwc"):
                subprocess.run(['sudo', 'hwclock', '--systohc'], check=True)
                self.get_logger().info('Updated on-board RTC source succesfully.')
            else:
                # custom implementation for external rtc
                pass
        except Exception as e:
            self.get_logger().info(f'Failed to update RTC clock: {str(e)}')

            
def main():
    rclpy.init()
    node = rtcKeeper()
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().info(f'Failed to start rtc_keeper: {str(e)}')
    except KeyboardInterrupt:
        node.get_logger().info("Manual interrupt: Shutting down rtc_keeper.")
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()