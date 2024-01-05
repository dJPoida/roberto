#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32
from smbus import SMBus

# Message Types
I2C_MSG_HEARTBEAT = 'H'
I2C_MSG_BATTERY_LEVEL_UPDATE = 'B'

# TODO: Parameterize these values
I2C_BUS_ADDRESS = 8     # I2C Bus Address (must match that of the arduino code)
I2C_BUS = 1             # indicates /dev/ic2-1

class I2CHost(Node):

    def __init__(self):
        super().__init__("i2c_host")
        self.bus_addr = I2C_BUS_ADDRESS # bus address 
        self.bus = SMBus(I2C_BUS) 

        # Create a timer to emit a heartbeat to the I2C slave
        self._heartbeat_timer = self.create_timer(0.5, self.emit_heartbeat)
        
        # Subscribe to the Power Monitor Battery Level topic and send the battery level over I2C when received
        self.power_monitor_subscriber = self.create_subscription(Float32, "power/battery_percent", self.emit_battery_level, 10)

        self.get_logger().info("I2C Host Node Initialised")

    def emit_heartbeat(self):
        try:
            self.bus.write_byte(self.bus_addr, ord(I2C_MSG_HEARTBEAT))
        except OSError:
            self.get_logger().error("Looks like the I2C connection has been severed")

    def emit_battery_level(self, battery_level):
        try:
            self.bus.write_byte_data(self.bus_addr, ord(I2C_MSG_BATTERY_LEVEL_UPDATE), int(math.floor(battery_level.data * 100)))
        except OSError:
            self.get_logger().error("Looks like the I2C connection has been severed")

def main(args=None):
    rclpy.init(args=args)
    node = I2CHost()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

