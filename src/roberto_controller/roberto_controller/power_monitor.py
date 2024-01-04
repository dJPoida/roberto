#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Float32
# TODO: move this to a lib or resource location and use as a dependency
from .submodules.DFRobot_INA219 import INA219

# Calibration
# TODO: allow parameterisation of the INA219 calibration values
ina219_reading_mA = 1000
ext_meter_reading_mA = 1000
battery_voltage_max = 12.6
battery_voltage_min = 10.8

class PowerMonitor(Node):

    def __init__(self):
        super().__init__("power_monitor")
        self.init_INA219()
        self._battery_voltage = 0
        self._battery_percent = 0
        self._current_draw_ma = 0
        self.publisher_battery_voltage = self.create_publisher(Float32, "power/battery_voltage", 10)
        self.publisher_battery_percent = self.create_publisher(Float32, "power/battery_percent", 10)
        self.publisher_current_draw_ma = self.create_publisher(Float32, "power/current_draw_ma", 10)
        self._publish_timer = self.create_timer(0.5, self.publish_messages)
        self.get_logger().info("Power Monitor Node Initialised")

    def init_INA219(self):
        # TODO: allow parameterisation of the I2C Address
        self._ina = INA219(1, INA219.INA219_I2C_ADDRESS4)
        # Wait for the INA to become available
        # TODO: turn this into an asynchronous process
        # TODO: fail if the INA219 could not be contacted after a certain amount of time
        while not self._ina.begin():
            time.sleep(2)
        self._ina.linear_cal(ina219_reading_mA, ext_meter_reading_mA)

    def publish_messages(self):
        self._battery_voltage = self._ina.get_bus_voltage_V()
        self._battery_percent = (self._battery_voltage - battery_voltage_min) / (battery_voltage_max - battery_voltage_min)
        self._current_draw_ma = self._ina.get_current_mA()
       
        msg_battery_voltage = Float32()
        msg_battery_voltage.data = self._battery_voltage
        self.publisher_battery_voltage.publish(msg_battery_voltage)

        msg_battery_percent = Float32()
        msg_battery_percent.data = self._battery_percent
        self.publisher_battery_percent.publish(msg_battery_percent)

        msg_current_draw_ma = Float32()
        msg_current_draw_ma.data = self._current_draw_ma
        self.publisher_current_draw_ma.publish(msg_current_draw_ma)

def main(args=None):
    rclpy.init(args=args)
    node = PowerMonitor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()