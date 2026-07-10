#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sabry_hardware.srv import LinearMotor
# import serial
from std_msgs.msg import Int32

class LinearMotorNode(Node):
    def __init__(self):
        super().__init__('tool_changer_server')

        self.server_ = self.create_service(LinearMotor, 'tool_changer/set_state', self.handle_tool_change )
        self.get_logger().info("tool_changer has started")
        # try:
        #     self.serial = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        #     self.get_logger().info("Tool Changer Service Ready on /dev/ttyUSB0")
        # except serial.SerialException as e:
        #     self.get_logger().error(f"Failed to open serial port: {e}")
        #     self.serial = None

        self.command_pub = self.create_publisher(
            Int32,
            'tool_changer/command',
            10
        )

    def handle_tool_change(self, request:LinearMotor.Request, response:LinearMotor.Response):
        # if self.serial is None:
        #     response.success = False
        #     response.message = "Serial port not available"
        #     return response

        # if request.command == 1:
        #     self.serial.write(b'L\n')   # Lock
        #     response.message = "Tool locked"
        # elif request.command == 2:
        #     self.serial.write(b'U\n')   # Unlock
        #     response.message = "Tool unlocked"
        # elif request.command == 0:
        #     self.serial.write(b'S\n')   # Stop
        #     response.message = "Tool stopped"
        # else:
        #     response.success = False
        #     response.message = "Invalid command"
        #     return response

        # response.success = True

        cmd = request.command

        if cmd not in [0, 1, 2]:
            response.success = False
            response.message = "Invalid command"
            return response

        # Publish command
        msg = Int32()
        msg.data = cmd
        self.command_pub.publish(msg)

        if cmd == 1:
            response.message = "Tool lock command sent"
        elif cmd == 2:
            response.message = "Tool unlock command sent"
        elif cmd == 0:
            response.message = "Tool stop command sent"

        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = LinearMotorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
