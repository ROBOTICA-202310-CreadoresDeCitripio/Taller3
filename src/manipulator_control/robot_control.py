import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import serial,time
import serial.tools.list_ports


class RobotController(Node):
    # El constructor de la clase se suscribe al tópico robot_cmdVel en la VM
    def __init__(self):
        super().__init__('robot_controller')
        # Open serial connection to Arduino
        ports = list(serial.tools.list_ports.comports())
        arduino_port = ports[0].device
        self.arduino = serial.Serial(port=arduino_port, baudrate=9600, timeout=0.1)
        # Wait for Arduino to reset
        time.sleep(2)

        self.subscription = self.create_subscription(
            Vector3,
            'manipulator_cmdVel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # Extract linear and angular velocities from Twist message
        value1 = msg.x
        value2 = msg.y
        value3 = msg.z
        # Pack linear and angular velocities into byte array and send over serial
        data = "{},{},{}\n".format(value1, value2, value3)
        self.ser.write(data.encode())


def main(args=None):
    # Creación del nodo de ROS2 que controla al robot
    rclpy.init(args=args)
    robot_controller = RobotController()
    rclpy.spin(robot_controller)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()