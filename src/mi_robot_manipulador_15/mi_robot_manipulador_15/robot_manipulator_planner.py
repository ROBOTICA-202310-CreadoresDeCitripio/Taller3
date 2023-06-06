from pynput.keyboard import Key, Listener

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import math
from std_msgs.msg import Float64MultiArray

class RobotManipulatorPlanner(Node):
    def __init__(self):
        super().__init__('robot_manipulator_planner')

        self.antebrazo = float(input("Por favor ingrese la posición final en x: "))
        self.brazo = float(input("Por favor ingrese la posición final en y: "))
        self.base = float(input("Por favor ingrese la posición final en z: "))
        self.publisher_ = self.create_publisher(Vector3, 'robot_manipulator_goal', 10)
        with Listener(on_press=self.callback_pressed, on_release=self.callback_released) as listener:
            listener.join()
        # Inicializar el Listener dentro del constructor de la clase
        listener.start()

    def callback_pressed(self, key):
        # Actualización de velocidades cuando se oprime una tecla
        msg = Vector3()
        print(msg)
        # Movimiento horario del antebrazo - Rotacional hacia adelante

        x = self.antebrazo / 100.0
        y = self.brazo / 100.0
        z = self.base / 100.0

        # Perform inverse kinematics calculations to determine joint angles
        # Replace the calculations below with your own inverse kinematics implementation
        if x >= 0:
            angle1 = 180.0
        else:
            angle1 = 0.0

        # Calculate angle2 using the Pythagorean theorem
        hypotenuse_length = 10.7
        opposite_length = y
        angle2 = math.degrees(math.acos(opposite_length / hypotenuse_length))
        # Limit angle2 between 0 and 180 degrees
        angle2 = max(0.0, min(angle2, 180.0))

        height = z
        angle3 = math.degrees(math.asin(height / hypotenuse_length))
        angle3 = max(0.0, min(angle3, 180.0))

        # Publish the joint angles
        if key.char == "v":
            msg.x = angle1
            msg.y = angle2
            msg.z = angle3
            self.publisher_.publish(msg)

    def callback_released(self, key):
        # Actualización a cero de las velocidades cuando se suelta una tecla
        vel_msg = Vector3()
        vel_msg.x = 0.0
        vel_msg.y = 0.0
        vel_msg.z = 0.0
        self.publisher_.publish(vel_msg)
        self.get_logger().info('No hay teclas presionadas. El manipulador se está deteniendo.')
        if key == Key.esc:
            # Stop listener when ESC is pressed
            return False

# =============== MÉTODO MAIN PARA EJECUCIÓN ===============
def main(args=None):
    rclpy.init(args=args)

    manipulador_teleop = RobotManipulatorPlanner()

    rclpy.spin(manipulador_teleop)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    manipulador_teleop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
