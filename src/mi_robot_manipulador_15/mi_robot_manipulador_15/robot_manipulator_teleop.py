from pynput.keyboard import Key, Listener

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3

class ManipuladorTeleop(Node):

    def __init__(self):
        super().__init__('robot_manipulator_teleop')
        # Asignar a los atributos de velocidad los parámetros ingresados
        self.antebrazo = float(input("Por favor ingrese la velocidad del antebrazo (en deg/s - max600): "))
        self.brazo = float(input("Por favor ingrese la velocidad del brazo (en deg/s - max600): "))
        self.base = float(input("Por favor ingrese la velocidad de la base (en deg/s - max600): "))
        # Publicar en el tópico manipulator_cmdVel el mensaje tipo Twist
        self.publisher_ = self.create_publisher(Vector3,'manipulator_cmdVel', 10)
        self.position_publisher_ = self.create_publisher(Vector3,'robot_manipulator_position', 10)
        # Definir el Listener de la librería pynput para que detecte tecleo
        with Listener(on_press=self.callback_pressed, on_release=self.callback_released) as listener:
            listener.join()
        # Inicializar el Listener dentro del constructor de la clase
        listener.start()


    # =============== FUNCIONES DE LA LIBRERÍA ===============
    def callback_pressed(self, key):
        # Actualización de velocidades cuando se oprime una tecla
        vel_msg = Vector3()
        pos_msg = Vector3()
        # Movimiento horario del antebrazo - Rotacional hacia adelante
        if key.char == "a":
            vel_msg.x = self.antebrazo
            self.publisher_.publish(vel_msg)
            self.get_logger().info('El antebrazo se giró hacia adelante')
        # Movimiento antihorario del antebrazo - Rotacional hacia atrás
        elif key.char == "q":
            vel_msg.x = -1*self.antebrazo
            self.publisher_.publish(vel_msg)
            self.get_logger().info('El antebrazo se giró hacia atrás')
        # Movimiento horario del brazo - Rotacional hacia adelante
        elif key.char == "s":
            vel_msg.y = self.brazo
            self.publisher_.publish(vel_msg)
            self.get_logger().info('El brazo se giró hacia adelante')
        # Movimiento antihorario del brazo - Rotacional hacia atrás
        elif key.char == "w":
            vel_msg.y = -1*self.brazo
            self.publisher_.publish(vel_msg)
            self.get_logger().info('El brazo se giró hacia atrás')
        # Movimiento horario de la base - Rotacional hacia adelante
        elif key.char == "d":
            vel_msg.z = self.base
            self.publisher_.publish(vel_msg)
            self.get_logger().info('La base se giró hacia adelante')
        # Movimiento antihorario de la base - Rotacional hacia atrás
        elif key.char == "e":
            vel_msg.z = -1*self.base
            self.publisher_.publish(vel_msg)
            self.get_logger().info('La base se giró hacia atrás')
        # Movimiento de la garra - Accionado de forma automática
        elif key == Key.space:
            vel_msg.x = 1
            vel_msg.y = 1
            vel_msg.z = 1
            self.publisher_.publish(vel_msg)
            self.get_logget().info('Se accionó el mecanismo de la garra')
        

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

    manipulador_teleop = ManipuladorTeleop()

    rclpy.spin(manipulador_teleop)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    manipulador_teleop.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
