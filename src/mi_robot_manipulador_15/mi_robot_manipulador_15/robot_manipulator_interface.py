import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3


class RobotManipulatorInterface(Node):
    def __init__(self):
        super().__init__('robot_manipulator_interface')

        # Subscribe to ROS2 topic "robot_manipulator_position"
        self.subscription = self.create_subscription(
            Vector3,
            'robot_manipulator_position',
            self.position_callback,
            10
        )

        # Initialize position values
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        # Initialize Pygame
        pygame.init()
        self.width, self.height = 800, 600
        pygame.display.set_mode((self.width, self.height), DOUBLEBUF | OPENGL)

        # Run the main loop
        self.main_loop()

    def position_callback(self, msg):
        # Update position values
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z

    def main_loop(self):
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    shutdown()
                    return

            # Clear the screen
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            glLoadIdentity()

            # Set up the perspective projection
            gluPerspective(45, (self.width / self.height), 0.1, 50.0)
            glTranslatef(0.0, 0.0, -5)

            # Draw the end-effector position
            glBegin(GL_POINTS)
            glColor3f(1.0, 0.0, 0.0)
            glVertex3f(self.x, self.y, self.z)
            glEnd()

            pygame.display.flip()
            pygame.time.wait(10)


if __name__ == '__main__':
    init()
    robot_interface = RobotManipulatorInterface()