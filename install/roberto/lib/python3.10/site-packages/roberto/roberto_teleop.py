#!usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import sys, os
import termios, select
import fcntl



class RobertoTeleopNode(Node):
    def __init__(self):
        super().__init__("roberto_teleop")
        self.roberto_cmdVel = self.create_publisher(Twist, "/roberto_cmdVel", 10)
        servo_angle = 30.0
        lineal_vel = 100.0


        self.sys_settings = sys.stdin.fileno()
        newattr = termios.tcgetattr(self.sys_settings)
        newattr[3] = newattr[3] & ~termios.ICANON
        newattr[3] = newattr[3] & ~termios.ECHO
        termios.tcsetattr(self.sys_settings, termios.TCSANOW, newattr)

        self.oldterm = termios.tcgetattr(self.sys_settings)
        self.oldflags = fcntl.fcntl(self.sys_settings, fcntl.F_GETFL)
        fcntl.fcntl(self.sys_settings, fcntl.F_SETFL, self.oldflags | os.O_NONBLOCK)

        self.servo_angle = servo_angle
        self.lineal_vel = lineal_vel
        self.key = None

        while (self.key != "q"):
            try:
                self.getKey()
            
                msg = Twist()
                if self.key == "w":
                    msg.linear.x = self.lineal_vel
                elif self.key == "s":
                    msg.linear.x = -self.lineal_vel
                else:
                    msg.linear.x = 0.0

                if self.key == "d":
                    msg.angular.z = -self.servo_angle
                elif self.key == "a":
                    msg.angular.z = self.servo_angle
                else:
                    msg.angular.z = 0.0
                self.roberto_cmdVel.publish(msg)

            except KeyboardInterrupt:
                self.cleanup_terminal()
                self.get_logger().info(f'Press Q to exit')
                break
                
        else:
            self.get_logger().info(f'Teleop Shutdown Press ctrl+c to kill the Node\n')

        self.cleanup_terminal()
        

    def getKey(self):
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            #tty.setraw(sys.stdin.fileno())
            k = sys.stdin.read(1)
            
            if k == '\x1b':
                kk = sys.stdin.read(2)
                match kk:
                    case '[A':
                        k = "w"  # Flecha arriba
                    case '[B':
                        k = "s"  # Flecha abajo
                    case '[C':
                        k = "d"  # Flecha derecha
                    case '[D':
                        k = "a"  # Flecha izquierda
                    case _:
                        k = None
            self.key = k
        else:
            # Si no se presionó ninguna tecla, el valor de key es None
            self.key = None

    def cleanup_terminal(self):
        # Restaurar los atributos originales de la terminal
        termios.tcsetattr(self.sys_settings, termios.TCSAFLUSH, self.oldterm)
        fcntl.fcntl(self.sys_settings, fcntl.F_SETFL, self.oldflags)
        os.system('stty sane')

def main(args=None):

    try:
        rclpy.init(args=args)
        node = RobertoTeleopNode()
        rclpy.spin(node)
        node.cleanup_terminal()
        node.destroy_node()
        rclpy.shutdown()

    except KeyboardInterrupt:

        if node is not None:
            print(f" pressed Teleop Finishing...\n")
    
    finally:

        print(f"Teleop Finished Successfuly :) \n")


if __name__ == '__main__':
    main()

