#!usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


import serial
import time



class RobertoMotorNode(Node):
    def __init__(self):
        super().__init__("roberto_motor")
        self.roberto_cmdVel = self.create_subscription(Twist, "/roberto_cmdVel", self.serial_motor, 10)
        self.get_logger().info(f"Roberto Motor Iniciado \n")

        try:
            self.serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
            time.sleep(2)  # Esperar a que se estabilice la conexión
            self.get_logger().info("Conexión serial con Arduino establecida")
        except Exception as e:
            self.get_logger().error(f"Error abriendo el puerto serial: {e}")
            self.serial_port = None


    def serial_motor(self, msg:Twist):

        if self.serial_port is None:
            self.get_logger().error("No hay conexión serial con Arduino")
            return

        # Extraer velocidad lineal y ángulo del servo
        velocidad = msg.linear.x
        angulo = msg.angular.z  # Asumimos que representa el ángulo del servo

        # Convertir a string para enviar
        comando = f"{velocidad:.2f},{angulo:.2f}\n"
        self.get_logger().info(f"Enviando a Arduino: {comando.strip()}")

        # ✉️ Enviar al Arduino
        self.serial_port.write(comando.encode())

    

def main(args=None):

    try:
        rclpy.init(args=args)
        node = RobertoMotorNode()
        rclpy.spin(node)
        node.cleanup_terminal()
        node.destroy_node()
        rclpy.shutdown()

    except KeyboardInterrupt:

        if node is not None:
            print(f" pressed Motor Finishing...\n")
    
    finally:

        print(f"Motor Finished Successfuly :) \n")


if __name__ == '__main__':
    main()

