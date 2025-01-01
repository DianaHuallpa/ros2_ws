import rclpy
from rclpy.node import Node
import numpy as np
import math

from sensor_msgs.msg import Imu

from geometry_msgs.msg import Vector3

from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Int16



class ImuReading(Node):
    def __init__(self):
        super().__init__('imu_reading')
        # Crear un perfil de QoS que sea compatible con el publicador
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Crear la suscripción usando el perfil de QoS
        self.subscription = self.create_subscription(
            Imu,
            'imu_BNO055',
            self.reading_callback,
            qos_profile
        )
        
        self.data_publisher1 = self.create_publisher(Vector3, 'orientation_IMU', 10)
        self.data_publisher2 = self.create_publisher(Vector3, 'angular_speed_IMU', 10)

        self.msg_orientacion = Vector3() 
        self.msg_velocidad_angular = Vector3() 

        self.timer = self.create_timer(0.1, self.timer_callback) #0.1 second

    def reading_callback(self, msg):
         # orientación
        orientation_quaternion = msg.orientation
        qx, qy, qz, qw = orientation_quaternion.x, orientation_quaternion.y, orientation_quaternion.z, orientation_quaternion.w


        magnitude = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
        qx /= magnitude
        qy /= magnitude
        qz /= magnitude
        qw /= magnitude

       
        # # Roll (rotación en el eje X)
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll_rad = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (rotación en el eje Y)
        sinp = 2.0 * (qw * qy - qz * qx)
        # if abs(sinp) >= 1.0:
        #     pitch_rad = math.copysign(math.pi / 2.0, sinp)  # Evita errores numéricos
        # else:
        #     pitch_rad = math.asin(sinp)
        cosp = 1.0 - 2.0 * (qy * qy + qx * qx)  # Componente adicional para desambiguar
        pitch_rad = math.atan2(sinp, cosp)
        
        # Yaw (rotación en el eje Z)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)

        # Convertir a grados
        # Ajustar Roll para que esté en el rango [-180, 180]
        roll_deg = 180- math.degrees(roll_rad)
        if roll_deg > 180:
            roll_deg -= 360
        roll=roll_deg
       
        pitch = math.degrees(pitch_rad)

        yaw = -1*math.degrees(yaw_rad)

        self.msg_orientacion.x = roll
        self.msg_orientacion.y = pitch
        self.msg_orientacion.z = yaw

        self.msg_velocidad_angular.x = msg.angular_velocity.x
        self.msg_velocidad_angular.y = msg.angular_velocity.y
        self.msg_velocidad_angular.z = msg.angular_velocity.z

    def timer_callback(self):
        self.data_publisher1.publish(self.msg_orientacion) # grados sexagesimales
        self.data_publisher2.publish(self.msg_velocidad_angular) #rad/s



def main(args=None):
    rclpy.init(args=args)
    node = ImuReading()
    print("Node is publishing data...")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Keyboard interrupt detected. Exiting...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()