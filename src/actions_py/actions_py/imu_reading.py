import rclpy
from rclpy.node import Node
import numpy as np
import transformations as tf

from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


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
        
        # Publicador al topic processed_data
        self.publisher_ = self.create_publisher(Imu, 'processed_data', 10)

    def reading_callback(self, msg):

        # orientación
        orientation_quaternion = msg.orientation
        qx, qy, qz, qw = orientation_quaternion.x, orientation_quaternion.y, orientation_quaternion.z, orientation_quaternion.w

        norm_quaternion = np.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
        qx /= norm_quaternion
        qy /= norm_quaternion
        qz /= norm_quaternion
        qw /= norm_quaternion

        euler_angles = tf.euler_from_quaternion([qx, qy, qz, qw], axes='xyzs')

        # Crear el mensaje para orientación (msg_orientacion)
        msg_orientacion = Imu()  # Asegúrate de importar el tipo de mensaje Imu
        msg_orientacion.orientation.x = -1*np.degrees(euler_angles[0])  # Roll
        msg_orientacion.orientation.y = np.degrees(euler_angles[1])  # Pitch
        msg_orientacion.orientation.z = -1*np.degrees(euler_angles[2])  # Yaw

        # Mostrar orientación recibida
        print("Orientation Quaternion:")
        print(f"x: {orientation_quaternion.x}")
        print(f"y: {orientation_quaternion.y}")
        print(f"z: {orientation_quaternion.z}")
        print(f"w: {orientation_quaternion.w}")

        print("Processed Orientation (Euler):") #Angles in degrees
        print(f"Roll: {msg_orientacion.orientation.x}")
        print(f"Pitch: {msg_orientacion.orientation.y}")
        print(f"Yaw: {msg_orientacion.orientation.z}")

        # velocidad angular
        angular_velocity = msg.angular_velocity
        print("Angular Velocity:")
        print(f"x: {angular_velocity.x}")
        print(f"y: {angular_velocity.y}")
        print(f"z: {angular_velocity.z}")

        # aceleración lineal
        linear_acceleration = msg.linear_acceleration
        print("Linear Acceleration:")
        print(f"x: {linear_acceleration.x}")
        print(f"y: {linear_acceleration.y}")
        print(f"z: {linear_acceleration.z}")

        # Publicar datos procesados
        self.publish_processed_data(msg_orientacion)

    def publish_processed_data(self, msg):
        # Publica el mismo mensaje recibido
        self.publisher_.publish(msg)
        #self.get_logger().info(f'Published processed data')
        print(f'Published processed data')

        
def main(args=None):
    rclpy.init(args=args)
    node = ImuReading()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Usa print() para evitar depender del logger
        print("Keyboard interrupt detected. Exiting...")
    finally:
        # Destruye el nodo primero
        node.destroy_node()
        # Luego verifica y apaga el contexto si es necesario
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

