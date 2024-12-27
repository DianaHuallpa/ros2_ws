import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String  # Para el estado de calibración
import board
import adafruit_bno055

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(Imu, 'imu_BNO055', 10)  # Nombre del tópico: imu_data
        self.calibration_publisher_ = self.create_publisher(String, 'imu_calibration', 10)  # Tópico de calibración
        self.timer = self.create_timer(0.1, self.publish_imu_data)  # Frecuencia: 10 Hz
        i2c = board.I2C()  # Inicializa el sensor
        self.sensor = adafruit_bno055.BNO055_I2C(i2c)

    def publish_imu_data(self):
        msg = Imu()

        # Obtener los datos de aceleración lineal
        acceleration = self.sensor.linear_acceleration
        if acceleration:
            msg.linear_acceleration.x = acceleration[0]
            msg.linear_acceleration.y = -1 * acceleration[1]  # Invertir el eje Y
            msg.linear_acceleration.z = acceleration[2]

        # Obtener los datos de velocidad angular (giroscopio)
        gyro = self.sensor.gyro
        if gyro:
            msg.angular_velocity.x = -1 * gyro[0]  # Invertir el eje X
            msg.angular_velocity.y = gyro[1]
            msg.angular_velocity.z = -1 * gyro[2]  # Invertir el eje Z

        # Obtener los datos de orientación utilizando la fusión de sensores (cuaterniones)
        quaternion = self.sensor.quaternion
        if quaternion:
            msg.orientation.x = -quaternion[0]
            msg.orientation.y = -quaternion[1]  
            msg.orientation.z = quaternion[2]  
            msg.orientation.w = quaternion[3]

        # Obtener estado de la calibración
        calibration_status = self.get_calibration_status()

        # Publicar los datos del IMU
        self.publisher_.publish(msg)

        # Publicar estado de calibración
        calibration_msg = String()
        calibration_msg.data = calibration_status
        self.calibration_publisher_.publish(calibration_msg)

        self.get_logger().info('Published IMU data and calibration status')

    def get_calibration_status(self):
        # Obtener el estado de calibración
        calibration = self.sensor.calibration_status
        # Los valores de calibración son 0 a 3, donde 3 es completamente calibrado
        calib_str = (
            f"Calibración completa: {calibration[0]}, "
            f"Giroscopio: {calibration[1]}, "
            f"Acelerómetro: {calibration[2]}, "
            f"Magnetómetro: {calibration[3]}"
        )
        return calib_str

def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
