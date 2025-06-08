import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point
from my_robot_msgs.msg import JointAngles  # Dodano za slanje JointAngles

import ast  # Za parsanje string liste

class ModeSelector(Node):
    def __init__(self):
        super().__init__('mode_selector')

        self.declare_parameter("manual_mode_options", ["angle", "xy"])

        self.detections = []
        self.subscription = self.create_subscription(
            String,
            'yolo_detection',
            self.detection_callback,
            10
        )

        self.start_pub = self.create_publisher(String, '/path/start', 10)
        self.end_pub = self.create_publisher(String, '/path/end', 10)
        self.mode_pub = self.create_publisher(String, '/selected_mode', 10)
        self.xy_pub = self.create_publisher(Point, '/manual/xy', 10)
        self.joint_angle_pub = self.create_publisher(JointAngles, '/manual_angles', 10)  # Novi publisher

        self.timer = self.create_timer(2.0, self.run_interface)

    def detection_callback(self, msg):
        try:
            self.detections = ast.literal_eval(msg.data)
        except Exception as e:
            self.get_logger().error(f"Neispravan format poruke: {e}")

    def run_interface(self):
        self.timer.cancel()  # Izvršava se samo jednom

        mode = input("Odaberi mod [automatic/manual]: ").strip().lower()
        self.mode_pub.publish(String(data=mode))

        if mode == "automatic":
            self.handle_automatic_mode()
        elif mode == "manual":
            self.handle_manual_mode()
        else:
            print("Nepoznat mod. Izlazim.")
            rclpy.shutdown()

    def handle_automatic_mode(self):
        if not self.detections:
            print("Nema detekcija.")
            return

        imena = [det[0] for det in self.detections]
        print("Detektirani objekti:")
        for ime in imena:
            print(f"- {ime}")

        start = input("Unesi ime početnog objekta: ").strip()
        end = input("Unesi ime završnog objekta: ").strip()

        if start not in imena or end not in imena:
            print("Jedan ili oba unosa nisu validni.")
            return

        self.start_pub.publish(String(data=start))
        self.end_pub.publish(String(data=end))
        print(f"Objekti '{start}' i '{end}' poslani.")

    def handle_manual_mode(self):
        opcija = input("Unesi tip unosa [angle/xy]: ").strip().lower()

        if opcija == "angle":
            kutovi = []
            for i in range(1, 4):
                kut = float(input(f"Unesi kut {i} (stupnjevi): "))
                kutovi.append(kut)
            # Slanje samo kao JointAngles
            joint_msg = JointAngles()
            joint_msg.phi = kutovi[0]
            joint_msg.alpha = kutovi[1]
            joint_msg.theta = kutovi[2]
            self.joint_angle_pub.publish(joint_msg)
            print(f"Kutevi poslani kao JointAngles na /manual_angles: phi={kutovi[0]}, alpha={kutovi[1]}, theta={kutovi[2]}")
        elif opcija == "xy":
            x = float(input("Unesi x koordinatu: "))
            y = float(input("Unesi y koordinatu: "))
            point = Point(x=x, y=y, z=0.0)
            self.xy_pub.publish(point)
            print(f"Koordinate ({x}, {y}) poslane na /manual/xy")
        else:
            print("Nepoznata opcija.")

if __name__ == "__main__":
    rclpy.init()
    try:
        node = ModeSelector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Prekid...")
    finally:
        rclpy.shutdown()
