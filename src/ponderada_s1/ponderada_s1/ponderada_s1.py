import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill, SetPen
from geometry_msgs.msg import Twist
import time

class Tartaruguinha(Node):
    def __init__(self, start_x, turtle_name):
        super().__init__('tartaruguinha_eu_escolho_voce')
        self.turtle_name = turtle_name
        self.spawn = self.create_client(Spawn, 'spawn')
        self.kill = self.create_client(Kill, 'kill')
        self.set_pen_client = self.create_client(SetPen, f'{self.turtle_name}/set_pen')
        self.move = self.create_publisher(Twist, f'{self.turtle_name}/cmd_vel', 10)
        self.start_x = start_x

        self.spawn_turtle()
        self.set_pen_color(255, 255, 255, 3, False)  # RGB for white, width 3, pen down

    def spawn_turtle(self):
        self.get_logger().info(f"Spawning turtle: {self.turtle_name} at x={self.start_x}")
        spawn = Spawn.Request()
        spawn.x = self.start_x
        spawn.y = 5.0
        spawn.theta = 0.0
        spawn.name = self.turtle_name
        future = self.spawn.call_async(spawn)
        rclpy.spin_until_future_complete(self, future)

    def set_pen_color(self, r, g, b, width, off):
        self.get_logger().info("Escolhendo a cor da pincelada")
        self.set_pen_client.wait_for_service()
        req = SetPen.Request()
        req.r = r
        req.g = g
        req.b = b
        req.width = width
        req.off = off
        future = self.set_pen_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def Faz_o_M(self):
        # Draw M
        twist = Twist()
        moves = [
            (0.0, 1.57, 1),  # Rotate left
            (1.0, 0.0, 1),  # Move up
            (0.0, -2.0, 1),  # Rotate left
            (1.0, 0.0, 1),  # Move diagonally down right
            (0.0, 1.50, 1),  # Rotate right
            (1.0, 0.0, 1),  # Move diagonally up right
            (0.0, -2.0, 1),  # Rotate right
            (1.0, 0.0, 1)   # Move down
        ]
        for linear_x, angular_z, duration in moves:
            twist.linear.x = linear_x
            twist.angular.z = angular_z
            for _ in range(duration):
                self.move.publish(twist)
                time.sleep(1)

        self.stop_and_kill_turtle()

    def faz_o_V(self):
        # Draw V
        twist = Twist()
        moves = [
            (0.0, -0.79, 1),  # Rotate slightly left
            (1.0, 0.0, 1),  # Move diagonally down left
            (0.0, 1.57, 1),  # Rotate right to bottom point
            (1.0, 0.0, 1)   # Move diagonally up right
        ]
        for linear_x, angular_z, duration in moves:
            twist.linear.x = linear_x
            twist.angular.z = angular_z
            for _ in range(duration):
                self.move.publish(twist)
                time.sleep(1)

        self.stop_and_kill_turtle()

    def stop_and_kill_turtle(self):
        # Stop and kill the turtle
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.move.publish(twist)
        kill = Kill.Request()
        kill.name = self.turtle_name
        self.kill.call_async(kill)
        self.get_logger().info(f'Seu trabalho aqui acabou, {self.turtle_name}')

def main(args=None):
    rclpy.init(args=args)
    M1 = Tartaruguinha(2.0, 'm1_turtle')
    V1 = Tartaruguinha(5.0, 'v1_turtle')
    M2 = Tartaruguinha(7.0, 'm2_turtle')
    
    M1.Faz_o_M()
    V1.faz_o_V()
    M2.Faz_o_M()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
