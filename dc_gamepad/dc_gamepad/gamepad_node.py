#!usr/bin/env
import rclpy
from rclpy.node import Node
from dc_gamepad_msgs.msg import GamePad
import socket

class GamePadNode(Node):
    def __init__(self):
        Node.__init__(self, 'gamepad_node')
        self.datapub = self.create_publisher(GamePad, '/pad', 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ip = '0.0.0.0'
        self.port = 25000
        self.sock.bind((self.ip, self.port))
        self.last = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        while True:
            self.listen_data()

    def listen_data(self):
        msg = GamePad()

        msg.last_up = bool(self.last[0])
        msg.last_left = bool(self.last[1])
        msg.last_right = bool(self.last[2])
        msg.last_down = bool(self.last[3])
        msg.last_y = bool(self.last[4])
        msg.last_x = bool(self.last[5])
        msg.last_b = bool(self.last[6])
        msg.last_a = bool(self.last[7])
        
        msg.last_lt = bool(self.last[8])
        msg.last_lb = bool(self.last[9])
        msg.last_rt = bool(self.last[10])
        msg.last_rb = bool(self.last[11])
        msg.last_m1 = bool(self.last[12])
        msg.last_m2 = bool(self.last[13])

        data = self.sock.recv(8)
        msg.x1 = data[0]
        msg.y1 = data[1]
        msg.x2 = data[2]
        msg.y2 = data[3]
        msg.up = bool(data[4] & 1)
        msg.left = bool(data[4] & 2)
        msg.right = bool(data[4] & 4)
        msg.down = bool(data[4] & 8)
        msg.y = bool(data[4] & 16)
        msg.x = bool(data[4] & 32)
        msg.b = bool(data[4] & 64)
        msg.a = bool(data[4] & 128)

        msg.lt = bool(data[5] & 1)
        msg.lb = bool(data[5] & 2)
        msg.rt = bool(data[5] & 4)
        msg.rb = bool(data[5] & 8)
        msg.m1 = bool(data[5] & 16)
        msg.m2 = bool(data[5] & 32)

        self.datapub.publish(msg)
        self.last = [msg.up, msg.left, msg.right, msg.down, msg.y, msg.x, msg.b, msg.a, msg.lt, msg.lb, msg.rt, msg.rb, msg.m1, msg.m2]

def main(args=None):
    rclpy.init(args=args)
    gamepad_node = GamePadNode()

    try:
        rclpy.spin(gamepad_node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__=="__main__":
    main()