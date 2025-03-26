import rclpy
from rclpy.node import Node


from geometry_msgs.msg import Point

from starship_detector import locate_func



class JokerFinder(Node):


    def __init__(self):
        super().__init__('target_location')
        self.pub=self.create_publisher(Point,'get_point',10)

        timer_period = 1
        self.timer = self.create_timer(timer_period, self.callback)



    def callback(self):
        msg=Point()
        status,x,y,z=locate_func.get_location()

        if (not status): 
            msg.x=0.0
            msg.y=0.0
            msg.z=0.0
            self.get_logger().error("Sensor Data Unavaiable")
        else:
            msg.x=x
            msg.y=y
            msg.z=z  
            if (x, y, z) == (0.0, 0.0, 0.0):
                self.get_logger().info("Object not detected")
            else:
                self.get_logger().info(f"Object detected at {x}, {y}, {z}")
        
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    publisher=JokerFinder()

    rclpy.spin(publisher)


if __name__ == '__main__':
    main()
