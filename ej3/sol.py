import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserEcho
from std_srvs.srv import Trigger
from std_srvs.srv import Empty
from std_srvs.srv import SetBool


class Explorer:
    def __init__(self):
        rospy.init_node('explorador')
        self.sub_robot_vel=rospy.Subscriber('/robot_velocity', Twist, self.robot_vel_callback)
        self.sub_sensor=rospy.Subscriber('/sensor', LaserEcho, self.sensor_callback)

        self.map_size = 15
        self.robot_pose = [7,7]
        self.robot_move = [0,0]
        self.internal_map = np.full((self.map_size, self.map_size), -1)
        rospy.spin()
    
    def update_cells(self, msg=LaserEcho):
        self.robot_pose[0] = self.robot_pose[0] + self.robot_move[0]
        self.robot_pose[1] = self.robot_pose[1] + self.robot_move[1]
        self.robot_move = [0,0]

        for val in msg.echoes:
            print(val)
        self.internal_map[int(self.robot_pose[0])][int(self.robot_pose[1]+1)] = msg.echoes[2]
        self.internal_map[int(self.robot_pose[0]+1)][int(self.robot_pose[1])] = msg.echoes[3]
        self.internal_map[int(self.robot_pose[0])][int(self.robot_pose[1]-1)] = msg.echoes[0]
        self.internal_map[int(self.robot_pose[0]-1)][int(self.robot_pose[1])] = msg.echoes[1]
        self.internal_map[int(self.robot_pose[0])][int(self.robot_pose[1])] = 10

    def print_map(self):
        for row in self.internal_map:
            for cell in row:
                if cell == 10:
                    print("@"),
                elif cell == -1:
                    print("#"),
                elif cell == 1:
                    print("="),
                elif cell == 0:
                    print("."),
                elif cell == 2:
                    print("C"),
                else:
                    print("_"),
            print("")

    def sensor_callback(self, msg):
        self.update_cells(msg)
        self.print_map()

    def robot_vel_callback(self, msg=Twist):
        delta_x = int(10*msg.linear.x)
        delta_y = int(10*msg.linear.y)
        self.robot_move = [0,0]

        if delta_x > 0:
            self.robot_move = [-1,0]
        elif delta_x < 0:
            self.robot_move = [1,0]
        elif delta_y > 0:
            self.robot_move = [0,-1]
        elif delta_y < 0:
            self.robot_move = [0,1]


if __name__=='__main__':
    explorer = Explorer()
