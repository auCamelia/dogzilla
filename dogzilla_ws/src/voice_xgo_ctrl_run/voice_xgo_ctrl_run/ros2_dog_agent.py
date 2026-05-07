import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import base64
import cv2
from cv_bridge import CvBridge
import math
import io
import numpy as np
from rclpy.qos import qos_profile_sensor_data
import json
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
import sys,os,re



class AgentCtrlSendGoal(Node):
    def __init__(self):
        super().__init__("Agent_ctrl_publisher")
        self.publisher = self.create_publisher(PoseStamped, '/voice_command', 10) #此节点不改了,和语音控制的一样
        
        # 订阅导航结果
        self.nav_result_sub = self.create_subscription(
            String,
            '/goal_status',  
            self.nav_result_callback,
            10
        )
        
        self.timer = self.create_timer(0.1, self.send_pose)
        self.x = 0.0
        self.z = 0.0
        self.goal_pose = PoseStamped()
        self.readflag = 0 #0不改变，1发生改变
        self.readreold = None #读取信息的内容
        self.active_goal = False  # 标记当前是否有活跃的目标点
        self.goal_name = None
    
        self.file_path1 = os.path.expanduser('/root/ROS_UDP/ROS2cc.txt') #只写
        self.file_path2 = os.path.expanduser('/root/ROS_UDP/ROS2.txt') #只读
        #self.get_logger().info(f"文件路径: {self.file_path1}")
        
        
        
    def nav_result_callback(self, msg):
        """Navigation result callback function"""
        if msg.data == "SUCCEEDED" and self.active_goal:
            self.get_logger().info("The target point has been reached!")
            self.write_file(self.goal_name+"Get")
            self.active_goal = False  # 重置状态
                  
        
    def write_file(self, content):
        """Write file (overwrite mode)"""
        try:
            with open(self.file_path1, 'w') as f:
                f.write(content)
            self.get_logger().info(f"success: {content.strip()}")
        except Exception as e:
            self.get_logger().error(f"fail: {str(e)}")
            
    def read_file(self):
        """Read file"""
        try:
            with open(self.file_path2, 'r') as f:
                content = f.read()
            # self.get_logger().info(f"文件内容: {content.strip()}")
            return content
        except FileNotFoundError:
            pass #不用输出信息
            #self.get_logger().error(f"file does not exist: {self.file_path}")
        except Exception as e:
            self.get_logger().error(f"fail to read file: {str(e)}")
        return None
    

    def send_pose(self):
        read_result = self.read_file()
        if read_result != self.readreold and read_result!=None:
            self.readflag = 1
            self.readreold = read_result
            print(read_result)
        
        if self.readflag == 1:
            self.readflag = 0
            print("MSG:"+read_result)
            if re.search("APoint",read_result):
                #导航去1点
                self.goal_pose.header.frame_id = "map"
                self.goal_pose.pose.position.x = 1.4076685905456543
                self.goal_pose.pose.position.y = 0.038116052746772766
                self.goal_pose.pose.orientation.w = 1.0
                self.publisher.publish(self.goal_pose)
                self.active_goal = True
                self.goal_name ="APoint"
            if re.search("BPoint",read_result):
                #导航去2点
                self.goal_pose.header.frame_id = "map"
                self.goal_pose.pose.position.x = 2.610774040222168
                self.goal_pose.pose.position.y = 0.15808261930942535
                self.goal_pose.pose.orientation.w = 1.0
                self.publisher.publish(self.goal_pose)
                self.active_goal = True
                self.goal_name ="BPoint"
            if re.search("CPoint",read_result):
                #导航去3点
                self.goal_pose.header.frame_id = "map"
                self.goal_pose.pose.position.x = 1.5
                self.goal_pose.pose.position.y = 0.0
                self.goal_pose.pose.orientation.w = 1.0
                self.publisher.publish(self.goal_pose)
                self.active_goal = True
                self.goal_name ="CPoint"
            if re.search("DPoint",read_result):
                #导航去4点
                self.goal_pose.header.frame_id = "map"
                self.goal_pose.pose.position.x = 1.5
                self.goal_pose.pose.position.y = 0.0
                self.goal_pose.pose.orientation.w = 1.0
                self.publisher.publish(self.goal_pose)
                self.active_goal = True
                self.goal_name ="DPoint"
            if re.search("EPoint",read_result):
                #导航去5点
                self.goal_pose.header.frame_id = "map"
                self.goal_pose.pose.position.x = 1.5
                self.goal_pose.pose.position.y = 0.0
                self.goal_pose.pose.orientation.w = 1.0
                self.publisher.publish(self.goal_pose)
                self.active_goal = True
                self.goal_name ="EPoint"
        



def main(args=None):
    if os.path.exists('/root/ROS_UDP/ROS2.txt'):
        os.remove('/root/ROS_UDP/ROS2.txt')
    if os.path.exists('/root/ROS_UDP/ROS2cc.txt'):
        os.remove('/root/ROS_UDP/ROS2cc.txt')
        
    rclpy.init(args=args)
    node = AgentCtrlSendGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        os.remove("/root/ROS_UDP/ROS2cc.txt")
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.remove("/root/ROS_UDP/ROS2cc.txt")
