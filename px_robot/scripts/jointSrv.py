"""
Allows to use the service dynamixel_command 
"""
import rospy
import time
# from std_msgs.msg import String
from dynamixel_workbench_msgs.srv import DynamixelCommand

__author__ = "F Gonzalez, S Realpe, JM Fajardo"
__credits__ = ["Felipe Gonzalez", "Sebastian Realpe", "Jose Manuel Fajardo", "Robotis"]
__email__ = "fegonzalezro@unal.edu.co"
__status__ = "Test"

def jointCommand(command, id_num, addr_name, value, time):
    #rospy.init_node('joint_node', anonymous=False)
    rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
    try:        
        dynamixel_command = rospy.ServiceProxy(
            '/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        result = dynamixel_command(command,id_num,addr_name,value)
        rospy.sleep(time)
        return result.comm_result
    except rospy.ServiceException as exc:
        print(str(exc))

if __name__ == '__main__':
    try:
        # Goal_Position (0,1023)
        # Torque_Limit (0,1023)
        jointCommand('', 6, 'Torque_Limit', 600, 0)
        jointCommand('', 7, 'Torque_Limit', 500, 0)
        jointCommand('', 8, 'Torque_Limit', 400, 0)
        jointCommand('', 9, 'Torque_Limit', 400, 0)
        jointCommand('', 9, 'Goal_Position', 512, 0.5)
        jointCommand('', 8, 'Goal_Position', 512, 0.5)
        time.sleep(0.5)
        jointCommand('', 7, 'Goal_Position', 512, 0.5)
        time.sleep(0.5)
        jointCommand('', 6, 'Goal_Position', 512, 0.5)
        jointCommand('', 9, 'Goal_Position', 750, 1)
        time.sleep(0.2)
        jointCommand('', 7, 'Goal_Position', 512, 1)
        time.sleep(0.5)
        jointCommand('', 8, 'Goal_Position', 240, 0.5)
        jointCommand('', 6, 'Goal_Position', 512, 0.5)
        
    except rospy.ROSInterruptException:
        pass