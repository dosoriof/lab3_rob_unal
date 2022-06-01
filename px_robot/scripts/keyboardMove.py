import rospy
import numpy
import time
import termios, sys, os
import numpy as np
import roboticstoolbox as rtb
from spatialmath import *
from spatialmath.base import *
from dynamixel_workbench_msgs.srv import DynamixelCommand
TERMIOS = termios

## This function creates de service to control de dynamixel motors
## First we wait for the service to be available in case it its
## been used by another process.
## Next we initialize the service inside a try, in case it fails
## the whole process don’t fail.
## Then we send the command with the parameters we called the function, 
## and wait a time for the command to be executed, 
## then we return the result of the command. 
def jointCommand(command, id_num, addr_name, value, time):
    rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
    try:        
        dynamixel_command = rospy.ServiceProxy(
            '/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        result = dynamixel_command(command,id_num,addr_name,value)
        rospy.sleep(time)
        return result.comm_result
    except rospy.ServiceException as exc:
        print(str(exc))

## This function reads the key is pressed and returns, to use this function 
## it is necessary the command window in which was called the script is in focus.
def getkey():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    new = termios.tcgetattr(fd)
    new[3] = new[3] & ~TERMIOS.ICANON & ~TERMIOS.ECHO
    new[6][TERMIOS.VMIN] = 1
    new[6][TERMIOS.VTIME] = 0
    termios.tcsetattr(fd, TERMIOS.TCSANOW, new)
    c = None
    try:
        c = os.read(fd, 1)
    finally:
        termios.tcsetattr(fd, TERMIOS.TCSAFLUSH, old)
    return c

## This function prints the name of 
## the action that will be executed
def printMov(numMov):
    if numMov == 1:
        print("trax")
    elif numMov == 2:
        print("tray")
    elif numMov == 3:
        print("traz")
    elif numMov == 4:
        print("rot")

## This function configures the maximum speed in all the joints
## to ensure the motors will not move to fast and prevent 
## collisions or accidents.
def setMovingSpeed(speed):
    for i in range(4):
        jointCommand('',i+1, 'Moving_Speed',speed, 0)   #Cambiar si cambian ID de motores

## This function executes the movement of all joints given 
## the value of all joint objective positions.
def moveLinks(links):
    links_b = round((links-(-5*np.pi/6))*(1023/(5*np.pi/3)))
    for i in range(4):
        jointCommand('',i+1, 'Goal_Position',links_b[i], 0)   #Cambiar si cambian ID de motores

## This function returns the solution for each joint,
## given a transformation matrix, and the desired
## elbow solution, 'down' or 'up'; uses the geometric 
## method and decouple.
def invKinPhantomX(T, elbow):
    l = np.array([14.5, 10.25, 10.25, 9]) # Longitudes eslabones
    
    #Desacople
    Posw = T - (l[3]*T[0:4,2]).reshape(4,1) # MTH Wrist
    
    #q1
    if T[1,3]==0 and T[0,3]==0:
        print('Singular position. q1 can have any value')
        q1 = np.NaN
    else:
        q1 = np.arctan2(T[1,3],T[0,3])
    
    # Solución 2R
    h = Posw[2,3] - l[0]  #Posw[2] - l[0] 
    r = np.sqrt(Posw[0,3]**2 + Posw[1,3]**2) #np.sqrt(Posw[1]**2 + Posw[1,3]**2)
    
    # Solucion de q4
    Rp = (rotz(q1).T).dot(T[0:3,0:3])
    pitch = np.arctan2(Rp[2,0],Rp[0,0])
    
    the3 = np.arccos((r**2+h**2-l[1]**2-l[2]**2)/(2*l[1]*l[2]))

    if not np.isnan(the3): 
        if elbow == 'down':
            the2 = np.arctan2(h,r) - np.arctan2(l[2]*np.sin(the3),l[1]+l[2]*np.cos(the3))
            q2 = -(np.pi/2-the2)
            q3 = the3
            q4 = pitch - q2 - q3 - 2*np.pi
        if elbow == 'up':
            the2 = np.arctan2(h,r) + np.arctan2(l[2]*np.sin(the3),l[1]+l[2]*np.cos(the3))
            q2 = -(np.pi/2-the2)
            q3 = -the3
            q4 = pitch - q2 - q3 - 2*np.pi
    else:
         q1 = np.NaN
         q2 = np.NaN
         q3 = np.NaN
         q4 = np.NaN
        
    
    if (elbow != 'down') and (elbow != 'up'):
        print('Wrong elbow imput - Write up or down')
        q1 = np.NaN
        q2 = np.NaN
        q3 = np.NaN
        q4 = np.NaN
    q_inv = np.array([q1, q2, q3, q4])
    return q_inv 

## This function calculates que requested action in the keyboard, 
## given the values of the action, actual position and direction, 
## return the MTH of the desired position.
def changeT(numMov, T_actual, direction):
    X, Y, Z = transl(T_actual)
    aZ1, aY, aZ2 = tr2eul(T_actual) #verificar que las funciones del toolbox se usen igual en python
    print(aZ2)
    d_tra = 1
    d_rot = 2*np.pi/180

    if numMov == 1:
        X = X + direction*d_tra
        aZ2 = np.arctan2(Y,X)
    if numMov == 2:
        Y = Y + direction*d_tra
        aZ2 = np.arctan2(Y,X)
    if numMov == 3:
        Z = Z + direction*d_tra
    if numMov == 4:
        aY = aY + direction*d_rot
    A = eul2r(aZ1,aY,aZ2)
    print(A)
    b = np.array([[X], [Y], [Z]])
    print(b)
    T_actual = rt2tr(eul2r(aZ1,aY,aZ2),b)
    print(b)
    return T_actual

## This function evaluates if the desires position is 
## achievable and within the max and min values for each joint.
def checkJointsValues(q):
    max = np.deg2rad(150)
    min = np.deg2rad(-150)
    if q[0]<max and q[0]>min:
        q1 = True
    else:
        q1 = False
        print("q1 value out of Limits")
    
    if q[1]<max and q[1]>min:
        q2 = True
    else:
        q2 = False
        print("q2 value out of Limits")

    if q[2]<max and q[2]>min:
        q3 = True
    else:
        q3 = False
        print("q3 value out of Limits")
    
    if q[3]<max and q[3]>min:
        q4 = True
    else:
        q4 = False
        print("q4 value out of Limits")
    
    return q1 and q2 and q3 and q4
    



## In the main function first we define in an array the number of joints and its 
## home an objective position, then we define the ID of the first motor, we 
## assume they are consecutive from there, then inside a try we configure the
## torque for each one of the motors, then inside an infinite loop we wait for
## a key to be pressed, if it is ‘w’ or ‘s’ they change the motor in focus, if it is
## ‘a’ or ‘d’ they change the position of the motor to its home or objective 
##position. 
if __name__ == '__main__':
    nMov = 1
    # T_act = np.dot(transl(18,0,10),troty(-np.pi))
    T_act = transl(18,0,10)@troty(-np.pi)
    print("T")
    print(T_act)
    q_act = invKinPhantomX(T_act, 'up')
    print("q")
    print(q_act)
    print("sirve?")
    print(checkJointsValues(q_act))
    T_act = changeT(nMov, T_act, 1)
    print(T_act)
    try:
        setMovingSpeed(200)
        # Posicion inicial 
        moveLinks(q_act)
        printMov(nMov)
        while 1:
            key = getkey()
            if key == b'w':
                a = nMov+1
                if (a > 4):
                    nMov = 1
                else:
                    nMov = nMov+1
                 printMov(nMov)
            if key == b's':
                a = nMov-1
                if (a < 1):
                    nMov = 4
                else:
                    nMov = nMov-1
                printMov(nMov)
             if key == b'a':
                T_act = changeT(nMov, T_act, -1)
                q_act = invKinPhantomX(T_act, 'up')
                if checkJointsValues(q_act):
                    moveLinks(q_act)
             if key == b'd':
                 T_act = changeT(nMov, T_act, 1)
                q_act = invKinPhantomX(T_act, 'up')
                if checkJointsValues(q_act):
                    moveLinks(q_act)
        
    except rospy.ROSInterruptException:
        pass
