#!/usr/bin/env python


# import ros stuff
import rospy
from sensor_msgs.msg import PointCloud2,LaserScan
from geometry_msgs.msg import Twist,Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
import numpy as np
import math
import sensor_msgs.point_cloud2 as pc2
import laser_geometry.laser_geometry as lg
import sensor_msgs.point_cloud2 as pc2

from sensor_msgs.msg import PointCloud2

from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

import tf2_ros

position_ = Point()
yaw_ = 0
# machine state
state_ = 0
checks_stack=[]
flood=np.array([[14,13,12,11,10,9,8,7,7,8,9,10,11,12,13,14],
        [13,12,11,10,9,8,7,6,6,7,8,9,10,11,12,13],
        [12,11,10,9,8,7,6,5,5,6,7,8,9,10,11,12],
        [11,10,9,8,7,6,5,4,4,5,6,7,8,9,10,11],
        [10,9,8,7,6,5,4,3,3,4,5,6,7,8,9,10],
        [9,8,7,6,5,4,3,2,2,3,4,5,6,7,8,9],
        [8,7,6,5,4,3,2,1,1,2,3,4,5,6,7,8],
        [7,6,5,4,3,2,1,0,0,1,2,3,4,5,6,7],
        [7,6,5,4,3,2,1,0,0,1,2,3,4,5,6,7],
        [8,7,6,5,4,3,2,1,1,2,3,4,5,6,7,8],
        [9,8,7,6,5,4,3,2,2,3,4,5,6,7,8,9],
        [10,9,8,7,6,5,4,3,3,4,5,6,7,8,9,10],
        [11,10,9,8,7,6,5,4,4,5,6,7,8,9,10,11],
        [12,11,10,9,8,7,6,5,5,6,7,8,9,10,11,12],
        [13,12,11,10,9,8,7,6,6,7,8,9,10,11,12,13],
        [14,13,12,11,10,9,8,7,7,8,9,10,11,12,13,14]])


path = np.zeros((16,16))

vert_walls = np.zeros((16,17),dtype=np.int8)
horiz_walls = np.zeros((17,16),dtype = np.int8)

vert_walls[:,0]=1
vert_walls[0,1]=1
vert_walls[:,16]=1
horiz_walls[0,:]=1
horiz_walls[16,:]=1

yaw_precision_ = math.pi / 90 # +/- 2 degree allowed
dist_precision_ = 0.3


regions = {
        'right': 0,

        'front': 0,

        'left': 0,
}

# publishers
pub = None
floor_x = 0
floor_y = 0

L=1
F=1
R=1
left_cell=0
right_cell=0
front_cell=0


X = np.zeros((1,720))
Y = np.zeros((1,720))
point_list =[]


def clbk_laser(msg):
    global regions,point_list,pc
    regions = {
        'right':  min(min(msg.ranges[0:60]), 10),

        'front':  min(min(msg.ranges[330:390]), 10),

        'left':   min(min(msg.ranges[660:719]), 10)
    }



    pc = lp.projectLaser(msg)
    
    

    
        

def global_coord():
    global X,Y,state_,position_,pc,point_list
    
    tf_buffer = tf2_ros.Buffer()

    tf_listener = tf2_ros.TransformListener(tf_buffer)

    transform = tf_buffer.lookup_transform("odom","sensor_laser", rospy.Time(),rospy.Duration(5.0))
    pcd2 = do_transform_cloud(pc, transform)
    
    point_list = pc2.read_points(pcd2)
    x = []
    y = []
    for point in point_list:
        x.append(point[0])
        y.append(point[1])
    X = np.array(x)
    Y = np.array(y)    
    
    X = np.reshape(np.abs((X+1.434)/0.18),(720,1))
    Y = np.reshape(np.abs((Y-1.434)/0.18),(720,1))

def clbk_odom(msg):
    global position_
    global yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


def get_pos_in_grid(position):
	global floor_x,floor_y

	#starting position
	#xs = -1.36
	#ys = 1.26

	#center of first block
	x0 = -1.434
	y0 = 1.434


	delta_x = position.x - x0
	delta_y = position.y - y0


	floor_x = math.floor(abs(delta_x)/0.178)
	floor_y = math.floor(abs(delta_y)/0.178)


	#print("\nCoordinates of bot in the grid\n")
	#print(floor_x)
	#print(floor_y)


def go_straight_ahead():
    global position_,X,Y
    
    #print("STATE:"+str(state_))
    get_pos_in_grid(position_)
    X=floor_x
    Y=floor_y
    if state_==0:
        x=X
        y=Y+1
    if state_==1:
        x=X+1
        y=Y
    if state_==2:
        x=X
        y=Y-1
    if state_==3:
        x=X-1
        y=Y
    


    des_pos=Point()
    des_pos.x=-1.35+x*0.18
    des_pos.y=1.35-y*0.18
    rate=rospy.Rate(10)


    if state_==0 or state_==2:

        while math.sqrt(pow(des_pos.y - position_.y, 2))>0.01:
        

            fix_yaw(state_)
            msg = Twist()
            msg.linear.x=0.4
            pub.publish(msg)
            rate.sleep()

    else:

        while math.sqrt(pow(des_pos.x - position_.x, 2))>0.01:
        

            fix_yaw(state_)
            msg = Twist()
            msg.linear.x=0.4
            pub.publish(msg)
            rate.sleep()
    get_pos_in_grid(position_)
    

    msg=Twist()
    pub.publish(msg)
    #maze_update()

    
    #fix_yaw(state_)

    get_min()




def fix_yaw(state):

    global yaw_, yaw_precision_,state_

    current_yaw = yaw_
    if state_==0:
        desired_yaw = 0.0
    if state_==1:
        desired_yaw = 1.57
    if state_==2:
        desired_yaw = 3.14
    if state_==3:
        desired_yaw = -1.57

    if state_==2:
        while math.fabs(desired_yaw-math.fabs(yaw_)) > yaw_precision_:
            err_yaw=desired_yaw-math.fabs(yaw_)
            #print("orientation: "+str(yaw_))
            msg = Twist()
            if yaw_<0:

                msg.angular.z = -0.4 if err_yaw > 0 else 0.4
                pub.publish(msg)
            else:



                msg.angular.z = 0.4 if err_yaw > 0 else -0.4
                pub.publish(msg)

    else:


        while math.fabs(desired_yaw-yaw_) > yaw_precision_:
            #print("orientation: "+str(yaw_))

            err_yaw=desired_yaw-yaw_
            msg = Twist()
            msg.angular.z = 0.4 if err_yaw > 0 else -0.4
            pub.publish(msg)


    msg=Twist()
    pub.publish(msg)
def turn_left():

    global yaw_, yaw_precision_,state_
    rospy.sleep(0.05)
    current_yaw = yaw_
    if state_==0:
        desired_yaw = 1.57
    if state_==1:
        desired_yaw = 3.14
    if state_==2:
        desired_yaw = -1.57
    if state_==3:
        desired_yaw = 0.0



    

    while math.fabs(desired_yaw-yaw_) > yaw_precision_:


        err_yaw = math.fabs(desired_yaw-yaw_)
        msg = Twist()
        msg.angular.z=4.3*err_yaw
        pub.publish(msg)


    msg=Twist()
    pub.publish(msg)
    state_+=1
    if state_>3:
        state_=0
def turn_right():

    global yaw_, yaw_precision_,state_
    rospy.sleep(0.05)
    current_yaw = yaw_
    if state_==0:
        desired_yaw = -1.57
    if state_==1:
        desired_yaw = 0.0
    if state_==2:
        desired_yaw = 1.57
    if state_==3:
        desired_yaw = -3.14




    while math.fabs(desired_yaw-yaw_) > yaw_precision_:


        err_yaw = math.fabs(desired_yaw-yaw_)
        msg = Twist()
        msg.angular.z=-4.3*err_yaw #0.7
        pub.publish(msg)


    msg=Twist()
    pub.publish(msg)
    state_-=1
    if state_<0:
        state_=3
def u_turn():
    turn_right()
    rospy.sleep(0.1)
    turn_right()
    

def get_min():
    global flood,floor_x,floor_y
    get_pos_in_grid(position_)
    x0 = floor_x
    y0 = floor_y
    
    x1 = x0
    y1 = y0 + 1

    x2 = x0
    y2 = y0 - 1

    x3 = x0 + 1
    y3 = y0

    x4 = x0 - 1
    y4 = y0

    accessible_values = []
    accessible_pos = []

    if isAx(x1,y1,x0,y0):
        accessible_values.append(flood[int(y1)][int(x1)])
        accessible_pos.append([x1,y1])
    if isAx(x2,y2,x0,y0):
        accessible_values.append(flood[int(y2)][int(x2)])
        accessible_pos.append([x2,y2])
    if isAx(x3,y3,x0,y0):
        accessible_values.append(flood[int(y3)][int(x3)])
        accessible_pos.append([x3,y3])
    if isAx(x4,y4,x0,y0):
        accessible_values.append(flood[int(y4)][int(x4)])
        accessible_pos.append([x4,y4])

    min_ = min(accessible_values)
    min_index = accessible_values.index(min_)
    min_pos = accessible_pos[min_index]

    x = min_pos[0]
    y = min_pos[1]

    return x,y



    
def go_to_min():
    global floor_x,floor_y,state_
    get_pos_in_grid(position_)
    x0 = floor_x
    y0 = floor_y
    x,y = get_min()
    get_walls()
    if state_==0:
        if y>y0:
            go_straight_ahead()
        if x>x0:
            
            turn_left()
        if x<x0:
            turn_right()
        if y<y0:
            u_turn()
        
    if state_==1:
        if x>x0:
            go_straight_ahead()
        if y<y0:
            turn_left()

        if y>y0:
            turn_right()
                
        if x<x0:
            u_turn()
    if state_==2:
        
        if y<y0:
            go_straight_ahead()
        
        if x<x0:
            turn_left()
        if x>x0:
            turn_right()
        if y>y0:
            u_turn()

    if state_==3:
        if x<x0:
            go_straight_ahead()
        if y>y0:
            turn_left()
        if y<y0:
            turn_right()
        if x>x0:
            u_turn()

def get_max():
    global flood,floor_x,floor_y
    get_pos_in_grid(position_)
    x0 = floor_x
    y0 = floor_y
    
    x1 = x0
    y1 = y0 + 1

    x2 = x0
    y2 = y0 - 1

    x3 = x0 + 1
    y3 = y0

    x4 = x0 - 1
    y4 = y0

    accessible_values = []
    accessible_pos = []

    if isAx(x1,y1,x0,y0):
        accessible_values.append(flood[int(y1)][int(x1)])
        accessible_pos.append([x1,y1])
    if isAx(x2,y2,x0,y0):
        accessible_values.append(flood[int(y2)][int(x2)])
        accessible_pos.append([x2,y2])
    if isAx(x3,y3,x0,y0):
        accessible_values.append(flood[int(y3)][int(x3)])
        accessible_pos.append([x3,y3])
    if isAx(x4,y4,x0,y0):
        accessible_values.append(flood[int(y4)][int(x4)])
        accessible_pos.append([x4,y4])

    max_ = max(accessible_values)
    max_index = accessible_values.index(max_)
    max_pos = accessible_pos[max_index]

    x = max_pos[0]
    y = max_pos[1]

    return x,y



    
def go_to_max():
    global floor_x,floor_y,state_
    get_pos_in_grid(position_)
    x0 = floor_x
    y0 = floor_y
    x,y = get_max()
    get_walls()
    if state_==0:
        if y>y0:
            go_straight_ahead()
        if x>x0:
            
            turn_left()
        if x<x0:
            turn_right()
        if y<y0:
            u_turn()
        
    if state_==1:
        if x>x0:
            go_straight_ahead()
        if y<y0:
            turn_left()

        if y>y0:
            turn_right()
                
        if x<x0:
            u_turn()
    if state_==2:
        
        if y<y0:
            go_straight_ahead()
        
        if x<x0:
            turn_left()
        if x>x0:
            turn_right()
        if y>y0:
            u_turn()

    if state_==3:
        if x<x0:
            go_straight_ahead()
        if y>y0:
            turn_left()
        if y<y0:
            turn_right()
        if x>x0:
            u_turn()

    get_max() 
        
        





def get_walls():
    global X,Y,vert_walls,horiz_walls
    global_coord()
    X_m = np.around(X)
    Y_n = np.around(Y)

    for i in range(720):
        if math.fabs(X_m[i]-X[i])<0.3 and math.fabs(Y_n[i]-Y[i])<0.3:
            continue
        elif math.fabs(X_m[i]-X[i])<0.3 and math.fabs(Y_n[i]-Y[i])>0.3:
            vert_walls[int(math.floor(Y[i])),int(X_m[i])]=1
        elif math.fabs(Y_n[i]-Y[i])<0.3 and math.fabs(X_m[i]-X[i])>0.3:
            horiz_walls[int(Y_n[i]),int(math.floor(X[i]))]=1
            
    #print(vert_walls)
    #print(horiz_walls)





def get_LRF():
    global L,R,F
    L=0
    R=0
    F=0

    if regions['left']<=0.13:
        L=1
    if regions['right']<=0.13:
        R=1
    if regions['front']<=0.13:
        F=1

    #print("L: "+str(L))
    #print("R: "+str(R))
    #print("F: "+str(F))



def isAx(x,y,xd,yd):
    # source block x y destination block xd,yd
    if xd < 0 or yd < 0:
        return (False)

    if xd > 15 or yd > 15:
        return (False)




    elif (x==xd):

        if(yd>y):

            if horiz_walls[int(yd),int(x)]:
                
                return (False)
            else:
                
                return(True)
        else:

            if horiz_walls[int(y),int(x)]:

                return (False)
            else:


                return(True)


    elif (y==yd):

        if(x<xd):


            if vert_walls[int(y),int(xd)]:
                return (False)
            else:
                return (True)
        else:

            if vert_walls[int(y),int(x)]:
                return (False)
            else:
                return (True)





def check_neigbours3(ele):
    global floor_x,floor_y
    get_pos_in_grid(position_)
    x = ele[0]
    y = ele[1]
    

    #neighbours
    x0 = x
    y0 = y + 1

    x1 = x
    y1 = y - 1

    x2 = x + 1
    y2 = y

    x3 = x - 1
    y3 = y

    
    surr_pos=[]
    access_values =[]




    if isAx(x,y,x0,y0):
        access_values.append(flood[int(y0)][int(x0)])
    if y0>=0 and ((x0!=8.0 or y0!=8.0)) and x0<16 and y0<16:


        surr_pos.append([x0,y0])




    if isAx(x,y,x2,y2):
        access_values.append(flood[int(y2)][int(x2)])
    if x2>=0 and ((x2!=8.0 or y2!=8.0)) and x2<16 and y2<16:
        surr_pos.append([x2,y2])

    if isAx(x,y,x3,y3):
        access_values.append(flood[int(y3)][int(x3)])
    if x3>=0 and ((x3!=8.0 or y3!=8.0)) and x3<16 and y3<16:
        surr_pos.append([x3,y3])

    if isAx(x,y,x1,y1):

        access_values.append(flood[int(y1)][int(x1)])
    if y1>=0 and ((x1!=8.0 or y1!=8.0)) and x1<16 and y1<16:


        surr_pos.append([x1,y1])


    
    #print("ACCESSIBLE: ")
    #print(access_values)


    current_value = flood[int(y)][int(x)]

    flag=0

    minimum= min(access_values)
    min_pos = access_values.index(minimum)
    if current_value != minimum +1:
        flag=1



    return flag,surr_pos, minimum, min_pos












def floodfill():

    global flood,checks_stack

    #current position
    global floor_x,floor_y


    get_pos_in_grid(position_)

    start =[floor_x,floor_y]



    get_min()

  
    while (floor_x != 8.0 or floor_y != 7.0):
        start =[floor_x,floor_y]
        
        checks_stack.append(start)
        #print(checks_stack)
        get_walls()
        while checks_stack:

            #print("CHECKING")
            #print(flood)
        
            check_ele = checks_stack.pop()
            #print("POPPED:")
            #print(check_ele)
       

 
            
 


            flag,surr_pos,mini,access_neighbour = check_neigbours3(check_ele)
            #print("FLAG: "+str(flag))

            if flag:



                flood[int(check_ele[1])][int(check_ele[0])] = mini + 1
            


                checks_stack.extend(surr_pos)
                          


                #print("pushed neighbour")
                #print(checks_stack)
                
       


        
        
        
        go_to_min()
        get_pos_in_grid(position_)
        
        start =[floor_x,floor_y]


                    
       


        
        







def get_path():
    global flood,path
    x=0
    y=0

    path[0][0]=1
    
    while x!=7 or y!=7:
        current_cell = flood[y][x]
        x0 = x
        y0 = y + 1

        x1 = x
        y1 = y - 1

        x2 = x + 1
        y2 = y

        x3 = x - 1
        y3 = y
        #print("X: "+str(x))
        #print("Y: "+str(y))  
        if isAx(x,y,x0,y0) and flood[y0][x0]==current_cell-1:
            path[y0][x0]=1
            #print("X0: "+str(x0))
            #print("Y0: "+str(y0))            
            x=x0
            y=y0
        if isAx(x,y,x1,y1) and flood[y1][x1]==current_cell-1:
            path[y1][x1]=1
            #print("X1: "+str(x1))
            #print("Y1: "+str(y1))             
            x=x1
            y=y1
        if isAx(x,y,x2,y2) and flood[y2][x2]==current_cell-1:
            path[y2][x2]=1
            #print("X2: "+str(x2))
            #print("Y2: "+str(y2))  
            x=x2
            y=y2
        if isAx(x,y,x3,y3) and flood[y3][x3]==current_cell-1:
            path[y3][x3]=1
            #print("X3: "+str(x3))
            #print("Y3: "+str(y3))  
            x=x3
            y=y3
    #print(path)


def go_back():
    global path,position_

    while floor_x!=0.0 or floor_y!=0.0:
        get_pos_in_grid(position_)
        get_LRF()
        current_cell = flood[int(floor_y)][int(floor_x)]
        if L:
            left_cell=0
        else:
            if state_==0:
                left_cell = path[int(floor_y)][int(floor_x)+1]
            if state_==1:
                left_cell = path[int(floor_y)-1][int(floor_x)]
            if state_==2:
                left_cell = path[int(floor_y)][int(floor_x)-1]
            if state_==3:
                left_cell = path[int(floor_y)+1][int(floor_x)]
        if R:

            right_cell=0
        else:
            if state_==0:
                right_cell = path[int(floor_y)][int(floor_x)-1]
            if state_==1:
                right_cell = path[int(floor_y)+1][int(floor_x)]
            if state_==2:
                right_cell = path[int(floor_y)][int(floor_x)+1]
            if state_==3:
                right_cell = path[int(floor_y)-1][int(floor_x)]

        if F:

            front_cell=0
        else:
            if state_==0:
                front_cell = path[int(floor_y)+1][int(floor_x)]
            if state_==1:
                front_cell = path[int(floor_y)][int(floor_x)+1]
            if state_==2:
                front_cell = path[int(floor_y)-1][int(floor_x)]
            if state_==3:
                front_cell = path[int(floor_y)][int(floor_x)-1]

        if front_cell == 1:
            go_straight_ahead()
        if left_cell == 1:
            turn_left()
            go_straight_ahead()
      
        if right_cell == 1:
            turn_right()
            go_straight_ahead()
    u_turn()
    
            
def fast_run():
    global path,position_
    
    while floor_x!=7.0 or floor_y!=7.0:
        get_pos_in_grid(position_)
        get_LRF()
        current_cell = flood[int(floor_y)][int(floor_x)]
        if L:
            left_cell=0
        else:
            if state_==0:
                left_cell = path[int(floor_y)][int(floor_x)+1]
            if state_==1:
                left_cell = path[int(floor_y)-1][int(floor_x)]
            if state_==2:
                left_cell = path[int(floor_y)][int(floor_x)-1]
            if state_==3:
                left_cell = path[int(floor_y)+1][int(floor_x)]
        if R:

            right_cell=0
        else:
            if state_==0:
                right_cell = path[int(floor_y)][int(floor_x)-1]
            if state_==1:
                right_cell = path[int(floor_y)+1][int(floor_x)]
            if state_==2:
                right_cell = path[int(floor_y)][int(floor_x)+1]
            if state_==3:
                right_cell = path[int(floor_y)-1][int(floor_x)]

        if F:

            front_cell=0
        else:
            if state_==0:
                front_cell = path[int(floor_y)+1][int(floor_x)]
            if state_==1:
                front_cell = path[int(floor_y)][int(floor_x)+1]
            if state_==2:
                front_cell = path[int(floor_y)-1][int(floor_x)]
            if state_==3:
                front_cell = path[int(floor_y)][int(floor_x)-1]

        if front_cell == 1:
            go_straight_ahead()
        if left_cell == 1:
            turn_left()
            go_straight_ahead()
      
        if right_cell == 1:
            turn_right()
            go_straight_ahead()

    go_straight_ahead()
    rospy.sleep(3)
    u_turn()
    go_straight_ahead()







if __name__=="__main__":
    rospy.init_node('maze_solver')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    lp = lg.LaserProjection()

    sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser,queue_size=10)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    pc_pub = rospy.Publisher('/laser2pc',PointCloud2,queue_size=1)
    rate = rospy.Rate(10)
    rospy.sleep(5)
    x = raw_input("PRESS ENTER TO START!")
    if not x:
        floodfill()
    #print(flood)
    get_path()
    #print(path)
    turn_left()
    
    go_straight_ahead()
    go_back()

    
    while not rospy.is_shutdown():

        x = raw_input("PRESS ENTER FOR FAST RUN!:  \n"+"Press Ctrl-C to Exit")
        if not x:


        
            fast_run()
        else:
            break
        
        go_back()
        