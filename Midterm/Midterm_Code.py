import socket
import sys
import time
import math
import numpy as np
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1

IP_ADDRESS = "192.168.0.210" #'192.168.0.202'


positions = {}
rotations = {}


# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    # Position and rotation received
    positions[robot_id] = position
    # The rotation is in quaternion. We need to convert it to euler angles

    rotx, roty, rotz = quaternion_to_euler_angle_vectorized1(rotation_quaternion)

    rotations[robot_id] = rotz






if __name__ == "__main__":
    clientAddress = "192.168.0.27"
    optitrackServerAddress = "192.168.0.4"
    robot_id = 210

    
    #robot_control_code 
    # Connect to the robot
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((IP_ADDRESS, 5000))
    print('Connected')


    # This will create a new NatNet client
    streaming_client = NatNetClient()
    streaming_client.set_client_address(clientAddress)
    streaming_client.set_server_address(optitrackServerAddress)
    streaming_client.set_use_multicast(True)
    # Configure the streaming client to call our rigid body handler on the emulator to send data out.
    streaming_client.rigid_body_listener = receive_rigid_body_frame

    # Start up the streaming client now that the callbacks are set up.
    # This will run perpetually, and operate on a separate thread.
    is_running = streaming_client.run()


    ## initial parameters for robot  ##
    
    
    
    #####
    midterm_time = 0.0
    total_time = 0.0
    robot_data = []
    destination_data = []
    robot_time = []
    try:
        #midterm_time = 0.0
        c = 180/math.pi
        a = [
            (-2.08727023,-1.6771098375,0.1116547),##origin
            (-2.141333342, -2.34975248, 0.11798380),##right
            (-0.7775433, -2.3567192, 0.108841568),##up
            (-0.7760088, -1.06876254, 0.090535223),##left
            (-2.0491845,-1.0383335,0.1083142690),##down
            (-2.0704936,-1.67319679,0.110856816)#right --> origin
        ]
        a_counter = 0
        while is_running:
            if robot_id in positions:
                #P##############################################################################

                ##### last position
                print('Last position', positions[robot_id], ' rotation', rotations[robot_id])

                #Equation 3.2 from notes with tf = 5.0
                # y = (a1 -a0)/time + a0
                pos_diff1 = tuple(map(lambda i, j: i - j, a[a_counter + 1], a[a_counter]))
                pos_diff2 = tuple(ti/5.0 for ti in pos_diff1)
                Goal = tuple(map(lambda i, j: i + j, pos_diff2, a[a_counter]))
                #pos_difference = a[a_counter + 1] - a[a_counter]
                #Goal = (pos_difference)/2.5 + a[a_counter]


                #Straight-Line Trajectory
                res = tuple(map(lambda i, j: i - j, a[a_counter + 1], Goal))

                #res2 is for difference between current location and end goal
                res2 = tuple(map(lambda i, j: i - j, a[a_counter + 1], positions[robot_id]))
                #prints distance between goal and robot location
                #print('distance', res ) 

                ####arc-tangent equation
                
                alpha = (math.atan2(res[1], res[0]) *180) / math.pi
                print('alpha ', alpha)

                #omega = 300 *  (alpha - rotations[robot_id])
                omega = 100 * ((math.atan2(math.sin((alpha - rotations[robot_id])*(math.pi / 180)), math.cos((alpha - rotations[robot_id])* (math.pi/180))) * 180) / math.pi)
                
                
                v = 1200 * math.sqrt((res2[1] ** 2) + (res2[0]**2))
                #v = 1400
                u = np.array([v - omega, v + omega])

                u[u > 1500] = 1500
                u[u < -1500] = -1500

                #### Send control input to the motors
                command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(u[0], u[0], u[1], u[1])
                s.send(command.encode('utf-8'))

                midterm_time += 0.1
                total_time += 0.1
                print(midterm_time)
                if (midterm_time > 5.0):
                    midterm_time = 0
                    a_counter +=1
                    if (a_counter > 4): # all the coordinates have been visited
                        break
                        
                time.sleep(0.1)
    
        command = 'CMD_MOTOR#00#00#00#00\n'
        print("Time: ",total_time)
        s.send(command.encode('utf-8'))
        streaming_client.shutdown()
        
        s.close()
    except KeyboardInterrupt:
        # STOP
        command = 'CMD_MOTOR#00#00#00#00\n'
        s.send(command.encode('utf-8'))
        streaming_client.shutdown()
        
    
    s.close()


#####################################

