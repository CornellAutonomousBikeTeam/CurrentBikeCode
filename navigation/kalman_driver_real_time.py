import csv
import matplotlib
import numpy as np
import gps_assisted_simulator_node 
import subprocess

import matplotlib
from matplotlib import pyplot as plt

import kalman
import requestHandler


#First we must make sure we are in the right directory in ROS
#subprocess.call(["cd"]) 
#Call the start script from the terminal to initialize ROS nodes; '&' - runs script in background
#subprocess.call("bash start.sh &", shell=True)    
def kalman(data)
    #Important fields from data
    latitude = data.data[0] # In degrees
    longitude = data.data[1]
    psi = data.data[7] # psi = heading in radians
    velocity = data.data[8]

    # Converts lat long to x,y 
    x, y = requestHandler.math_convert(float(latitude), float(longitude)) 

    # gps current state - only relevant fields
    gps_curr = [x, y, float(psi), float(velocity)]
    # The Kalman filter wants the GPS data in matrix form
    gps_matrix = np.matrix(gps_curr)
    # Run the Kalman filter
    output_matrix = kalman.kalman_no_loop(gps_matrix, np.matrix([[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])) 
    #Change output_matrix to a standard array for publishing
    kalman_state = output_matrix.flatten()
    #save gps state values for later plotting
    gps_data.append(gps_matrix)
    #save predicted state values for later plotting
    kalman_data.append(output_matrix) 



def listener():
    pub = rospy.Publisher('kalman_state', Float32MultiArray, queue_size=10)
    rospy.init_node('kalman', anonymous=True)
    rospy.Subscriber("gps", Float32MultiArray, kalman)
    rate = rospy.Rate(100)
    #Run until the nodes are shutdown (end.sh run OR start.sh was killed)
    while not rospy.is_shutdown():
        dim = [MultiArrayDimension('data', 4, 4)]
        layout = MultiArrayLayout(dim, 0)
        #rospy.loginfo("({:.4f}, {:.4f}), heading {:.4f}, velocity {:.4f}, lean angle/rate {:.4f}/{:.4f}, steering {:.4f} [gps_assisted_simulator_node]".format(new_bike.xB, new_bike.yB, new_bike.psi, new_bike.v, new_bike.phi, new_bike.w_r, new_bike.delta))
        #rospy.loginfo(l)
        pub.publish(layout, kalman_state)
        rate.sleep()
    print 'Test was terminated'
    # Plot the GPS data
    plt.scatter(gps_data[:,0], gps_data[:,1], c='r')
    # Plot the Kalman output
    plt.scatter(kalman_data[:,0], kalman_data[:,1])
    # Show everything
    plt.show()

if __name__ == '__main__':
    #kalman/gps data saved as we go for later plotting
    kalman_data = []
    gps_data = []
    #state that is published to ROS
    kalman_state = []
    listener()
