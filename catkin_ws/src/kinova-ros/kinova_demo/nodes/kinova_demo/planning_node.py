#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from gormetry_msgs.msg import Pose
	
arm_pub = None
ugv_pub = None
object_side = 0
arm_points_completed = 0
arm_points_sent = 0

def camera_callback(data):
	# here the camera has determined where the object of interest is and we need
	#to tell the UGV to go there. Not sure how to line the maps up, that will be tough
	#this is the path that the code will take to tell the UGV where to go
	ugv_desired_pose = Pose.pose()
	#need to fill it here with the actual values
	ugv_pub.publish(ugv_desired_pose)
	pass

#Jordan - this is where you need to fill in the grid logic
def executeGrid():
	desired_arm_pose = Pose.pose()
	if (object_side % 2 == 0): # if we are on an even, we assume that to be the long side
		#make the grid for the long side
		#desired_arm_pose.position.[x,y,z] = 
		#desired_arm_pose.orientation = a quaternion (See Ryan for this, we will likely igrnore)
		arm_pub.publish(desired_arm_pose)
	else:
		#make the grid for the short side
		#desired_arm_pose.position.[x,y,z] = 
		#desired_arm_pose.orientation = a quaternion (See Ryan for this, we will likely igrnore)
		arm_pub.publish(desired_arm_pose)

	global arm_points_sent 
	arm_points_sent += 1

#Need to listen to /ugv_arrived. In this callback, start the logic to make the grid
def ugv_arrived_callback(data):
	#Make grid of Poses to send to Jaco
    #create new pose message for each desired point and publish it to /desired_arm_pose
	executeGrid()

def arm_complete_callback(data):
	global arm_points_completed
	arm_points_completed += 1
	if (arm_points_completed == arm_points_sent):
		# the arm has completed its motion and can tell the ugv to move
		ugv_desired_pose = Pose.pose()
		#need to fill it here with the actual values
		ugv_pub.publish(ugv_desired_pose)

#this will read the overall dimensions from the topic, I am not sure how we will orient our coordinate system with that of the STL
def dimension_callback(dimensions):
	pass

	

# main function that gets called to start the program
if __name__ == '__main__':
	try:
		rospy.init_node('planning_node', anonymous=True)
		arm_pub = rospy.Publisher('/desired_arm_pose', Pose, queue_size = 10)
		ugv_pub = rospy.Publisher('/desired_ugv_pose', Pose, queue_size = 10)
	
	#types of these subscriptions could change
		rospy.Subscriber("/object_location", String, camera_callback)
		rospy.Subscriber("/ugv_arrived", String, ugv_arrived_callback)
		rospy.Subscriber("/arm_point_completed", String, arm_complete_callback)
		rospy.Subscriber("/dimensions", String, dimension_callback)
		rospy.spin()
		# the arm will publish here (or something like it) when if completes each point
	except rospy.ROSInterruptException:
	    pass




'''


/* This function will make a rectangle in the y plane that has dimensions [x_travel x z_travel]
* the x,y,z coordinate is the of the lower fight corner (x is minimized and z is minimized)
*/
void makeRectangle(TrajectoryPoint pointToSend, float x, float z, float x_step, float z_step) {

	//this starts in the lower right corner
	pointToSend.Position.CartesianPosition.X = x;
	pointToSend.Position.CartesianPosition.Z = z;
	sendPointToBot(pointToSend);

	//upper right corner
	pointToSend.Position.CartesianPosition.X = x;
	pointToSend.Position.CartesianPosition.Z = z + z_step;
	sendPointToBot(pointToSend); '''# replace these sendPointToBot with publishing to the /desired_arm_pose topic
'''
	//upper left corner
	pointToSend.Position.CartesianPosition.X = x + x_step;
	pointToSend.Position.CartesianPosition.Z = z + z_step;
	sendPointToBot(pointToSend);

	//lower left corner
	pointToSend.Position.CartesianPosition.X = x + x_step;
	pointToSend.Position.CartesianPosition.Z = z;
	sendPointToBot(pointToSend);

	//return to lower right
	pointToSend.Position.CartesianPosition.X = x;
	pointToSend.Position.CartesianPosition.Z = z;
	sendPointToBot(pointToSend);

    pointToSend.Position.CartesianPosition.X = 0;
	pointToSend.Position.CartesianPosition.Z = 0;
	sendPointToBot(pointToSend);
}

for(float x = min_x_dimension; x <= max_x_dimension; x += x_step) {
	makeRectangle(pointToSend, x, min_z_dimension, x_step, z_step);
}
'''



