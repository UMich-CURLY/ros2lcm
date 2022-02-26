import rospy
import yaml
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
import csv
from nav_msgs.srv import GetPlan

class tour_planner():
	selection_done = False
	selected_points = [];
	flag_use_vulcan_path = True
	def __init__(self):
		if (not rospy.core.is_initialized()):
			rospy.init_node("get_points",anonymous=False)
		_sensor_rate = 50  # hz
		self._r = rospy.Rate(_sensor_rate)
		self._pub_markers_initial = rospy.Publisher("~selected_points", MarkerArray, queue_size = 1)
		self._pub_plan_3d_robot_1 = rospy.Publisher("robot_1/plan_3d", numpy_msg(Floats),queue_size = 1)
		self._pub_plan_robot_1 = rospy.Publisher("robot_1/global_plan", Path, queue_size=1)
		with open('./scripts/all_points.csv') as csvfile:
			spamreader = csv.reader(csvfile, delimiter=',')
			for row in spamreader:
				map_points = list(map(float,row))
				self.selected_points.append(list(map(float,row)))
		self.selected_points_pose_stamped = []
		pose_temp = PoseStamped()
		pose_temp.pose.position.x = self.selected_points[0][0]
		pose_temp.pose.position.y = self.selected_points[0][1]

		self.selected_points_pose_stamped.append(pose_temp)

		pose_temp_1 = PoseStamped()
		pose_temp_1.pose.position.x = self.selected_points[1][0]
		pose_temp_1.pose.position.y = self.selected_points[1][1]

		self.selected_points_pose_stamped.append(pose_temp_1)

		self.path_srv = GetPlan()
		self.vulcan_graph_srv = GetPlan()
		self.get_plan = rospy.ServiceProxy('/move_base/NavfnROS/make_plan', GetPlan)
		self.get_vulcan_plan = rospy.ServiceProxy('/get_distance', GetPlan)
		print("Created tour planner object")
		self._r.sleep()



	def generate_distance_matrix_pose_stamped(self):
		distance_matrix = []
		for start_point in self.selected_points_pose_stamped:
			distances = []
			for end_point in self.selected_points_pose_stamped:
				self.path_srv.start = start_point
				self.path_srv.goal = end_point
				self.path_srv.tolerance = .5
				path = self.get_plan(self.path_srv.start, self.path_srv.goal, self.path_srv.tolerance)
				prev_x = 0.0
				prev_y = 0.0
				total_distance = 0.0
				first_time = True
				print (len(distance_matrix))
				for current_point in path.plan.poses:
					x = current_point.pose.position.x
					y = current_point.pose.position.y
					if not first_time:
						total_distance += math.hypot(prev_x - x, prev_y - y) 
					else:
						first_time = False
					prev_x = x
					prev_y = y
				distances.append(total_distance)
			distance_matrix.append(distances)
		print("iN CREATE MODE, FOUND THE DISTANCE MATRIX ", distance_matrix)
		return distance_matrix

	def generate_distance_vulcan_grid(self):
		print("In Vulcan grid distance callback")
		distance_matrix = []
		for start_point in self.selected_points_pose_stamped:
			distances = []
			for end_point in self.selected_points_pose_stamped:
				self.vulcan_graph_srv.start = start_point
				self.vulcan_graph_srv.goal = end_point
				self.vulcan_graph_srv.tolerance = .5
				path = self.get_vulcan_plan(self.vulcan_graph_srv.start, self.vulcan_graph_srv.goal, self.vulcan_graph_srv.tolerance)
				total_total_distance = 0.0
				for i in range(0,len(path.plan.poses)-1):
					self.path_srv.start = path.plan.poses[i]
					self.path_srv.goal = path.plan.poses[i+1]
					self.path_srv.tolerance = .5
					print("before calling the service")
					a_star_path = self.get_plan(self.path_srv.start, self.path_srv.goal, self.path_srv.tolerance)
					print(a_star_path.plan.poses.size())
					# prev_x = 0.0
					# prev_y = 0.0
					# total_distance = 0.0
					# first_time = True
					# for current_point in a_star_path.plan.poses:
					# 	x = current_point.pose.position.x
					# 	y = current_point.pose.position.y
					# 	if not first_time:
					# 		total_distance += math.hypot(prev_x - x, prev_y - y) 
					# 	else:
					# 		first_time = False
					# 	prev_x = x
					# 	prev_y = y
					# total_total_distance = total_total_distance + total_distance
				total_total_distance = path.plan.poses[0].pose.position.x
				distances.append(total_total_distance)
			distance_matrix.append(distances)
		print("iN CREATE MODE, FOUND THE DISTANCE MATRIX ", distance_matrix)
		return distance_matrix

def main():
	tour_plan = tour_planner()
	tour_plan.generate_distance_matrix_pose_stamped()
	tour_plan.generate_distance_vulcan_grid()
	while not rospy.is_shutdown():
		rospy.spin()

if __name__ == "__main__":
	main()
