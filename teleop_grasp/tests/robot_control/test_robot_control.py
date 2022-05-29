#!/usr/bin/env python3

from cProfile import label
import os
import sys
import rospy
import rosbag
import bagpy
import matplotlib.pyplot as plt
import pandas as pd

from geometry_msgs.msg import Point, Wrench, Vector3
from gazebo_msgs.srv import ApplyBodyWrench, ApplyBodyWrenchRequest
from franka_controllers.msg import CartesianAdmittanceControllerDebug

def setup_test(filepath, test_id=""):

	test_name = os.path.splitext(filepath.partition("test_")[2])[0]
	test_name = test_name if len(sys.argv) == 1 else sys.argv[1]
	dir_test = os.path.dirname(filepath)
	dir_data = f"{dir_test}/data/{test_id}"
	dir_img  = f"{dir_test}/img"
	os.makedirs(dir_data, exist_ok=True)
	os.makedirs(dir_img, exist_ok=True)

	return test_name, dir_data, dir_img


def apply_wrench(force, torque, duration, reference_point=(0, 0, 0)):

	srv_apply_wrench = rospy.ServiceProxy("/gazebo/apply_body_wrench", ApplyBodyWrench)

	global wrench
	wrench = Wrench(force=Vector3(*force), torque=Vector3(*torque))

	req = ApplyBodyWrenchRequest(
		body_name="panda::panda_link6",
		reference_frame="world",
		reference_point=Point(*reference_point),
		wrench=wrench,
		duration=rospy.Duration(duration) # sec
	)

	rospy.loginfo(f"applying wrench for {duration} seconds...")
	# print(req)
	res = srv_apply_wrench(req)
	rospy.sleep(duration)

	# reset wrench
	wrench = Wrench(force=Vector3(), torque=Vector3())


def update_bag(msg):

	global bag
	bag.write("/cartesian_admittance_controller/debug", msg)
	bag.write("/wrench", globals()["wrench"])


if __name__ == "__main__":

	# setup test; test_name is inferred from filename as "test_<test_name>.py"
	(test_name, dir_data, dir_img) = setup_test(filepath=__file__, test_id="")
	rospy.init_node(f"test_{test_name}")

	# setup rosbag
	bag_path = f"{dir_data}/{test_name}.bag"
	bag = rosbag.Bag(bag_path, "w")
	sub = rospy.Subscriber( # setup sub to write to bag
		"/cartesian_admittance_controller/debug",
		CartesianAdmittanceControllerDebug, update_bag, queue_size=1, tcp_nodelay=True
	)

	# run tests
	rospy.wait_for_service("/gazebo/apply_body_wrench")

	wrench = Wrench(force=Vector3(), torque=Vector3())
	apply_wrench(force=(10, 0, 0), torque=(0, 0, 0), duration=5)
	apply_wrench(force=(0, 5, 0), torque=(0, 0, 0), duration=5)
	apply_wrench(force=(0, 0, 0), torque=(0, 0, 0), duration=5)

	sub.unregister()
	bag.close()

	# plot
	# convert rosbag to pandas dataframe; offset time column to begin at 0
	b = bagpy.bagreader(bag_path)
	csv_filepath = b.message_by_topic(topic="/cartesian_admittance_controller/debug")
	df = pd.read_csv(csv_filepath)
	df['Time'] = df['Time'].apply(lambda x: x - df['Time'][0])
	# print(df.columns)

	plt.style.use("teleop_grasp")

	plt.plot(df["Time"], df["h_e.force.x"], label="x", color="r")
	plt.plot(df["Time"], df["h_e.force.y"], label="y", color="g")
	plt.plot(df["Time"], df["h_e.force.z"], label="z", color="b")
	plt.ylabel("Force [N m]"); plt.xlabel("Time [s]"); plt.legend(); plt.grid()
	# plt.show()
	plt.savefig(f"{dir_img}/{test_name}.pdf")
	print(f"saved image: '{dir_img}/{test_name}.pdf'")
