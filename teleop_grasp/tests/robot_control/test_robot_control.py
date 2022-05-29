#!/usr/bin/env python3

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

	node_name = os.path.splitext(filepath.partition("test_")[2])[0]
	test_name = node_name if len(sys.argv) == 1 else sys.argv[1]
	dir_test = os.path.dirname(filepath)
	dir_data = f"{dir_test}/data/{test_id}"
	dir_img  = f"{dir_test}/img"
	os.makedirs(dir_data, exist_ok=True)
	os.makedirs(dir_img, exist_ok=True)

	return node_name, test_name, dir_data, dir_img


def apply_wrench(force, torque, duration, reference_point=(0, 0, 0)):

	srv_apply_wrench = rospy.ServiceProxy("/gazebo/apply_body_wrench", ApplyBodyWrench)

	global wrench
	wrench = Wrench(force=Vector3(*force), torque=Vector3(*torque))

	req = ApplyBodyWrenchRequest(
		body_name="panda::panda_link7",
		reference_frame="world",
		reference_point=Point(*reference_point),
		wrench=wrench,
		duration=rospy.Duration(duration) # sec
	)

	rospy.loginfo(f"applying wrench {force, torque} for {duration} seconds...")
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
	(node_name, test_name, dir_data, dir_img) = setup_test(filepath=__file__, test_id="")
	rospy.init_node(f"test_{node_name}")

	bag_path = f"{dir_data}/{test_name}.bag"
	if not os.path.exists(bag_path):

		# setup rosbag
		bag = rosbag.Bag(bag_path, "w")
		sub = rospy.Subscriber( # setup sub to write to bag
			"/cartesian_admittance_controller/debug",
			CartesianAdmittanceControllerDebug, update_bag, queue_size=1, tcp_nodelay=True
		)

		# run tests
		rospy.wait_for_service("/gazebo/apply_body_wrench")

		wrench = Wrench(force=Vector3(), torque=Vector3())
		apply_wrench(force=(5, 0, 0), torque=(0, 0, 0), reference_point=(0, 0, 0.107), duration=5)
		apply_wrench(force=(0, 0, 0), torque=(0, 0, 0), duration=5)
		apply_wrench(force=(0, 0, 0), torque=(1, 0, 0), reference_point=(0, 0, 0.107), duration=5)
		apply_wrench(force=(0, 0, 0), torque=(0, 0, 0), duration=5)

		sub.unregister()
		bag.close()

	# plot

	# convert rosbag to pandas dataframe; offset time column to begin at 0
	b = bagpy.bagreader(bag_path)

	df_ee = pd.read_csv(b.message_by_topic(topic="/cartesian_admittance_controller/debug"))
	df_ee["Time"] = df_ee["Time"].apply(lambda x: x - df_ee["Time"][0])
	print(df_ee.columns)

	df_wrench = pd.read_csv(b.message_by_topic(topic="/wrench"))
	df_wrench["Time"] = df_wrench["Time"].apply(lambda x: x - df_wrench["Time"][0])
	# print(df_wrench.columns)

	plt.style.use("teleop_grasp")

	## force (measured + desired)
	fig, ax = plt.subplots()
	ax.plot(df_ee["Time"], df_ee["h_e.force.x"], label=r"$f_{e,x}$", color="r", alpha=0.25)
	ax.plot(df_ee["Time"], df_ee["h_e.force.y"], label=r"$f_{e,y}$", color="g", alpha=0.25)
	ax.plot(df_ee["Time"], df_ee["h_e.force.z"], label=r"$f_{e,z}$", color="b", alpha=0.25)
	ax.plot(df_wrench["Time"], df_wrench["force.x"], label=r"$f_{d,x}$", color="r")
	ax.plot(df_wrench["Time"], df_wrench["force.y"], label=r"$f_{d,y}$", color="g")
	ax.plot(df_wrench["Time"], df_wrench["force.z"], label=r"$f_{d,z}$", color="b")
	ax.set_ylabel("Force [N m]"); ax.set_xlabel("Time [s]"); ax.legend(); ax.grid()
	fig.savefig(f"{dir_img}/{test_name}-force.pdf")

	## torque (measured + desired)
	fig, ax = plt.subplots()
	ax.plot(df_ee["Time"], df_ee["h_e.torque.x"], label=r"$\mu_{e,x}$", color="r", alpha=0.25)
	ax.plot(df_ee["Time"], df_ee["h_e.torque.y"], label=r"$\mu_{e,y}$", color="g", alpha=0.25)
	ax.plot(df_ee["Time"], df_ee["h_e.torque.z"], label=r"$\mu_{e,z}$", color="b", alpha=0.25)
	ax.plot(df_wrench["Time"], df_wrench["torque.x"], label=r"$\mu_{d,x}$", color="r")
	ax.plot(df_wrench["Time"], df_wrench["torque.y"], label=r"$\mu_{d,y}$", color="g")
	ax.plot(df_wrench["Time"], df_wrench["torque.z"], label=r"$\mu_{d,z}$", color="b")
	ax.set_ylabel("Force [N m]"); ax.set_xlabel("Time [s]"); ax.legend(); ax.grid()
	fig.savefig(f"{dir_img}/{test_name}-torque.pdf")

	## compliant frame: position
	fig, axs = plt.subplots(3, 1, sharex=True)
	axs[0].plot(df_ee["Time"], df_ee["p_d.x"], label=r"$p_{d,x}$", color="r", alpha=0.25)
	axs[0].plot(df_ee["Time"], df_ee["p_c.x"], label=r"$p_{c,x}$", color="r")
	axs[1].plot(df_ee["Time"], df_ee["p_d.y"], label=r"$p_{d,y}$", color="g", alpha=0.25)
	axs[1].plot(df_ee["Time"], df_ee["p_c.y"], label=r"$p_{c,y}$", color="g")
	axs[2].plot(df_ee["Time"], df_ee["p_d.z"], label=r"$p_{d,z}$", color="b", alpha=0.25)
	axs[2].plot(df_ee["Time"], df_ee["p_c.z"], label=r"$p_{c,z}$", color="b")
	for ax in axs.flat:
		ax.grid(); ax.legend(loc="upper right")
	fig.supylabel("Position [m]"); fig.supxlabel("Time [s]")
	fig.savefig(f"{dir_img}/{test_name}-pos.pdf")

	## compliant frame: orientation
	fig, axs = plt.subplots(3, 1, sharex=True)
	axs[0].plot(df_ee["Time"], df_ee["eps_d.x"], label=r"$\varepsilon_{d,x}$", color="r", alpha=0.25)
	axs[0].plot(df_ee["Time"], df_ee["eps_c.x"], label=r"$\varepsilon_{c,x}$", color="r")
	axs[1].plot(df_ee["Time"], df_ee["eps_d.y"], label=r"$\varepsilon_{d,y}$", color="g", alpha=0.25)
	axs[1].plot(df_ee["Time"], df_ee["eps_c.y"], label=r"$\varepsilon_{c,y}$", color="g")
	axs[2].plot(df_ee["Time"], df_ee["eps_d.z"], label=r"$\varepsilon_{d,z}$", color="b", alpha=0.25)
	axs[2].plot(df_ee["Time"], df_ee["eps_c.z"], label=r"$\varepsilon_{c,z}$", color="b")
	for ax in axs.flat:
		ax.grid(); ax.legend(loc="upper right")
	fig.supylabel("Orientation"); fig.supxlabel("Time [s]")
	fig.savefig(f"{dir_img}/{test_name}-ori.pdf")

