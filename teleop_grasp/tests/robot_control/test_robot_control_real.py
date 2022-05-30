#!/usr/bin/env python3

import os
import sys
import subprocess
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


if __name__ == "__main__":

	# setup test; test_name is inferred from filename as "test_<test_name>.py"
	(node_name, test_name, dir_data, dir_img) = setup_test(filepath=__file__, test_id="")
	rospy.init_node(f"test_{node_name}")

	info = subprocess.check_output(["rosrun", "dynamic_reconfigure", "dynparam", "get", "/cartesian_admittance_controller"])
	print(info, file=open(f"{dir_data}/{test_name}.txt", "w", encoding="utf-8"))

	bag_path = f"{dir_data}/{test_name}.bag"
	if not os.path.exists(bag_path): # skip if already done

		# setup rosbag
		bag = rosbag.Bag(bag_path, "w")
		topic = "/cartesian_admittance_controller/debug"
		sub = rospy.Subscriber( # setup sub to write to bag
			topic,
			CartesianAdmittanceControllerDebug, lambda msg: bag.write(topic, msg), queue_size=1, tcp_nodelay=True
		)

		# run tests
		input("press [RETURN] to stop test...")

		sub.unregister()
		bag.close()

	# plot

	# convert rosbag to pandas dataframe; offset time column to begin at 0
	b = bagpy.bagreader(bag_path)

	df_ee = pd.read_csv(b.message_by_topic(topic="/cartesian_admittance_controller/debug"))
	df_ee["Time"] = df_ee["Time"].apply(lambda x: x - df_ee["Time"][0])
	# print(df_ee.columns)

	plt.style.use("teleop_grasp")

	## force (measured)
	fig, ax = plt.subplots()
	ax.plot(df_ee["Time"], df_ee["h_e.force.x"], label=r"$f_{e,x}$", color="r")
	ax.plot(df_ee["Time"], df_ee["h_e.force.y"], label=r"$f_{e,y}$", color="g")
	ax.plot(df_ee["Time"], df_ee["h_e.force.z"], label=r"$f_{e,z}$", color="b")
	ax.set_ylabel("Force [N m]"); ax.set_xlabel("Time [s]"); ax.legend(); ax.grid()
	fig.savefig(f"{dir_img}/{test_name}-force.pdf")

	## torque (measured)
	fig, ax = plt.subplots()
	ax.plot(df_ee["Time"], df_ee["h_e.torque.x"], label=r"$\mu_{e,x}$", color="r")
	ax.plot(df_ee["Time"], df_ee["h_e.torque.y"], label=r"$\mu_{e,y}$", color="g")
	ax.plot(df_ee["Time"], df_ee["h_e.torque.z"], label=r"$\mu_{e,z}$", color="b")
	ax.set_ylabel("Torque [N m]"); ax.set_xlabel("Time [s]"); ax.legend(); ax.grid()
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

