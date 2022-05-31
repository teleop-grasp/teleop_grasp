#!/usr/bin/env python3

from glob import glob
import os
import sys
import subprocess
import rospy
import rosbag
import bagpy
import matplotlib.pyplot as plt
import pandas as pd
from threading import Lock
from scipy.spatial.transform import Rotation

from geometry_msgs.msg import Point, Wrench, Vector3, Pose, PoseStamped
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

def update_bag(topic, msg):
	global bag, mtx_bag
	mtx_bag.acquire()
	bag.write(topic, msg)
	mtx_bag.release()


if __name__ == "__main__":

	# setup test; test_name is inferred from filename as "test_<test_name>.py"
	(node_name, test_name, dir_data, dir_img) = setup_test(filepath=__file__, test_id="")
	rospy.init_node(f"test_{node_name}")

	bag_path = f"{dir_data}/{test_name}.bag"
	if not os.path.exists(bag_path): # skip if already done

		# read current impedance gains
		info = subprocess.check_output(["rosrun", "dynamic_reconfigure", "dynparam", "get", "/cartesian_admittance_controller"])
		print(info, file=open(f"{dir_data}/{test_name}.txt", "w", encoding="utf-8"))

		# setup rosbag; add subsribers to write to bag
		mtx_bag = Lock()
		bag = rosbag.Bag(bag_path, "w")

		sub_ee = rospy.Subscriber(
			name="/cartesian_admittance_controller/debug", data_class=CartesianAdmittanceControllerDebug,
			callback=lambda msg: update_bag("/cartesian_admittance_controller/debug", msg), queue_size=1, tcp_nodelay=True
		)

		sub_pose_hand = rospy.Subscriber(
			name="/pose_gesture/pose_hand", data_class=Pose,
			callback=lambda msg: update_bag("/pose_gesture/pose_hand", msg), queue_size=1, tcp_nodelay=True
		)

		sub_pose_ee_des = rospy.Subscriber(
			name="/cartesian_admittance_controller/command", data_class=PoseStamped,
			callback=lambda msg: update_bag("/cartesian_admittance_controller/command", msg), queue_size=1, tcp_nodelay=True
		)

		# run tests
		input("press [RETURN] to stop test...")

		sub_ee.unregister()
		sub_pose_hand.unregister()
		sub_pose_ee_des.unregister()
		bag.close()

	# plot

	# convert rosbag to pandas dataframe; offset time column to begin at 0
	b = bagpy.bagreader(bag_path)
	print(b.topic_table)

	df_ee = pd.read_csv(b.message_by_topic(topic="/cartesian_admittance_controller/debug"))
	df_ee["Time"] = df_ee["Time"].apply(lambda x: x - df_ee["Time"][0])
	# print(df_ee.columns)

	df_pose_hand = pd.read_csv(b.message_by_topic(topic="/pose_gesture/pose_hand"))
	df_pose_hand["Time"] = df_pose_hand["Time"].apply(lambda x: x - df_pose_hand["Time"][0])
	df_pose_hand[["rpy.x", "rpy.y", "rpy.z"]] = df_pose_hand.apply(
		lambda row: pd.Series([*Rotation.from_quat([row["orientation.x"], row["orientation.y"], row["orientation.z"], row["orientation.w"]]).as_euler("zyx", degrees=True)]),
		axis=1
	)
	# print(df_pose_hand.columns)

	df_pose_ee_des = pd.read_csv(b.message_by_topic(topic="/cartesian_admittance_controller/command"))
	df_pose_ee_des["Time"] = df_pose_ee_des["Time"].apply(lambda x: x - df_pose_ee_des["Time"][0])
	# print(df_pose_ee_des.columns)

	plt.style.use("teleop_grasp")
	
	## pose hand (position)
	fig, ax = plt.subplots()
	ax.plot(df_pose_hand["Time"], df_pose_hand["position.x"], label=r"$p_{x}$", color="r")
	ax.plot(df_pose_hand["Time"], df_pose_hand["position.y"], label=r"$p_{y}$", color="g")
	ax.plot(df_pose_hand["Time"], df_pose_hand["position.z"], label=r"$p_{z}$", color="b")
	ax.set_ylabel("Position [m]"); ax.set_xlabel("Time [s]"); ax.legend(loc="upper right"); ax.grid()
	fig.savefig(f"{dir_img}/{test_name}-pose-hand-pos.pdf")

	## pose hand (orientation RPY)
	fig, ax = plt.subplots()
	ax.plot(df_pose_hand["Time"], df_pose_hand["rpy.x"], label=r"$\varphi_{x}$", color="r")
	ax.plot(df_pose_hand["Time"], df_pose_hand["rpy.y"], label=r"$\varphi_{y}$", color="g")
	ax.plot(df_pose_hand["Time"], df_pose_hand["rpy.z"], label=r"$\varphi_{z}$", color="b")
	ax.set_ylabel("RPY [deg]"); ax.set_xlabel("Time [s]"); ax.legend(loc="upper right"); ax.grid()
	fig.savefig(f"{dir_img}/{test_name}-pose-hand-ori.pdf")

	## pose ee desired (position)
	fig, ax = plt.subplots()
	ax.plot(df_pose_ee_des["Time"], df_pose_ee_des["pose.position.x"], label=r"$p_{d,x}$", color="r")
	ax.plot(df_pose_ee_des["Time"], df_pose_ee_des["pose.position.y"], label=r"$p_{d,y}$", color="g")
	ax.plot(df_pose_ee_des["Time"], df_pose_ee_des["pose.position.z"], label=r"$p_{d,z}$", color="b")
	ax.set_ylabel("Position [m]"); ax.set_xlabel("Time [s]"); ax.legend(loc="upper right"); ax.grid()
	fig.savefig(f"{dir_img}/{test_name}-pose-ee-des-pos.pdf")

	## pose ee actual (position)
	fig, ax = plt.subplots()
	ax.plot(df_ee["Time"], df_ee["T_e.pose.position.x"], label=r"$p_{e,x}$", color="r")
	ax.plot(df_ee["Time"], df_ee["T_e.pose.position.y"], label=r"$p_{e,y}$", color="g")
	ax.plot(df_ee["Time"], df_ee["T_e.pose.position.z"], label=r"$p_{e,z}$", color="b")
	ax.set_ylabel("Position [m]"); ax.set_xlabel("Time [s]"); ax.legend(loc="upper right"); ax.grid()
	fig.savefig(f"{dir_img}/{test_name}-pose-ee-pos.pdf")
