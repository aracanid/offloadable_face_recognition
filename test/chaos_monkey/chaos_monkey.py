#!/usr/bin/env python

import os
import sys 
import time
import random
import rosnode
import xmlrpclib
import rospy
import rostest
import numpy as np
from sensor_msgs.msg import Image
from offloadable_face_recognition.msg import SchedulerCommand, FaceBox

class ChaosMonkey():
	
	def __init__(self):
		self.master = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
		rospy.init_node("ChaosMonkey")
		self.caller_id = '/script'
		self.continue_chaos = True
		self.nodes_to_kill = []

		self.IMAGE_TOPIC = "offloadable_face_recognition/Image"
		self.FACE_BOX_TOPIC = "offloadable_face_recognition/FaceBox"
		self.SCHEDULER_TOPIC = "offloadable_face_recognition/SchedulerCommand"
		self.INFECTED_TOPIC_NAME = "infected_topic"

		self.MAX_DELAY = 10
		self.MIN_DELAY = 4
		self.MAX_INFECTION_TIME = 2 #seconds
		self.EXCLUDED_NODES = []
		self.EXCLUDED_TOPICS = []

		# Publishers to allow flooding topics
		self.queue_size = 10
		self.scheduler_pub = rospy.Publisher("scheduler_commands", SchedulerCommand, queue_size=self.queue_size)
		self.output_face_box_pub = rospy.Publisher("face_box_coordinates", FaceBox, queue_size=self.queue_size)

	def print_kill_list(self, input_list):
		print "Killed the follow nodes:"

		for item in input_list:
			print "\t"+ str(item)
		print "\n"

	def infect_topic(self, topic_name, topic_type):
		
		start_time = time.time()
		while time.time() < (start_time + self.MAX_INFECTION_TIME*1000):
			if topic_type == self.IMAGE_TOPIC:
				pass
			elif topic_type == self.FACE_BOX_TOPIC:
				face_box = FaceBox()
				face_box.x = random.randrange(0,640)
				face_box.y = random.randrange(0,480)
				face_box.width = face_box.x + random.randrange(0,640)
				face_box.height = face_box.y + random.randrange(0,480)
				self.output_face_box_pub.publish(face_box)
			elif topic_type == self.SCHEDULER_TOPIC:
				scheduler_command = SchedulerCommand()
				scheduler_command.node_name = self.INFECTED_TOPIC_NAME
				scheduler_command.offload = True
				self.scheduler_pub.publish(scheduler_command)
			else:
				break
		if topic_type == self.SCHEDULER_TOPIC or topic_type == self.FACE_BOX_TOPIC or topic_type == self.IMAGE_TOPIC:
			print "Infected the following topics \n\t"+ str(item)
			
	def kill_nodes(self, node_name):
		self.nodes_to_kill.append(node_name)
		rosnode.kill_nodes(self.nodes_to_kill)
		self.print_kill_list(self.nodes_to_kill)
		self.nodes_to_kill = []

	def cause_chaos(self):
		while self.continue_chaos:
			try:
				self.randomise_chaos()
				delay = random.randrange(self.MIN_DELAY, self.MAX_DELAY)
				time.sleep(delay)
	  		except (KeyboardInterrupt, SystemExit):
			  	print "Killing ChaosMonkey!"
			  	self.continue_chaos = False

	# Returns a list of active nodes
	def get_nodes(self):
		return rosnode.get_node_names()

	# Selects a target node from a given list of nodes
	def select_node(self):
		nodes = self.get_nodes()
		nodes_len = len(nodes)
		node = nodes[random.randrange(0, nodes_len)]
		if node == "/ChaosMonkey":
			self.select_node()
		else:
			return node

	# Returns a list of topics and their types in the form [[topic,type]...]
	def get_topics(self):
		code, status, topics = self.master.getTopicTypes(self.caller_id)
		return topics

	# Selects a target topic from a given list of topics and types
	def select_topic(self):
		topics = self.get_topics()
		topics_len = len(topics)
		rand_num = random.randrange(0, topics_len)
		topic = topics[rand_num][0]
		topic_type = topics[rand_num][1]
		return topic, topic_type

	def randomise_chaos(self):
		rand_num = random.randrange(0,10)
		target_node = self.select_node()
		target_topic, target_topic_type = self.select_topic()

		# 4/10 chance of killing a node instantly, 
		# 2/10 chance it is added to a list of nodes to kill
		# 3/10
		# 1/10 chance the function is recalled
		if rand_num >= 6:
			self.kill_nodes(target_node)
		elif rand_num == 5 or rand_num == 6:
			self.nodes_to_kill.append(target_node)
		elif rand_num >= 1 and rand_num <=3:
			self.infect_topic(target_topic, target_topic_type)
		else:
			self.randomise_chaos()


if __name__ == '__main__':
    CM = ChaosMonkey()
    CM.cause_chaos()
    print "ChaosMonkey has been killed"