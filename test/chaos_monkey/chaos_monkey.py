#!/usr/bin/env python

import sys 
import time
import random
import rosnode
import rospy
import rostest

class ChaosMonkey():
	
	def __init__(self):
		self.continue_chaos = True
		self.nodes_to_kill = []
		self.MAX_DELAY = 10
		self.MIN_DELAY = 4

	def print_kill_list(self, input_list):
		print "Killed the follow nodes:"

		for item in input_list:
			print "\t"+ str(item)
		print "\n"

	def infect_topic(self, topic_name):
		pass

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

	def get_nodes(self):
		return rosnode.get_node_names()

	def select_node(self):
		nodes = self.get_nodes()
		nodes_len = len(nodes)
		node = nodes[random.randrange(0, nodes_len)]
		return node

	# def get_topics(self):
	# 	return rostopic.get_topics

	# def select_topic(self):
	# 	rand_num = random.randrange(0, 10)

	# 	if rand_num >= 6:
	# 		topic_type = "Image"
	# 	elif rand_num >= 3: 
	# 		topic_type = "FaceBox"
	# 	else:
	# 		topic_type = "FeatureCoordinates"

	# 	topics = self.get_topics()
	# 	topics_len = len(self.topics)
	# 	topic = random.randrange(0, topics_len)
	# 	return topic, topic_type

	def randomise_chaos(self):
		rand_num = random.randrange(0,10)
		target_node = self.select_node()
		#target_topic = self.select_topic()

		# 6/10 chance of killing a node instantly, 
		# 3/10 chance it is added to a list of nodes to kill
		# 1/10 chance the function is recalled
		if rand_num >= 4:
			self.kill_nodes(target_node)
		elif rand_num >= 1 and rand_num < 4:
			self.nodes_to_kill.append(target_node)
		else:
			self.randomise_chaos()

if __name__ == '__main__':
    CM = ChaosMonkey()
    CM.cause_chaos()
    print "ChaosMonkey has been killed"