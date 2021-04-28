"""
TITLE: IEEE Challenge - Test 3
AUTHOR: Francis Indaheng, findaheng@berkeley.edu
DESCRIPTION: Perform lane change/low-speed merge
"""

#################################
# MAP AND MODEL                 #
#################################

param map = localPath('/home/scenic/Desktop/Carla/VerifiedAI/Scenic-devel/examples/lgsvl/maps/sanfrancisco.xodr')
param lgsvl_map = 'SanFrancisco'
param time_step = 1.0/10

param apolloHDMap = 'San Francisco'
model scenic.simulators.lgsvl.model

#################################
# CONSTANTS                     #
#################################

INIT_DIST = 50
TERM_TIME = 5

#################################
# SPATIAL RELATIONS             #
#################################

initLane = Uniform(*network.lanes)
endPt = OrientedPoint in initLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

npc = Car at endPt,
	with behavior FollowLaneBehavior(target_speed=7)

npc_2 = Car ahead of npc by -20,
	with behavior FollowLaneBehavior(target_speed=7)

ego = ApolloCar left of npc_2 by 3,
	with behavior DriveTo(endPt)

require (distance to intersection) > INIT_DIST
