"""
TITLE: Behavior Prediction - Passing 02
AUTHOR: Francis Indaheng, findaheng@berkeley.edu
DESCRIPTION: Adversary vehicle performs a lane change to bypass the 
slow ego vehicle before returning to its original lane.
"""

#################################
# MAP AND MODEL                 #
#################################

param map = localPath('../../../../tests/formats/opendrive/maps/CARLA/Town03.xodr')
param carla_map = 'Town03'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_SPEED = 3
ADV_SPEED = 10

BYPASS_DIST = (15, 15)
INIT_DIST = 50
TERM_DIST = 70

#################################
# AGENT BEHAVIORS               #
#################################

behavior AdversaryBehavior(speed):
	try:
		do FollowLaneBehavior(target_speed=speed)
	interrupt when withinDistanceToAnyObjs(self, BYPASS_DIST[0]):
		do LaneChangeBehavior(
				laneSectionToSwitch=self.laneSection.fasterLane,
				target_speed=speed)
		do FollowLaneBehavior(
				target_speed=speed,
				laneToFollow=fasterLane.lane) \
			until (distance to ego) > BYPASS_DIST[1]

#################################
# SPATIAL RELATIONS             #
#################################

initLane = Uniform(*network.lanes)
advSpawnPt = OrientedPoint in initLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

adversary = Car at advSpawnPt,
	with behavior AdversaryBehavior(EGO_SPEED)

ego = Car following roadDirection for Range(10, 25),
	with behavior FollowLaneBehavior(ADV_SPEED)

require (distance to intersection) > INIT_DIST
require (distance from adversary to intersection) > INIT_DIST
require always (ego.laneSection._fasterLane is not None)
terminate when (distance from adversary to advSpawnPt) > TERM_DIST
