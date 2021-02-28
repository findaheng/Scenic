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

EGO_SPEED = VerifaiRange(2, 4)

ADV_DIST = VerifaiRange(-25, -10)
ADV_SPEED = VerifaiRange(7, 10)

BYPASS_DIST = [15, 15]
INIT_DIST = 50
TERM_TIME = 5

#################################
# AGENT BEHAVIORS               #
#################################

behavior AdversaryBehavior(speed):
	try:
		do FollowLaneBehavior(target_speed=speed)
	interrupt when withinDistanceToAnyObjs(self, BYPASS_DIST[0]):
		fasterLaneSec = self.laneSection.fasterLane
		do LaneChangeBehavior(
				laneSectionToSwitch=fasterLaneSec,
				target_speed=speed)
		do FollowLaneBehavior(
				target_speed=speed,
				laneToFollow=fasterLaneSec.lane) \
			until (distance to adversary) > BYPASS_DIST[1]
		slowerLaneSec = self.laneSection.slowerLane
		do LaneChangeBehavior(
				laneSectionToSwitch=slowerLaneSec,
				target_speed=speed)
		do FollowLaneBehavior(target_speed=speed) for TERM_TIME seconds
		terminate 

#################################
# SPATIAL RELATIONS             #
#################################

initLane = Uniform(*network.lanes)
egoSpawnPt = OrientedPoint in initLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = Car at egoSpawnPt,
	with behavior FollowLaneBehavior(target_speed=EGO_SPEED)

adversary = Car following roadDirection for ADV_DIST,
	with behavior AdversaryBehavior(ADV_SPEED)

require (distance to intersection) > INIT_DIST
require (distance from adversary to intersection) > INIT_DIST
require always (ego.laneSection._fasterLane is not None)
