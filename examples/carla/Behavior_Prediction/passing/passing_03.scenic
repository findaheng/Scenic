"""
TITLE: Behavior Prediction - Passing 03
AUTHOR: Francis Indaheng, findaheng@berkeley.edu
DESCRIPTION: Ego vehicle performs a lane change to bypass a slow 
adversary vehicle but cannot return to its original lane because 
the adversary accelerates. Ego vehicle must then slow down to avoid 
collision with leading vehicle in new lane.
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

EGO_SPEED = VerifaiRange(7, 10)
EGO_BRAKE = VerifaiRange(0.5, 1.0)

LEAD_SPEED = VerifaiRange(5, 7)

ADV_DIST = VerifaiRange(10, 25)
ADV_INIT_SPEED = VerifaiRange(2, 4)
ADV_END_SPEED = VerifaiRange(7, 10)

BYPASS_DIST = [15, 10]
SAFE_DIST = 10
INIT_DIST = 50
TERM_TIME = 5

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(speed):
	try:
		do FollowLaneBehavior(target_speed=speed)
	interrupt when (distance to adversary) < BYPASS_DIST[0]:
		fasterLaneSec = self.laneSection.fasterLane
		do LaneChangeBehavior(
				laneSectionToSwitch=fasterLaneSec,
				target_speed=speed)
		try:
			do FollowLaneBehavior(
					target_speed=speed,
					laneToFollow=fasterLaneSec.lane) \
				until (distance to adversary) > BYPASS_DIST[1]
		interrupt when (distance to lead) < SAFE_DIST:
			take SetBrakeAction(EGO_BRAKE)
		slowerLaneSec = self.laneSection.slowerLane
		do LaneChangeBehavior(
				laneSectionToSwitch=slowerLaneSec,
				target_speed=speed)
		do FollowLaneBehavior(target_speed=speed) for TERM_TIME seconds
		terminate 

behavior AdversaryBehavior(init_speed, end_speed):
	do FollowLaneBehavior(target_speed=init_speed) \
		until self.lane is not ego.lane
	do FollowLaneBehavior(target_speed=end_speed)

#################################
# SPATIAL RELATIONS             #
#################################

initLane = Uniform(*network.lanes)
leadLaneSec = initLane.laneSection.fasterLane.successor

egoSpawnPt = OrientedPoint in initLane.centerline
leadSpawnPt = OrientedPoint in leadLaneSec.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = Car at egoSpawnPt,
	with behavior EgoBehavior(EGO_SPEED)

adversary = Car following roadDirection for ADV_DIST,
	with behavior AdversaryBehavior(ADV_INIT_SPEED, ADV_END_SPEED)

lead = Car at leadSpawnPt,
	with behavior FollowLaneBehavior(target_speed=LEAD_SPEED)

require (distance to intersection) > INIT_DIST
require (distance from adversary to intersection) > INIT_DIST
require (distance from lead to intersection) > INIT_DIST
require always (adversary.laneSection._fasterLane is not None)
