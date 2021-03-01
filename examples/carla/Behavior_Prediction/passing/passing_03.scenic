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
EGO_BRAKE = VerifaiRange(0.7, 1.0)

ADV_DIST = VerifaiRange(10, 15)
ADV_INIT_SPEED = VerifaiRange(2, 4)
ADV_END_SPEED = 2 * VerifaiRange(7, 10)
ADV_BUFFER_TIME = 5

LEAD_DIST = ADV_DIST + 10
LEAD_SPEED = EGO_SPEED - 4

BYPASS_DIST = [15, 10]
SAFE_DIST = 10
INIT_DIST = 50
TERM_DIST = 70
TERM_TIME = 10

#################################
# AGENT BEHAVIORS               #
#################################

behavior DecelerateBehavior(brake):
	take SetBrakeAction(brake)

behavior EgoBehavior():
	try:
		do FollowLaneBehavior(target_speed=EGO_SPEED)
	interrupt when (distance to adversary) < BYPASS_DIST[0]:
		fasterLaneSec = self.laneSection.fasterLane
		do LaneChangeBehavior(
				laneSectionToSwitch=fasterLaneSec,
				target_speed=EGO_SPEED)
		try:
			do FollowLaneBehavior(
					target_speed=EGO_SPEED,
					laneToFollow=fasterLaneSec.lane) \
				until (distance to adversary) > BYPASS_DIST[1]
		interrupt when (distance to lead) < SAFE_DIST:
			try:
				do DecelerateBehavior(EGO_BRAKE)
			interrupt when (distance to lead) > SAFE_DIST:
				do FollowLaneBehavior(target_speed=LEAD_SPEED) for TERM_TIME seconds
				terminate 

behavior AdversaryBehavior():
	do FollowLaneBehavior(target_speed=ADV_INIT_SPEED) \
		until self.lane is not ego.lane
	do FollowLaneBehavior(target_speed=ADV_END_SPEED)

behavior LeadBehavior():
	fasterLaneSec = self.laneSection.fasterLane
	do LaneChangeBehavior(
			laneSectionToSwitch=fasterLaneSec,
			target_speed=LEAD_SPEED)
	do FollowLaneBehavior(target_speed=LEAD_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

initLane = Uniform(*network.lanes)
egoSpawnPt = OrientedPoint in initLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = Car at egoSpawnPt,
	with behavior EgoBehavior()

adversary = Car following roadDirection for ADV_DIST,
	with behavior AdversaryBehavior()

lead = Car following roadDirection for LEAD_DIST,
	with behavior LeadBehavior()

require (distance to intersection) > INIT_DIST
require (distance from adversary to intersection) > INIT_DIST
require (distance from lead to intersection) > INIT_DIST
require always (adversary.laneSection._fasterLane is not None)
terminate when (distance to egoSpawnPt) > TERM_DIST
