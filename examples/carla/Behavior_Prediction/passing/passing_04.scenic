"""
TITLE: Behavior Prediction - Passing 04
AUTHOR: Francis Indaheng, findaheng@berkeley.edu
DESCRIPTION: Ego vehicle performs multiple lane changes to bypass 
three slow adversary vehicles.
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

ADV1_DIST = VerifaiRange(20, 25)
ADV2_DIST = VerifaiRange(20, 25)
ADV3_DIST = VerifaiRange(20, 25)
ADV_SPEED = VerifaiRange(2, 4)

BYPASS_DIST = 10
INIT_DIST = 50
TERM_DIST = 75

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(speed, adversaries, laneSectionToSwitch):
	try:
		do FollowLaneBehavior(target_speed=speed)
	interrupt when (len(adversaries) > 0 and
				   (distance to adversaries[0]) < BYPASS_DIST):
		do LaneChangeBehavior(
			laneSectionToSwitch=laneSectionToSwitch,
			target_speed=speed)
		do EgoBehavior(
			speed=speed,
			adversaries=adversaries[1:],
			laneSectionToSwitch=(self.laneSection.laneToRight
								 if len(adversaries) % 2 == 0
								 else self.laneSection.laneToLeft))

behavior Adversary2Behavior(speed):
	rightLaneSec = self.laneSection.laneToRight
	do LaneChangeBehavior(
			laneSectionToSwitch=rightLaneSec,
			target_speed=speed)
	do FollowLaneBehavior(target_speed=speed)

#################################
# SPATIAL RELATIONS             #
#################################

initLane = Uniform(*filter(lambda lane:
	all([sec._laneToRight is not None for sec in lane.sections]),
	network.lanes))
egoSpawnPt = OrientedPoint in initLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = Car at egoSpawnPt,
	with behavior EgoBehavior(EGO_SPEED, EGO_BRAKE)

adversary_1 = Car following roadDirection for ADV1_DIST,
	with behavior FollowLaneBehavior(target_speed=ADV_SPEED)

adversary_2 = Car following roadDirection for (ADV1_DIST + ADV2_DIST),
	with behavior Adversary2Behavior(ADV_SPEED)

adversary_3 = Car following roadDirection for (ADV1_DIST + ADV2_DIST + ADV3_DIST),
	with behavior FollowLaneBehavior(target_speed=ADV_SPEED)

require (distance to intersection) > INIT_DIST
require (distance from adversary_1 to intersection) > INIT_DIST
require (distance from adversary_2 to intersection) > INIT_DIST
require (distance from adversary_3 to intersection) > INIT_DIST
require always (adversary.laneSection._laneToRight is not None)
terminate when (distance to adversary_3) > TERM_DIST