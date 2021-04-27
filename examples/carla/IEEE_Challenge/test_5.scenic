"""
TITLE: IEEE Challenge - Test 5
AUTHOR: Francis Indaheng, findaheng@berkeley.edu
DESCRIPTION: Detect and respond to encroaching oncoming vehicles
"""

#################################
# MAP AND MODEL                 #
#################################

param map = localPath('../../../tests/formats/opendrive/maps/CARLA/Town03.xodr')
param carla_map = 'Town03'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

param EGO_SPEED = VerifaiRange(5, 10)

param ADV_THROTTLE = VerifaiRange(0.5, 1.0)
param ADV_OFFSET = VerifaiRange(1, 2)

INIT_DIST = 50
CRASH_DIST = 15
TERM_DIST = 70

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(speed):
	try:
		do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)
	interrupt when (distance to adversary) < CRASH_DIST:
		terminate

behavior DriveForwardBehavior(throttle):
	try:
		take SetThrottleAction(throttle)
	interrupt when (distance from adversary to ego) < CRASH_DIST:
		take SetBrakeAction(1.0)

#################################
# SPATIAL RELATIONS             #
#################################

initLane = Uniform(*network.lanes)
egoSpawnPt = OrientedPoint in initLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = Car following roadDirection from egoSpawnPt by -INIT_DIST,
	with behavior FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

adversary = Car right of egoSpawnPt by globalParameters.ADV_OFFSET,
	facing toward ego,
	with behavior DriveForwardBehavior(globalParameters.ADV_THROTTLE)

require (distance to intersection) > INIT_DIST
require (distance from adversary to intersection) > INIT_DIST
require always (ego.laneSection._fasterLane is not None)
terminate when (distance to adversary) > TERM_DIST
