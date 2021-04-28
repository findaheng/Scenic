"""
TITLE: IEEE Challenge - Test 5
AUTHOR: Francis Indaheng, findaheng@berkeley.edu
DESCRIPTION: Detect and respond to encroaching oncoming vehicles
"""

#################################
# MAP AND MODEL                 #
#################################

param map = localPath('/home/scenic/Desktop/Carla/VerifiedAI/Scenic-devel/examples/lgsvl/maps/borregasave.xodr')
param lgsvl_map = 'BorregasAve'
param time_step = 1.0/10

param apolloHDMap = 'Borregas Ave'
model scenic.simulators.lgsvl.model

#################################
# CONSTANTS                     #
#################################

param EGO_SPEED = VerifaiRange(5, 10)

param ADV_THROTTLE = VerifaiRange(0.2, 0.5)
param ADV_OFFSET = VerifaiRange(2, 3)

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
endPt = OrientedPoint following roadDirection from egoSpawnPt for TERM_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = ApolloCar following roadDirection from egoSpawnPt by -INIT_DIST,
	with behavior DriveTo(endPt)

adversary = Car left of egoSpawnPt by globalParameters.ADV_OFFSET,
	facing toward ego,
	with behavior DriveForwardBehavior(globalParameters.ADV_THROTTLE)

require (distance to intersection) > INIT_DIST
require (distance from adversary to intersection) > INIT_DIST
#require always (ego.laneSection._fasterLane is not None)
# terminate when (distance to adversary) > TERM_DIST
