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
param ADV_SPEED = VerifaiRange(5, 10)

INIT_DIST = 50
TERM_DIST = 70

#################################
# SPATIAL RELATIONS             #
#################################

initLane = Uniform(*network.lanes)
egoSpawnPt = OrientedPoint in initLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = Car following roadDirection from egoSpawnPt by -INIT_DIST,
	with FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

adversary = Car right of egoSpawnPt by 2,
	facing toward ego,
	with speed globalParameters.ADV_SPEED

require (distance to intersection) > INIT_DIST
require (distance from adversary to intersection) > INIT_DIST
require always (ego.laneSection._fasterLane is not None)
terminate when (distance to egoSpawnPt) + INIT_DIST > TERM_DIST
