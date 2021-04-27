"""
TITLE: IEEE Challenge - Test 5
AUTHOR: Francis Indaheng, findaheng@berkeley.edu
DESCRIPTION: Detect and respond to school buses
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

param IS_RIGHT = VerifaiDiscreteRange(0, 1)

param EGO_SPEED = VerifaiRange(5, 10)

INIT_DIST = 50
TERM_DIST = 70

#################################
# SPATIAL RELATIONS             #
#################################

initLane = Uniform(*network.lanes)
spawnPt = OrientedPoint in initLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = Car behind spawnPt by 10,
	with behavior FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

bus = Car (right of spawnPt by 3) if globalParameters.IS_RIGHT == 1 else (left of spawnPt by 3)

require (distance to intersection) > INIT_DIST
require always (ego.laneSection._fasterLane is None)
require always (ego.laneSection._slowerLane is not None)
terminate when (distance to spawnPt) > TERM_DIST
