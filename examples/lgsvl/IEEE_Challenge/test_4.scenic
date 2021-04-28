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

MODEL = 'vehicle.lincoln.mkz2017'  # set to bus in LGSVL

EGO_OFFSET = 10
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

ego = Car behind spawnPt by EGO_OFFSET,
	with behavior FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

bus = Car right of spawnPt by 3

require (distance to intersection) > INIT_DIST
require always (ego.laneSection._fasterLane is None)
require always (ego.laneSection._slowerLane is not None)
terminate when (distance to spawnPt) - EGO_OFFSET > TERM_DIST
