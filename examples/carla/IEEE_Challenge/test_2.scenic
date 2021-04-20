"""
TITLE: IEEE Challenge - Test 2
AUTHOR: Francis Indaheng, findaheng@berkeley.edu
DESCRIPTION: Perform vehicle following
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

param LEAD_DIST = VerifaiRange(10, 20)
param LEAD_SPEED = VerifaiRange(2, 7)

INIT_DIST = 50
TERM_TIME = 5

#################################
# SPATIAL RELATIONS             #
#################################

initLane = Uniform(*network.lanes)
egoSpawnPt = OrientedPoint in initLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = Car at egoSpawnPt,
	with behavior FollowLaneBehavior(target_speed=globalParameters.LEAD_SPEED)

adversary = Car following roadDirection for globalParameters.LEAD_DIST,
	with behavior FollowLaneBehavior(target_speed=globalParameters.LEAD_SPEED)

require (distance to intersection) > INIT_DIST
