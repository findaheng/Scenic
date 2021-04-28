"""
TITLE: IEEE Challenge - Test 2
AUTHOR: Francis Indaheng, findaheng@berkeley.edu
DESCRIPTION: Perform vehicle following
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

param LEAD_DIST = VerifaiRange(10, 20)
param LEAD_SPEED = VerifaiRange(2, 7)

INIT_DIST = 50
TERM_DIST = 70

#################################
# SPATIAL RELATIONS             #
#################################

initLane = Uniform(*network.lanes)
egoSpawnPt = OrientedPoint in initLane.centerline
endPt = OrientedPoint following roadDirection from egoSpawnPt for TERM_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = ApolloCar at egoSpawnPt,
	with behavior DriveTo(endPt)

lead = Car following roadDirection for globalParameters.LEAD_DIST,
	with behavior FollowLaneBehavior(target_speed=globalParameters.LEAD_SPEED)

require (distance to intersection) > INIT_DIST
# terminate when (distance to egoSpawnPt) > TERM_DIST
