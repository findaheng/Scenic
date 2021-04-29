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

param LEAD_OFFSET = VerifaiRange(10, 20)
param LEAD_SPEED = VerifaiRange(2, 7)

BUFFER_DIST = 50
TERM_DIST = 70

#################################
# SPATIAL RELATIONS             #
#################################

refPt = OrientedPoint in Uniform(*network.lanes).centerline
endPt = OrientedPoint following roadDirection from refPt for TERM_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = ApolloCar at refPt,
	with behavior DriveTo(endPt)

lead = Car following roadDirection for globalParameters.LEAD_OFFSET,
	with behavior FollowLaneBehavior(target_speed=globalParameters.LEAD_SPEED)

require (distance to intersection) > BUFFER_DIST
