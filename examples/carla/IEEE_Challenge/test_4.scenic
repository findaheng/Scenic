"""
TITLE: IEEE Challenge - Test 5
AUTHOR: Francis Indaheng, findaheng@berkeley.edu
DESCRIPTION: Detect and respond to school buses
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

# MODEL = 'vehicle.lincoln.mkz2017'  # set to bus in LGSVL

EGO_OFFSET = 10
param EGO_SPEED = VerifaiRange(5, 10)

INIT_DIST = 50
TERM_DIST = 70

#################################
# SPATIAL RELATIONS             #
#################################

initLane = Uniform(*network.lanes)
spawnPt = OrientedPoint in initLane.centerline
endPt = OrientedPoint following roadDirection from spawnPt for TERM_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = ApolloCar following roadDirection from spawnPt by -EGO_OFFSET,
	with behavior DriveTo(endPt)

bus = Bus left of spawnPt by 3

require (distance to intersection) > INIT_DIST
# require always (ego.laneSection._fasterLane is None)
# require always (ego.laneSection._slowerLane is not None)
# terminate when (distance to spawnPt) - EGO_OFFSET > TERM_DIST
