"""
TITLE: IEEE Challenge - Test 4
AUTHOR: Francis Indaheng, findaheng@berkeley.edu
DESCRIPTION: Detect and respond to school buses
"""

#################################
# MAP AND MODEL                 #
#################################

param map = localPath('../../../tests/formats/opendrive/maps/borregasave.xodr')
param lgsvl_map = 'BorregasAve'
param time_step = 1.0/10
param apolloHDMap = 'Borregas Ave'
model scenic.simulators.lgsvl.model

#################################
# CONSTANTS                     #
#################################

param EGO_OFFSET = VerifaiRange(-20, -10)
param EGO_SPEED = VerifaiRange(5, 10)

BUFFER_DIST = 50
TERM_DIST = 70

#################################
# SPATIAL RELATIONS             #
#################################

spawnPt = OrientedPoint in Uniform(*network.lanes).centerline
endPt = OrientedPoint following roadDirection from spawnPt for TERM_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = ApolloCar following roadDirection from spawnPt by globalParameters.EGO_OFFSET,
	with behavior DriveTo(endPt)

bus = Bus left of spawnPt by 3

require (distance to intersection) > BUFFER_DIST
