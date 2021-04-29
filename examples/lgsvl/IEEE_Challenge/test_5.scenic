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

param EGO_OFFSET = VerifaiRange(-50, -30)
param ADV_OFFSET = VerifaiRange(2, 3)
param ADV_THROTTLE = VerifaiRange(0.2, 0.5)

BUFFER_DIST = 50
TERM_DIST = 20

#################################
# AGENT BEHAVIORS               #
#################################

behavior AdvBehavior():
	try:
		take SetThrottleAction(globalParameters.ADV_THROTTLE)
	interrupt when (distance from adversary to ego) < CRASH_DIST:
		take SetBrakeAction(1)

#################################
# SPATIAL RELATIONS             #
#################################

refPt = OrientedPoint in Uniform(*network.lanes).centerline
endPt = OrientedPoint following roadDirection from refPt for TERM_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = ApolloCar following roadDirection from refPt by globalParameters.EGO_OFFSET,
	with behavior DriveTo(endPt)

adversary = Car left of refPt by globalParameters.ADV_OFFSET,
	facing toward ego,
	with behavior DriveForwardBehavior()

require (distance to intersection) > BUFFER_DIST
require (distance from adversary to intersection) > BUFFER_DIST
