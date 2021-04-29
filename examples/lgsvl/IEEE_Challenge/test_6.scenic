"""
TITLE: IEEE Challenge - Test 6
AUTHOR: Francis Indaheng, findaheng@berkeley.edu
DESCRIPTION: Detect and respond to pedestrians
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

param EGO_OFFSET = VerifaiRange(-30, -20)
param PED_OFFSET = VerifaiRange(0, 6)
param PED_SPEED = VerifaiRange(1, 3)
param PED_HESITATE = VerifaiRange(0, 30)

BUFFER_DIST = 75
TERM_DIST = 50

#################################
# AGENT BEHAVIORS               #
#################################

behavior PedBehavior(midPt, endPt):
    do FollowWaypoints([midPt])
    while not withinDistanceToAnyObjs(self, globalParameters.PED_HESITATE):
        wait
    do FollowWaypoints([endPt])

#################################
# SPATIAL RELATIONS             #
#################################

refPt = OrientedPoint in Uniform(*network.lanes).centerline
egoEndPt = OrientedPoint following roadDirection from refPt for TERM_DIST
pedEndPt = OrientedPoint left of refPt by PED_OFFSET,
    with speed globalParameters.PED_SPEED

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = ApolloCar following roadDirection from refPt for globalParameters.EGO_OFFSET,
    with behavior DriveTo(egoEndPt)

ped = Pedestrian right of refPt by 3,
    with heading 90 deg relative to refPt.heading,
    with regionContainedIn None,
    with behavior PedBehavior(refPt, pedEndPt)

require (distance to intersection) > BUFFER_DIST
require always (ego.laneSection._slowerLane is None)
require always (ego.laneSection._fasterLane is None)