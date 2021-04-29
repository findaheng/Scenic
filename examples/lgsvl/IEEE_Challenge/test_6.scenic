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
param PED_OFFSET = VerifaiRange(3, 6)
param PED_SPEED = VerifaiRange(1, 3)
param PED_INIT_HESITATE = VerifaiRange(10, 30)
param PED_MID_HESITATE = VerifaiRange(10, 50)

BUFFER_DIST = 75
TERM_DIST = 50

#################################
# AGENT BEHAVIORS               #
#################################

behavior PedBehavior(midPt, endPt):
    while simulation().currentTime - currTime < globalParameters.PED_INIT_HESITATE:
        wait
    take FollowWaypointsAction([midPt])
    currTime = simulation().currentTime
    while simulation().currentTime - currTime < globalParameters.PED_MID_HESITATE:
        wait
    take FollowWaypointsAction([endPt])

#################################
# SPATIAL RELATIONS             #
#################################

refPt = OrientedPoint in Uniform(*network.lanes).centerline
egoEndPt = OrientedPoint following roadDirection from refPt for TERM_DIST
pedMidPt = OrientedPoint left of refPt by 0,
	with speed globalParameters.PED_SPEED
pedEndPt = OrientedPoint left of refPt by globalParameters.PED_OFFSET,
    with speed globalParameters.PED_SPEED

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = ApolloCar following roadDirection from refPt for globalParameters.EGO_OFFSET,
    with behavior DriveTo(egoEndPt)

ped = Pedestrian right of refPt by 3,
    with heading 90 deg relative to refPt.heading,
    with regionContainedIn None,
    with behavior PedBehavior(pedMidPt, pedEndPt)

require (distance to intersection) > BUFFER_DIST
require always (ego.laneSection._slowerLane is None)
require always (ego.laneSection._fasterLane is None)
