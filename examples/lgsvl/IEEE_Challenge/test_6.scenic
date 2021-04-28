"""
TITLE: IEEE Challenge - Test 5
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

param EGO_INIT_DIST = VerifaiRange(-30, -20)
param EGO_SPEED = VerifaiRange(7, 10)
EGO_BRAKE = 1.0

PED_MIN_SPEED = 1.0
PED_THRESHOLD = 20

param SAFETY_DIST = VerifaiRange(10, 15)
BUFFER_DIST = 75
CRASH_DIST = 5
TERM_DIST = 50

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)
    interrupt when withinDistanceToObjsInLane(self, globalParameters.SAFETY_DIST) and (ped in network.drivableRegion):
        take SetBrakeAction(EGO_BRAKE)
    interrupt when withinDistanceToAnyObjs(self, CRASH_DIST):
        terminate

#################################
# SPATIAL RELATIONS             #
#################################

lane = Uniform(*network.lanes)
spawnPt = OrientedPoint in lane.centerline
endPt = OrientedPoint following roadDirection from spawnPt for TERM_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = ApolloCar following roadDirection from spawnPt for globalParameters.EGO_INIT_DIST,
    with behavior DriveTo(endPt)

ped = Pedestrian right of spawnPt by 3,
    with heading 90 deg relative to spawnPt.heading,
    with regionContainedIn None,
    with behavior CrossingBehavior(ego, PED_MIN_SPEED, PED_THRESHOLD)

require (distance to intersection) > BUFFER_DIST
require always (ego.laneSection._slowerLane is None)
require always (ego.laneSection._fasterLane is None)
# terminate when (distance to spawnPt) > TERM_DIST
