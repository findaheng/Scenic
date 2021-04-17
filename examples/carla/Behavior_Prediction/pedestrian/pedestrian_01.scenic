"""
TITLE: Behavior Prediction - Pedestrian 01
AUTHOR: Francis Indaheng, findaheng@berkeley.edu
DESCRIPTION: Ego vehicle must stop suddenly to avoid collision when 
pedestrian crosses the road unexpectedly .
SOURCE: Carla Challenge, #03
"""

#################################
# MAP AND MODEL                 #
#################################

param map = localPath('../../../../tests/formats/opendrive/maps/CARLA/Town01.xodr')
param carla_map = 'Town01'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

MODEL = 'vehicle.lincoln.mkz2017'

param EGO_INIT_DIST = VerifaiRange(-30, -20)
param EGO_SPEED = VerifaiRange(7, 10)
param EGO_BRAKE = VerifaiRange(0.5, 1.0)

param PED_MIN_SPEED = 1.0
param PED_THRESHOLD = 20

param SAFETY_DIST = VerifaiRange(5, 10)
BUFFER_DIST = 75
TERM_DIST = 50

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=EGO_SPEED)
    interrupt when withinDistanceToObjsInLane(self, SAFETY_DIST):
        take SetBrakeAction(EGO_BRAKE)

#################################
# SPATIAL RELATIONS             #
#################################

lane = Uniform(*network.lanes)
spawnPt = OrientedPoint on lane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ped = Pedestrian right of spawnPt by 3,
    with heading 90 deg relative to spawnPt.heading,
    with regionContainedIn None,
    with behavior CrossingBehavior(ego, PED_MIN_SPEED, PED_THRESHOLD)

ego = Car following roadDirection from spawnPt for EGO_INIT_DIST,
    with blueprint MODEL,
    with behavior EgoBehavior()

require (distance to intersection) > BUFFER_DIST
require always (ego.laneSection._slowerLane is None)
require always (ego.laneSection._fasterLane is None)
terminate when (distance to spawnPt) > TERM_DIST
