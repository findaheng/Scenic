"""
TITLE: Behavior Prediction - Intersection 01
AUTHOR: Francis Indaheng, findaheng@berkeley.edu
DESCRIPTION: Ego vehicle goes straight at 4-way intersection and must 
suddenly stop to avoid collision when adversary vehicle from opposite 
lane makes a left turn.
SOURCE: Carla Challenge, #04
"""

#################################
# MAP AND MODEL                 #
#################################

param map = localPath('../../../../tests/formats/opendrive/maps/CARLA/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

MODEL = 'vehicle.lincoln.mkz2017'

EGO_INIT_DIST = [20, 25]
param EGO_SPEED = VerifaiRange(7, 10)
EGO_BRAKE = 1.0

PED_MIN_SPEED = 1.0
PED_THRESHOLD = 20

param SAFETY_DIST = VerifaiRange(10, 15)
CRASH_DIST = 5
TERM_DIST = 50

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(trajectory):
	try:
		do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
	interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
		take SetBrakeAction(EGO_BRAKE)
	interrupt when withinDistanceToAnyObjs(self, CRASH_DIST):
		terminate

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way or i.is3Way and len(i.crossings) > 0, network.intersections))

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = OrientedPoint in egoInitLane.centerline

tempManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, egoManeuver.reverseManeuvers))
tempInitLane = tempManeuver.startLane
tempSpawnPt = tempInitLane.centerline[-1]

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior(egoTrajectory)

ped = Pedestrian right of tempSpawnPt by 3,
	with heading ego.heading,
	with regionContainedIn None,
	with behavior CrossingBehavior(ego, PED_MIN_SPEED, PED_THRESHOLD)

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
terminate when (distance to egoSpawnPt) > TERM_DIST
