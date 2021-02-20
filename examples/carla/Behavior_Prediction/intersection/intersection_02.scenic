"""
TITLE: Behavior Prediction - Intersection 02
AUTHOR: Francis Indaheng, findaheng@berkeley.edu
DESCRIPTION: Ego vehicle makes an unprotected left turn at signalized 
intersection and must suddenly stop to avoid collision when adversary 
vehicle from oncoming parallel lane goes straight.
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

EGO_INIT_DIST = [20, 25]
EGO_SPEED = 10
EGO_BRAKE = 1.0

ADV_INIT_DIST = [15, 20]
ADV_SPEED = 10

SAFETY_DIST = 20
TERM_DIST = 70

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(speed, trajectory):
	try:
		do FollowTrajectoryBehavior(target_speed=speed, trajectory=trajectory)
	interrupt when withinDistanceToAnyObjs(self, SAFETY_DIST):
		take SetBrakeAction(EGO_BRAKE)

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way and i.isSignalized, network.intersections))

advInitLane = Uniform(*intersection.incomingLanes)
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, advInitLane.maneuvers))
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = OrientedPoint in advInitLane.centerline

egoInitLane = Uniform(*filter(lambda m:
		m.type is ManeuverType.STRAIGHT,
		Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, advInitLane.maneuvers))
			.disjointManeuvers)
	).startLane
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = OrientedPoint in egoInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = Car at egoSpawnPt,
	with behavior EgoBehavior(EGO_SPEED, egoTrajectory)

adversary = Car at advSpawnPt,
	with behavior FollowTrajectoryBehavior(target_speed=ADV_SPEED, trajectory=advTrajectory)

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary to intersection) <= EGO_INIT_DIST[1]
terminate when (distance to egoSpawnPt) > TERM_DIST