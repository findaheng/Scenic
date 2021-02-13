"""
TITLE: Behavior Prediction - Intersection 01
AUTHOR: Francis Indaheng, findaheng@berkeley.edu
DESCRIPTION: Ego vehicle goes straight at signalized intersection and 
must suddenly stop to avoid collision when oncoming adversary vehicle 
makes an unprotected left turn.
"""

#################################
# MAP AND MODEL                 #
#################################

param map = localPath('../../../tests/formats/opendrive/maps/CARLA/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = 'vehicle.lincoln.mkz2017'
EGO_INIT_DIST = (20,25)
EGO_SPEED = 10
EGO_BRAKE = 1.0

ADV_MODEL = 'vehicle.audi.a2'
ADV_INIT_DIST = (15,20)
ADV_SPEED = 10
ADV_BRAKE = 1.0

SAFETY_DIST = 20
TERM_DIST = 70

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(speed, trajectory):
	try:
		do FollowTrajectoryBehavior(target_speed=speed, trajectory=trajectory)
		do FollowLaneBehavior(target_speed=speed)
	interrupt when withinDistanceToAnyObjs(self, SAFETY_DIST):
		take SetBrakeAction(EGO_BRAKE)

behavior AdversaryBehavior(speed, trajectory):
	do FollowTrajectoryBehavior(target_speed=speed, trajectory=trajectory)

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way and i.isSignalized, network.intersections))

egoStartLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoStartLane.maneuvers))
egoTrajectory = [egoStartLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = OrientedPoint in egoStartLane.centerline

advManeuvers = Uniform(*filter( \
	lambda m: m.type is ManeuverType.LEFT_TURN and m.startLane.road is egoManeuver.endLane.road, 
	egoManeuver.conflictingManeuvers))
advStartLane = advManeuver.startLane
advTrajectory = [advStartLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = OrientedPoint in advStartLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = Car at egoSpawnPt,
	with blueprint EGO_MODEL,
	with behavior EgoBehavior(EGO_SPEED, egoTrajectory)

adversary = Car at advSpawnPt,
	with blueprint ADV_MODEL,
	with behavior AdversaryBehavior(ADV_SPEED, advTrajectory)

require EGO_INIT_DIST[0] <= (distance from ego to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary to intersection) <= EGO_INIT_DIST[1]
terminate when (distance to egoSpawnPt) > TERM_DIST
