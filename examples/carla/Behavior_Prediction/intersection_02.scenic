"""
TITLE: Behavior Prediction - Intersection 02
AUTHOR: Francis Indaheng, findaheng@berkeley.edu
DESCRIPTION: Ego vehicle either goes straight or makes an unprotected 
left turn at signalized intersection and must suddenly stop to avoid 
collision when adversary vehicle from perpendicular lane continues 
shortly after red light, going either straight or left.
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
	interrupt when withinDistanceToAnyObjs(self, SAFETY_DIST):
		take SetBrakeAction(EGO_BRAKE)

behavior AdversaryBehavior(speed, trajectory):
	do FollowTrajectoryBehavior(target_speed=speed, trajectory=trajectory)

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way and i.isSignalized, network.intersections))

advStartLane = Uniform(*intersection.incomingLanes)
advManeuver = Uniform(*filter(lambda m:
		m.type in (ManeuverType.STRAIGHT, ManeuverType.LEFT_TURN),
		advStartLane.maneuvers))
advTrajectory = [advStartLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = OrientedPoint in advStartLane.centerline

egoStartLane = Uniform(*filter(lambda m:
		m.type is ManeuverType.STRAIGHT,
		Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, advStartLane.maneuvers))
			.conflictingManeuvers)
	).startLane
egoManeuver = Uniform(*filter(lambda m:
		m.type in (ManeuverType.STRAIGHT, ManeuverType.LEFT_TURN),
		egoStartLane.maneuvers))
egoTrajectory = [egoStartLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = OrientedPoint in egoStartLane.centerline

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
terminate when (distance from ego to egoSpawnPt) > TERM_DIST
