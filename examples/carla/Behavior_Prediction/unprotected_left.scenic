""" Scenario Descripton
Signalized intersection in which ego vehicle goes straight and adversary vehicle makes an unprotected left turn.
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
EGO_SPEED = 10
EGO_BRAKE = 1.0

ADV_MODEL = 'vehicle.audi.a2'
ADV_SPEED = 10
ADV_BRAKE = 1.0

EGO_INIT_DIST = (20,25)
ADV_INIT_DIST = (15,20)
SAFETY_DIST = 20
TERM_DIST = 70

#################################
# SPATIAL RELATIONS             #
#################################

# TODO: Figure out why intersection object has no isSignalized attribute
fourWayIntersections = filter(lambda i: i.is4Way, network.intersections)
assert len(fourWayIntersections) > 0, f'No signalized 4-way intersections in {carla_map} map'
intersection = Uniform(*fourWayIntersections)

egoStartLane = Uniform(*intersection.incomingLanes)
egoManeuver = network.ManeuverType.STRAIGHT
egoTrajectory = [egoStartLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = OrientedPoint on egoStartLane.centerline

advManeuver = Uniform(*[m for m in egoManeuver.conflictingManeuvers if m.type is network.ManeuverType.LEFT_TURN and m.startLane.road is egoStartLane.road])
advStartLane = advManeuver.startLane
advTrajectory = [advStartLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = OrientedPoint on advStartLane.centerline

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
# SCENARIO DESCRIPTION          #
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
