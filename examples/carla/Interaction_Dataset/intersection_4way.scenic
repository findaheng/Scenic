""" Scenario Descripton
Crossing negotiation at an unsignalized 4-way intersection.
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

ADV_SPEED = (5, 15)

SAFETY_DISTANCE = 20

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(speed, trajectory):
	try:
		do FollowTrajectoryBehavior(target_speed=speed, trajectory=trajectory)
		do FollowLaneBehavior(target_speed=speed)
	interrupt when withinDistanceToAnyObjs(self, SAFETY_DISTANCE):
		take SetBrakeAction(BRAKE_INTENSITY)

behavior AdversaryBehavior(speed, trajectory):
	FollowTrajectoryBehavior(target_speed=speed, trajectory=trajectory)

#################################
# SPATIAL RELATIONS             #
#################################

fourWayIntersections = filter(lambda i: i.is4Way and i.isSignalized, network.intersections)
assert len(fourWayIntersections) > 0, f'No signalized 4-way intersections in {carla_map} map'

# Choose random 4-way intersection
intersection = Uniform(*fourWayIntersections)

egoStartLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*egoStartLane.maneuvers)
egoTrajectory = [egoStartLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = OrientedPoint on egoStartLane.centerline

advManeuver = Uniform(*egoManeuver.conflictingManeuvers)
advStartLane = advManeuver.startLane
advTrajectory = [advStartLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = OrientedPoint on advStartLane.centerline

#################################
# OBJECT PLACEMENT              #
#################################

ego = Car at egoSpawnPt,
	with blueprint EGO_MODEL,
	with behavior EgoBehavior(EGO_SPEED, egoTrajectory)

adversary = Car at advSpawnPt,
	with behavior AdversaryBehavior(ADV_SPEED, advTrajectory)

require 20 <= (distance to intersection) <= 25
require 15 <= (distance from adversary to intersection) <= 20
terminate when (distance to egoSpawnPt) > 70
