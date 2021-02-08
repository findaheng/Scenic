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

ADV_SPEED = 10
ADV_BRAKE = 1.0

STOP_DURATION = 50
STOP_DIST = 5
SAFETY_DIST = 20

#################################
# SPATIAL RELATIONS             #
#################################

fourWayIntersections = filter(lambda i: i.is4Way and not i.isSignalized, network.intersections)
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
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(speed, trajectory):
	stopped_ctr = 0
	try:
		do FollowTrajectoryBehavior(target_speed=speed, trajectory=trajectory)
		do FollowLaneBehavior(target_speed=speed)
	interrupt when (distance to intersection) < STOP_DIST and stopped_ctr < STOP_DURATION:
		take SetBrakeAction(EGO_BRAKE)
		stopped_ctr += 1
	interrupt when withinDistanceToAnyObjs(self, SAFETY_DIST):
		take SetBrakeAction(EGO_BRAKE)

behavior AdversaryBehavior(speed, trajectory):
	stopped_ctr = 0
	try:
		take FollowTrajectoryBehavior(target_speed=speed, trajectory=trajectory)
	interrupt when (distance to intersection) < STOP_DIST and stopped_ctr < STOP_DURATION:
		take SetBrakeAction(ADV_BRAKE)
		stopped_ctr += 1

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
