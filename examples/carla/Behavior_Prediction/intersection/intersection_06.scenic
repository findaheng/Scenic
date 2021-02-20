"""
TITLE: Behavior Prediction - Intersection 06
AUTHOR: Francis Indaheng, findaheng@berkeley.edu
DESCRIPTION: Ego vehicle slows down for an adversary vehicle to perform 
a lane change to pass a stationary vehicle waiting to make an 
unprotected left turn.
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

EGO_INIT_DIST = [15, 20]
EGO_SPEED = 10

STAT_INIT_DIST = [0, 5]

ADV_INIT_DIST = (5, 10)
ADV_SPEED = 10
ADV_BRAKE = 1.0

BYPASS_DIST = 10
TERM_DIST = 70

#################################
# AGENT BEHAVIORS               #
#################################

behavior AdversaryBehavior(speed, trajectory):
	take SetBrakeAction(ADV_BRAKE) \
		until (distance from adversary to ego) > BYPASS_DIST
	rightLaneSec = self.laneSection.laneToRight
	do LaneChangeBehavior(
			laneSectionToSwitch=rightLaneSec,
			target_speed=speed)
	do FollowTrajectoryBehavior(target_speed=speed, trajectory=trajectory)

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way and i.isSignalized, network.intersections))

statInitLane = Uniform(*filter(lambda lane: 
	all([sec._laneToLeft is None and sec._laneToRight is not None for sec in lane.sections]),
	intersection.incomingLanes))
statSpawnPt = OrientedPoint in statInitLane.centerline

egoInitLane = statInitLane.laneToRight
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoTrajectory = [advInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = OrientedPoint in egoInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = Car at egoSpawnPt,
	with behavior FollowTrajectoryBehavior(target_speed=ADV_SPEED, trajectory=advTrajectory)

stationary = Car at statSpawnPt

adversary = Car behind stationary by EGO_INIT_DIST,
	with behavior EgoBehavior(EGO_SPEED, advTrajectory)

require STAT_INIT_DIST[0] <= (distance from stationary to intersection) <= STAT_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary to intersection) <= EGO_INIT_DIST[1]
terminate when (distance to statSpawnPt) > TERM_DIST
