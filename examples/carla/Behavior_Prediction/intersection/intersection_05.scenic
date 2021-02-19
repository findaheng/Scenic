"""
TITLE: Behavior Prediction - Intersection 05
AUTHOR: Francis Indaheng, findaheng@berkeley.edu
DESCRIPTION: Ego vehicle waits for an adversary vehicle to pass before 
performing a lane change to bypass a stationary vehicle waiting to make 
an unprotected left turn.
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

EGO_INIT_DIST = (5, 10)
EGO_SPEED = 10
EGO_BRAKE = 1.0

STAT_INIT_DIST = [0, 5]

ADV_INIT_DIST = [15, 20]
ADV_SPEED = 10

BYPASS_DIST = 10
TERM_DIST = 70

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(speed, trajectory):
	take SetBrakeAction(EGO_BRAKE) \
		until (distance to adversary) > BYPASS_DIST
	rightLaneSec = self.laneSection.laneToRight
	do LaneChangeBehavior(
			laneSectionToSwitch=rightLaneSec,
			target_speed=speed)
	do FollowTrajectoryBehavior(target_speed=speed, trajectory=trajectory)

behavior AdversaryBehavior(speed, trajectory):
	do FollowTrajectoryBehavior(target_speed=speed, trajectory=trajectory)

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way and i.isSignalized, network.intersections))

statInitLane = Uniform(*filter(lambda lane: 
	all([sec._laneToLeft is None and sec._laneToRight is not None for sec in lane.sections]),
	intersection.incomingLanes))
statSpawnPt = OrientedPoint in statInitLane.centerline

advInitLane = statInitLane.laneToRight
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, advInitLane.maneuvers))
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = OrientedPoint in advInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

stationary = Car at statSpawnPt

ego = Car behind stationary by EGO_INIT_DIST,
	with behavior EgoBehavior(EGO_SPEED, advTrajectory)

adversary = Car at advSpawnPt,
	with behavior AdversaryBehavior(ADV_SPEED, advTrajectory)

require STAT_INIT_DIST[0] <= (distance from stationary to intersection) <= STAT_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary to intersection) <= EGO_INIT_DIST[1]
terminate when (distance to egoSpawnPt) > TERM_DIST
