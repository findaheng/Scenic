"""
TITLE: Behavior Prediction - Intersection 05
AUTHOR: Francis Indaheng, findaheng@berkeley.edu
DESCRIPTION: Ego vehicle waits for an adversary vehicle to pass before 
performing a lane change to bypass a stationary vehicle waiting to make 
a left turn at a 4-way intersection.
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

EGO_INIT_DIST = VerifaiRange(10, 15)
EGO_SPEED = VerifaiRange(7, 10)
EGO_BRAKE = VerifaiRange(0.5, 1.0)

STAT_INIT_DIST = [0, 5]

ADV_INIT_DIST = [15, 20]
ADV_SPEED = 10

BYPASS_DIST = 8
CRASH_DIST = 5
TERM_DIST = 70

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
	try:
		while (distance to adversary) < BYPASS_DIST:
			take SetBrakeAction(EGO_BRAKE)
		rightLaneSec = self.laneSection.laneToRight
		do LaneChangeBehavior(
				laneSectionToSwitch=rightLaneSec,
				target_speed=EGO_SPEED)
		do FollowLaneBehavior(target_speed=EGO_SPEED)
	interrupt when withinDistanceToAnyObjs(self, CRASH_DIST):
		terminate

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

statInitLane = Uniform(*filter(lambda lane: 
	all([sec._laneToRight is not None for sec in lane.sections]),
	intersection.incomingLanes))
statSpawnPt = OrientedPoint in statInitLane.centerline

advInitLane = statInitLane.sectionAt(statSpawnPt).laneToRight.lane
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, advInitLane.maneuvers))
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = OrientedPoint in advInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

stationary = Car at statSpawnPt,
	with blueprint MODEL,

ego = Car behind stationary by EGO_INIT_DIST,
	with blueprint MODEL,
	with behavior EgoBehavior()

adversary = Car at advSpawnPt,
	with blueprint MODEL,
	with behavior FollowTrajectoryBehavior(target_speed=ADV_SPEED, trajectory=advTrajectory)

require STAT_INIT_DIST[0] <= (distance from stationary to intersection) <= STAT_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary to intersection) <= ADV_INIT_DIST[1]
terminate when (distance to statSpawnPt) > TERM_DIST
