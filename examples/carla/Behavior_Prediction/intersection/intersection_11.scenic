"""
TITLE: Behavior Prediction - Intersection 01
AUTHOR: Francis Indaheng, findaheng@berkeley.edu
DESCRIPTION: N vehicles approach an intersection and take a random maneuver.
SOURCE: Kesav Viswanadha
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

param N = 5  # number of additional vehicles

INIT_DIST = [20, 25]
TERM_DIST = 70

param EGO_SPEED = VerifaiRange(7, 10)
param EGO_SAFETY_DIST = VerifaiRange(10, 20)
param EGO_BRAKE = VerifaiRange(0.5, 1.0)

param OTHER_SPEEDS = [VerifaiRange(7, 10) for _ in range(N)]
param OTHER_SAFETY_DISTS = [VerifaiRange(10, 20) for _ in range(N)]
param OTHER_BRAKES = [VerifaiRange(0.5, 1.0) for _ in range(N)]

#################################
# AGENT BEHAVIORS               #
#################################

behavior IntersectionBehavior(trajectory, speed, safetyDist, brake):
	try:
		do FollowTrajectoryBehavior(target_speed=speed, trajectory=trajectory)
	interrupt when withinDistanceToAnyObjs(self, safetyDist):
		take SetBrakeAction(brake)

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(egoInitLane.maneuvers)
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = Car on egoInitLane with behavior IntersectionBehavior(egoTrajectory, EGO_SPEED, EGO_SAFETY_DIST, EGO_BRAKE)

for i in range(N):
	tempInitLane = Uniform(*intersection.incomingLanes)
	tempManeuver = Uniform(*tempInitLane.maneuvers)
	tempTrajectory = [tempInitLane, tempManeuver.connectingLane, tempManeuver.endLane]
	Car on tempInitLane with behavior IntersectionBehavior(tempTrajectory, OTHER_SPEEDS[i], OTHER_SAFETY_DISTS[i], OTHER_BRAKES[i])
