# 3 way intersection. ego car turns left. actor has right of way.

from scenic.simulators.lgsvl.simulator import LGSVLSimulator
from scenic.simulators.lgsvl.map import loadLocalNetwork
loadLocalNetwork(__file__, 'maps/cubetown.xodr')
from scenic.simulators.lgsvl.model import *

simulator LGSVLSimulator('CubeTown')
param time_step = 1.0/10

# CONSTANTS
space = [2,3,4,5]

# GEOMETRY
threeWayIntersections = filter(lambda i: i.is3Way, network.intersections)
intersection = Uniform(*threeWayIntersections)

left_maneuvers = filter(lambda m: m.type == ManeuverType.LEFT_TURN, intersection.maneuvers)
ego_maneuver = Uniform(*left_maneuvers)
ego_L_centerlines = [ego_maneuver.startLane.centerline, ego_maneuver.connectingLane.centerline, ego_maneuver.endLane.centerline]
egoStart = (OrientedPoint at ego_maneuver.startLane.centerline[-1]) offset by (-2, 2) @ 0 

# ---

conflicting_left = filter(lambda m: m.type == ManeuverType.LEFT_TURN, ego_maneuver.conflictingManeuvers)
actor_maneuver = Uniform(*conflicting_left)
actor_centerlines = [actor_maneuver.startLane.centerline, actor_maneuver.connectingLane.centerline, actor_maneuver.endLane.centerline]
actorStart = actor_maneuver.startLane.centerline[-1]


# BEHAVIOR
behavior EgoBehavior(thresholdDistance, target_speed=20, trajectory = None):
	assert trajectory is not None
	brakeIntensity = 0.7

	try: 
		FollowTrajectoryBehavior(target_speed=15, trajectory=trajectory)

	interrupt when distanceToAnyCars(car=self, thresholdDistance=thresholdDistance):
		take SetBrakeAction(brakeIntensity)


# PLACEMENT
ego = EgoCar following roadDirection from egoStart by -Uniform(*space),
		with blueprint 'vehicle.tesla.model3',
		with behavior EgoBehavior(target_speed=10, trajectory=ego_L_centerlines, thresholdDistance = 20)

other = EgoCar following roadDirection from actorStart by -Uniform(*space),
		with blueprint 'vehicle.tesla.model3',
		with behavior FollowTrajectoryBehavior(target_speed=15, trajectory=actor_centerlines)


# require that other car reaches the intersection before the ego car