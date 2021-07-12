param map = localPath('../../../tests/formats/opendrive/maps/CARLA/Town01.xodr')
param carla_map = 'Town01'
model scenic.simulators.carla.model

scenario IntersectionScenario(intersection, maneuver):
    setup:

        advInitLane = Uniform(*intersection.incomingLanes)
        advManeuver = Uniform(*filter(lambda m: m.type is maneuver, advInitLane.maneuvers))
        advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
        advSpawnPt = OrientedPoint in advInitLane.centerline

        adv = Car at advSpawnPt,
            with behavior FollowTrajectoryBehavior(
                target_speed=globalParameters.ADV_SPEED, 
                trajectory=advTrajectory)

scenario Main():
    setup:
        ego = Car on road with FollowLaneBehavior(target_speed=5)
    compose:
        while True:
            inter = network.intersectionAt(ego.position)
            if inter is not None:
                do choose IntersectionScenario(inter, ManeuverType.LEFT_TURN), 
                    IntersectionScenario(inter, ManeuverType.STRAIGHT),
                    IntersectionScenario(inter, ManeuverType.RIGHT_TURN)
