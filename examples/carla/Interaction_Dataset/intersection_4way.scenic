""" Scenario Descripton
Ego-vehicle approaches a busy 4-way intersection and goes straight.
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

EGO_SPEED = 10

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
	do FollowLaneBehavior(EGO_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

intersecs4way = [i for i in network.intersections if i.is4Way]
assert len(intersecs4way) > 0, f'No 4-way intersections in {carla_map} map'

# Choose random 4-way intersection
intersection = Uniform(*intersecs4way)

egoLane = intersection.incomingLanes[0]

#################################
# OBJECT PLACEMENT              #
#################################

egoPt = OrientedPoint on egoLane.centerline

ego = Car at egoPt,
	with behavior EgoBehavior()
