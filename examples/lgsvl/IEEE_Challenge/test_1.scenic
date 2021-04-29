"""
TITLE: IEEE Challenge - Test 3
AUTHOR: Francis Indaheng, findaheng@berkeley.edu
DESCRIPTION: Perform lane change/low-speed merge
"""

#################################
# MAP AND MODEL                 #
#################################

param map = localPath('/home/scenic/Desktop/Carla/VerifiedAI/Scenic-devel/examples/lgsvl/maps/borregasave.xodr')
param lgsvl_map = 'BorregasAve'
param time_step = 1.0/10

param apolloHDMap = 'Borregas Ave'
model scenic.simulators.lgsvl.model

#################################
# CONSTANTS                     #
#################################

INIT_DIST = 50
TERM_TIME = 5

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: any([len(l.group.lanes) > 1 for l in i.incomingLanes]), network.intersections))
rightTurnManuever = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, intersection.maneuvers))

spawnPt = OrientedPoint in rightTurnManuever.startLane.centerline
endPt = OrientedPoint in rightTurnManuever.endLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

npc = Car at spawnPt,
	with behavior FollowLaneBehavior(target_speed=7)

npc_2 = Car behind npc by 20,
	with behavior FollowLaneBehavior(target_speed=7)

ego = ApolloCar left of npc_2 by 3,
	with behavior DriveTo(endPt)

require (distance from npc to intersection) > 10
