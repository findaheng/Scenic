"""
TITLE: IEEE Challenge - Test 3
AUTHOR: Francis Indaheng, findaheng@berkeley.edu
DESCRIPTION: Perform lane change/low-speed merge
"""
from scenic.simulators.lgsvl.utils import *

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
TERM_DIST = 70


behavior hesitateBehavior():
	currTime = simulation().currentTime
	while simulation().currentTime - currTime < 7:
		wait
	while True:
		try:
			do FollowLaneBehavior(target_speed=3)
		interrupt when withinDistanceToAnyObjs(self, 7):
			take SetBrakeAction(0.5)

#################################
# SPATIAL RELATIONS             #
#################################

laneGroup = Uniform(*filter(lambda group: len(group.lanes) == 2, network.laneGroups))

# intersection = Uniform(*filter(lambda i: any([len(l.group.lanes) > 1 for l in i.incomingLanes]), network.intersections))
# rightTurnManuever = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, intersection.maneuvers))

# spawnPt = OrientedPoint in rightTurnManuever.startLane.centerline
# endPt = OrientedPoint in rightTurnManuever.endLane.centerline
spawnPt = OrientedPoint in laneGroup.lanes[-1].centerline
npc_spawnPt = 1.03999996185303 @ 29.75
hardcoded_ego_pt = 4.23999977111816 @ 28.3099994659424
endPt = OrientedPoint following roadDirection from npc_spawnPt for 40
endPt = OrientedPoint right of endPt by 8
# hardcoded_npc_pt = gpsToScenicPosition(4141575.96, 587099.75)


#################################
# SCENARIO SPECIFICATION        #
#################################

npc = Car following roadDirection from npc_spawnPt for 0,
	with behavior hesitateBehavior()

npc_2 = Car behind npc by -20,
	with behavior FollowLaneBehavior(target_speed=10)

ego = ApolloCar at hardcoded_ego_pt,
	with behavior DriveTo(endPt)

# require (len(rightTurnManuever.startLane.group.lanes) > 1)
