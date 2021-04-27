"""
TITLE: IEEE Challenge - Test 3
AUTHOR: Francis Indaheng, findaheng@berkeley.edu
DESCRIPTION: Move out of travel lane/park
"""

#################################
# MAP AND MODEL                 #
#################################

param map = localPath('../../../tests/formats/opendrive/maps/CARLA/Town03.xodr')
param carla_map = 'Town03'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

INIT_DIST = 50
TERM_TIME = 5

#################################
# SPATIAL RELATIONS             #
#################################

initLane = Uniform(*filter(lamdba ln: network.lanes)
npcSpawnPt = OrientedPoint in initLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

npc = Car at npcSpawnPt
npc_2 = Car ahead of npc by 2
npc_3 = Car behind npc by 10

ego = Car left of npc_3 by 3

require (distance to intersection) > INIT_DIST
require (distance from adversary to intersection) > INIT_DIST
require always (npc.laneSection._laneToRight is None)
require always (npc.laneSection._fasterLane is not None)
