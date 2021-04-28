"""
TITLE: IEEE Challenge - Test 3
AUTHOR: Francis Indaheng, findaheng@berkeley.edu
DESCRIPTION: Move out of travel lane/park
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

initLane = Uniform(*network.lanes)
endPt = OrientedPoint in initLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

npc = Car ahead of endPt by 10
npc_2 = Car ahead of npc by 7
npc_3 = Car behind npc by 10

ego = ApolloCar left of npc_3 by 3,
	with behavior DriveTo(endPt)

require (distance to intersection) > INIT_DIST
