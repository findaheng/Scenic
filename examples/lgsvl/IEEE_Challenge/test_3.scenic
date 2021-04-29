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

BUFFER_DIST = 50

#################################
# SPATIAL RELATIONS             #
#################################

refPt = OrientedPoint in Uniform(*network.lanes).centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

npc_1 = Car ahead of refPt by 10
npc_2 = Car ahead of npc_1 by 7
npc_3 = Car behind npc_1 by 10

ego = ApolloCar left of npc_3 by 3,
	with behavior DriveTo(refPt)

require (distance to intersection) > BUFFER_DIST
