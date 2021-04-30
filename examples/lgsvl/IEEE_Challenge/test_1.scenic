"""
TITLE: IEEE Challenge - Test 1
AUTHOR: Francis Indaheng, findaheng@berkeley.edu
DESCRIPTION: Perform lane change/low-speed merge
"""
from scenic.simulators.lgsvl.utils import *

#################################
# MAP AND MODEL                 #
#################################

param map = localPath('../../../tests/formats/opendrive/maps/borregasave.xodr')
param lgsvl_map = 'BorregasAve'
param time_step = 1.0/10
param apolloHDMap = 'Borregas Ave'
model scenic.simulators.lgsvl.model

#################################
# AGENT BEHAVIORS               #
#################################

behavior AllowMergeBehavior():
    try:
        do FollowLaneBehavior(target_speed=3)
    interrupt when withinDistanceToAnyObjs(self, 7):
        take SetBrakeAction(0.5)

#################################
# SPATIAL RELATIONS             #
#################################

npcPt = 1.03999996185303 @ 29.75
egoPt = 4.23999977111816 @ 28.3099994659424
endPt = OrientedPoint right of (OrientedPoint following roadDirection from npcPt for 40) by 8

#################################
# SCENARIO SPECIFICATION        #
#################################

npc_1 = Car at npcPt,
	with behavior AllowMergeBehavior()

npc_2 = Car following roadDirection from npcPt for 20,
    with behavior FollowLaneBehavior(target_speed=10)

ego = ApolloCar at egoPt,
    with behavior DriveTo(endPt)
