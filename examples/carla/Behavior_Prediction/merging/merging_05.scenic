"""
TITLE: Behavior Prediction - Merging 05
AUTHOR: Francis Indaheng, findaheng@berkeley.edu
DESCRIPTION: 
SOURCE: INTERACTION, DR_CHN_Merging_MT
"""

#################################
# MAP AND MODEL                 #
#################################

param map = localPath('../../../../tests/formats/opendrive/maps/CARLA/Town06.xodr')
param carla_map = 'Town06'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_INIT_DIST = [20, 25]
EGO_SPEED = VerifaiRange(7, 10)
EGO_BRAKE = VerifaiRange(0.5, 1.0)

ADV_INIT_DIST = [15, 20]
ADV_SPEED = VerifaiRange(7, 10)

SAFETY_DIST = VerifaiRange(10, 20)
TERM_DIST = 70

#################################
# AGENT BEHAVIORS               #
#################################



#################################
# SPATIAL RELATIONS             #
#################################


#################################
# SCENARIO SPECIFICATION        #
#################################

ego = Car at egoSpawnPt,
	with behavior FollowLaneBehavior(EGO_SPEED)

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
terminate when (distance to egoSpawnPt) > TERM_DIST
