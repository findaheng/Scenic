# Behavior Prediction Scenario Descriptions

The following includes a library of 30 Scenic scripts written specifically for use with CARLA simulator.
Scenarios were created primarily for the testing of behavior prediction models, though the may be used in general cases.
Inspiration was drawn from multiple sources, including [NHSTA's Pre-Crash Scenarios](https://rosap.ntl.bts.gov/view/dot/41932/dot_41932_DS1.pdf) and the [INTERACTION dataset](https://github.com/interaction-dataset/interaction-dataset).
For questions and concerns, please contact Francis Indaheng at findaheng@berkeley.edu or post an issue to this repo.

> Some notation definitions:
> - DIP => Development in-progress
> - TIP => Testing in-progress

## Intersection
01. Ego vehicle goes straight at 4-way intersection and must suddenly stop to avoid collision when adversary vehicle from opposite lane makes a left turn. (source: NHSTA, #30)
02. Ego vehicle makes a left turn at 4-way intersection and must suddenly stop to avoid collision when adversary vehicle from opposite lane goes straight. (source: NHSTA, #30)
03. Ego vehicle either goes straight or makes a left turn at 4-way intersection and must suddenly stop to avoid collision when adversary vehicle from lateral lane continues straight. (source: NHSTA, #28 #29)
04. Ego vehicle either goes straight or makes a left turn at 4-way intersection and must suddenly stop to avoid collision when adversary vehicle from lateral lane makes a left turn. (source: NHSTA, #28 #29)
05. Ego vehicle makes a right turn at 4-way intersection while adversary vehicle from opposite lane makes a left turn. (source: NHSTA, #25) (TIP)
06. Ego vehicle makes a right turn at 4-way intersection while adversary vehicle from lateral lane goes straight. (source: NHSTA, #25 #26) (TIP)
07. Ego vehicle makes a left turn at 3-way intersection and must suddenly stop to avoid collision when adversary vehicle from lateral lane continues straight. (source: NHSTA, #30) (TIP)
08. Ego vehicle goes straight at 3-way intersection and must suddenly stop to avoid collision when adversary vehicle makes a left turn. (source: NHSTA #30) (TIP)
09. Ego vehicle makes a right turn at 3-way intersection while adversary vehicle from lateral lane goes straight. (source: NHSTA, #28 #29) (TIP)
10. Ego Vehicle waits at 4-way intersection while adversary vehicle in adjacent lane passes before performing a lane change to bypass a stationary vehicle waiting to make a left turn. (source: NHSTA, #16)

## Bypassing
01. Ego vehicle performs a lane change to bypass a slow adversary vehicle before returning to its original lane. (source: NHSTA, #16)
02. Advesary vehicle performs a lange change to bypass the ego vehicle before returning to its original lane. (source: NHSTA, #16)
03. Ego vehicle performs a lane change to bypass a slow adversary vehicle but cannot return to its original lane because the adversary accelerates. Ego vehicle must then slow down to avoid collision with leading vehicle in new lane. (source: NHSTA, #16)
04. Ego vehicle performs multiple lane changes to bypass two slow adversary vehicles. (source: NHSTA, #16) (TIP)
05. Ego vehicle performs multiple lane changes to bypass three slow adversary vehicles. (source: NHSTA, #16) (TIP)

## Roundabout
01. (DIP)
02. (DIP)
03. (DIP)
04. (DIP)
05. (DIP)

## Merging
01. (DIP)
02. (DIP)
03. (DIP)
04. (DIP)
05. (DIP)
