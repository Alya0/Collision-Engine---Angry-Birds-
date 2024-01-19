# Unity Collision System: Angry Birds Edition

## Overview
This project provides a comprehensive collision system implemented from scratch in Unity, demonstrated within an Angry Birds-style game>

## Features
* Explore the collision system in action through a 3 leveled Angry Birds game, with different bird functionalities.
* Collision algorithms pipeline optimzed for 3D game scenarios.
* View detailed description about different variables in your scene.
* Multiple camera options.

## Collision algorithms pipeline
### Collision detection
The collision detection is split into two phases: the broad phase and the narrow phase.

**The broad phase** is responsible for finding pairs of rigid bodies that are potentially colliding and excluding pairs that are certainly not colliding from amongst the whole set of bodies that are in the simulation. This phase is important to narrow down the complexity of collision detection.
In this phase, we used:
- Axis-Aligned Bounding Boxes (AABB)
- Sort and Sweep Algorithm

**The narrow phase** operates on the pairs of rigid bodies found by the broad phase that might be colliding. It is a refinement step where we determine if the collisions are actually happening, and for each collision that is found, we compute the contact points that will be used to solve the collisions later.
In this phase, we used:
- VHACD (to convert every concave shape to convex)
- The Gilbert-Johnson-Keerthi Algorithm "GJK"
- Expanding Polytope Algorithm "EPA"

### Collision response

We implemented the collision responses utilizing impulse, incorporating intricate mathematical calculations based on the references below to achieve optimal results.


## Release

You can download the game from [here](https://lnkd.in/efGh39j7).

## Setup


## Preview


## Refrences
* https://www.toptal.com/game/video-game-physics-part-ii-collision-detection-for-solid-objects

* https://hitokageproduction.com/article/11

* https://chrishecker.com/Rigid_Body_Dynamics (part 3)

