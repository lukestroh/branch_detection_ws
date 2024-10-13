# TODO

 - Needs to be a kind of SLAM, needs localization but also some mapping

 - Particle = (x,y,z,r,p,y, 6-dof)
    - Each particle has a map associated with it (and a gaussian value associated with that map)

### Some dx,dy,dz
1. Update all particles with motion
x + dx + noise
2.  Update map
Kalman filter of branch location with respect to particle
3. Weighting
    - Does my observation look like my map?
    - If we have tree model, we can just ask for pose estimate with respect to tree
    - If we do not have the tree model, can we map it
- Other code:
- Masked image, xyz robot
- Which trees/branches are in the frustum
