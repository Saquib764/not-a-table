# Steps
1. Maintain 5 points
2. 2nd point should be the current target. Switching happens only if both motors cross the target.
3. compute angles between segments.
4. If angles above a threshold, clip velocity to zero.
5. If angle is low, maintain smooth transition in speed.
6. Interpolate the points by 1mm distance.
7. compute an array of distance, speed and acceleration pair. transition to low speed should happen atleast 3mm
8. Wait for both motors at corners.


# Simulation

```
g++ test_trajectory.cpp && ./a.out && python visualize.py
```

