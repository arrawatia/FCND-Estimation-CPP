# 1

Run for 1 simulation
```
python stddev.py
MeasuredStdDev_GPSPosXY =  0.7263455847056927
MeasuredStdDev_AccelXY =  0.5102579905374903
```

```
Simulation #3 (../config/06_SensorNoise.txt)
PASS: ABS(Quad.GPS.X-Quad.Pos.X) was less than MeasuredStdDev_GPSPosXY for 68% of the time
PASS: ABS(Quad.IMU.AX-0.000000) was less than MeasuredStdDev_AccelXY for 70% of the time
```


Step 2

From the pdf
we use the state to define a quaternion, qt, for the euler angles for φ, θ and ψ. Then we can define dq to be the quaternion that consists of the measurement of the angular rates from the IMU in the body frame, following Equation 84 in Diebel [5]. Using these two, we can define a predicted quaternion, q ̄ as follows:
Finally we can define θ ̄ and φ ̄ as follows: tt
t
q ̄ =dq∗q (43) tt

```
PASS: ABS(Quad.Est.E.MaxEuler) was less than 0.100000 for at least 3.000000 seconds
```


Step 3

Step 3.1: Implement the state transition function g
Scenario 08_PredictState

Step 3.2: Add a real IMU, implement Predict and Calculate rgb prime
Scenario 09_PredictCovariance

Implemented Predict and RGBPrime

Param tuning
QVelXYStd = .18
QVelZStd = .1

After tuning. the covariance models the data well.

Step 4

Magnetometer update
Scenario : 10_MagUpdate

QYawStd = 0.2	

```
PASS: ABS(Quad.Est.E.Yaw) was less than 0.120000 for at least 10.000000 seconds
PASS: ABS(Quad.Est.E.Yaw-0.000000) was less than Quad.Est.S.Yaw for 79% of the time
```

Step 5 
Closed Loop + GPS Update

11_GPSUpdate

```
Simulation #2 (../config/11_GPSUpdate.txt)
PASS: ABS(Quad.Est.E.Pos) was less than 1.000000 for at least 20.000000 seconds
```

Step 6
Adding the controller

```Simulation #1 (../config/11_GPSUpdate.txt)
Simulation #2 (../config/11_GPSUpdate.txt)
PASS: ABS(Quad.Est.E.Pos) was less than 1.000000 for at least 20.000000 seconds
```




