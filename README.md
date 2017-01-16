# udacity_ai_for_robotics
Runaway robot

There is a runaway robot. It can do one of two things: it can drive in a straight line or it can turn. The runaway has gotten lost somewhere and is stuck driving in an almost-circle. Every time step the robot sends back data of its current position.

## Part 1 - Locate the runaway robot

### Problem

For the first part we assume perfectly accurate sensor measurements. The goal is to be able to predict the robot’s next location.

Every time step ‘estimate_next_pos’ is call. It is given an argument ‘measurement’, which is a single (x,y) point and an argument OTHER which is used to keep track of state between function calls.

### My Solution
I decided to use a Kalman Filter with angle of the robot and distance it has traveled in a time step. 

Uncertainty Covariance matrix
P = [ [0,0,0,0], [0,0,0,0], [0,0,1000,0], [0,0,0,1000]] 

Initial uncertainty of 0 for heading and distance since we get the perfect measurements for x,y position. And an arbitrarily large uncertainty of 1000 for the angular velocity ( change in angle over time) and change in distance over time.

State Transition Matrix
F =  [[1,0,dt,0], [0,1,0,dt], [0,0,1,0], [0,0,0,1]]

Measurement Function
H = [[1,0,0,0], [0,1,0,0]]

We observe heading and distance but not derivatives.

Measurement Update
y = z - Hx 	// where z = [[heading, distance]] and x is [[heading],[distance], [d(heading)/dt], [d(distance)/dt)]
S = HPHT + R
K = PHTS-1
x’ = x + Ky
P’ = (I - KH)P

Prediction step
x’= Fx + Bu // no external motion
P’ = FPFT + Q // no process noise

We need a previous point to calculate the heading and distance. So I start my measurement update and prediction step on the second measurement.

After I predict the new heading and distance I use it to calculate the Δx and Δy. I then add the  Δx and Δy of the current measurement to predict the next position.

I didn’t have to use a kalman filter to pass part 1. I could have used least-squares to find the best fitting circle. I saw the next step was to add noise so I bit the bullet.

## Part 2 - Adding Noise
### Problem
Now the measurement data given from the runaway robot is a bit noisy.
The goal is still to estimate the next position of the robot within 0.01 stepsizes.

### My Solution
To account for noise I updated my uncertainty covariance matrix:

Uncertainty Covariance matrix
P = [ [measurement_noise,0,0,0], [0,measurement_noise,0,0], [0,0,1000,0], [0,0,0,1000]] 


## Part 3 - Tracking down the runaway robot
### Problem
In this scenario, I have another robot that can travel about twice as fast as the runaway bot. 

Every time step ‘next_move’ is called the position and heading of my bot, the most recent (x,y) measurement of the runaway bot, the max distance my bot can move, and an ‘OTHER’ variable used to maintain state between calls. In ‘next_move’, I should return the turning ( angle to turn the robot)  and distance I want for my bot. 

The goal is to get within 0.01 step sizes of the runaway robot.

### My Solution
In ‘next_move’, I predict the target position of the runaway robot as I did in part 2.  I compare the predicted runaway position against my robot’s current position to get the turning and distance needed to get to the runaway robot’s future position. If the distance between my current robot and the runaway’s position is bigger than my max distance, I travel as far as I can in that direction.

## Part 4 - Tracking down robot with the same speed
### Problem
The goal for this part is still to track down the robot, but this time the speed of our robot is about the same as the speed as the runaway bot.

### My Solution
The solution in Part 3 does not work for when our bot and the runaway’s bot are going the same speed. We simply have our robot hopelessly following the runaway bot forever.

In ‘next_move’, I still estimate the runaway bot’s next position as we did in Part 3. But now I check to see if the distance between my robot’s position and the runaway’s next position is greater than  the max_distance I can move per time step. If it is, I predict 2 steps further. I predict the runaway’s position 3 time steps from now, and head toward that future position.  If the runaway’s next step is within our reach, I head toward that next step as we did in Part 3.

 









