# ----------
# Background
# 
# A robotics company named Trax has created a line of small self-driving robots 
# designed to autonomously traverse desert environments in search of undiscovered
# water deposits.
#
# A Traxbot looks like a small tank. Each one is about half a meter long and drives
# on two continuous metal tracks. In order to maneuver itself, a Traxbot can do one
# of two things: it can drive in a straight line or it can turn. So to make a 
# right turn, A Traxbot will drive forward, stop, turn 90 degrees, then continue
# driving straight.
#
# This series of questions involves the recovery of a rogue Traxbot. This bot has 
# gotten lost somewhere in the desert and is now stuck driving in an almost-circle: it has
# been repeatedly driving forward by some step size, stopping, turning a certain 
# amount, and repeating this process... Luckily, the Traxbot is still sending all
# of its sensor data back to headquarters.
#
# In this project, we will start with a simple version of this problem and 
# gradually add complexity. By the end, you will have a fully articulated
# plan for recovering the lost Traxbot.
# 
# ----------
# Part One
#
# Let's start by thinking about circular motion (well, really it's polygon motion
# that is close to circular motion). Assume that Traxbot lives on 
# an (x, y) coordinate plane and (for now) is sending you PERFECTLY ACCURATE sensor 
# measurements. 
#
# With a few measurements you should be able to figure out the step size and the 
# turning angle that Traxbot is moving with.
# With these two pieces of information, you should be able to 
# write a function that can predict Traxbot's next location.
#
# You can use the robot class that is already written to make your life easier. 
# You should re-familiarize yourself with this class, since some of the details
# have changed. 
#
# ----------
# YOUR JOB
#
# Complete the estimate_next_pos function. You will probably want to use
# the OTHER variable to keep track of information about the runaway robot.
#
# ----------
# GRADING
# 
# We will make repeated calls to your estimate_next_pos function. After
# each call, we will compare your estimated position to the robot's true
# position. As soon as you are within 0.01 stepsizes of the true position,
# you will be marked correct and we will tell you how many steps it took
# before your function successfully located the target bot.

# These import steps give you access to libraries which you may (or may
# not) want to use.
from robot import *
from math import *
from matrix import *
import random


# This is the function you have to write. The argument 'measurement' is a 
# single (x, y) point. This function will have to be called multiple
# times before you have enough information to accurately predict the
# next position. The OTHER variable that your function returns will be 
# passed back to your function the next time it is called. You can use
# this to keep track of important information over time.
import math

def kalman_estimate(measurement, OTHER = None):
    if OTHER == None:
        x = matrix([[test_target.heading], [test_target.distance], [test_target.turning], [0.]]) # initial state (location and velocity)
        P =  matrix([[0,0,0,0],[0,0,0,0],[0,0,1000,0],[0,0,0,1000]])# initial uncertainty: 0 for positions x and y, 1000 for the two velocities
        
        prev_point = [test_target.x, test_target.y]
    else:
        x = OTHER['x']
        P = OTHER['P']
        
        prev_point = OTHER['prev']    
    dx = measurement[0] - prev_point[0]
    dy = measurement[1] - prev_point[1]
    m_heading = math.atan2(dy, dx)
    if m_heading < 0:
        m_heading += 2*pi
    m_distance = distance_between(prev_point, measurement)

    # measurement update
    Z = matrix([[m_heading, m_distance]])

    y = Z.transpose() - (H * x)
    S = H * P * H.transpose() + R
    K = P * H.transpose() * S.inverse()
    x = x + (K * y)
    P = (I - (K * H)) * P
    
    # prediction
    x = (F * x) + u
    P = F * P * F.transpose()
    
    heading = x.value[0][0]
    distance = x.value[1][0]

    new_x = distance * cos(heading)
    new_y = distance * sin(heading)
    xy_estimate = [measurement[0] + new_x, measurement[1] + new_y]
    return xy_estimate, {'x': x, 'P': P, 'prev': measurement} 
    
def get_h_k_r_for_circle(measurements):
    sum_x = sum([m[0] for m in measurements])
    sum_x_2 = sum([m[0]*m[0] for m in measurements])
    sum_y = sum([m[1] for m in measurements])
    sum_y_2 = sum([m[1]*m[1] for m in measurements])
    sum_x_y = sum([m[0]* m[1] for m in measurements])
    n = len(measurements)
    
    sum_1 = sum([m[0]*(m[0]*m[0] + m[1]*m[1]) for m in measurements])
    sum_2 = sum([m[1]*(m[0]*m[0] + m[1]*m[1]) for m in measurements])
    sum_3 = sum([(m[0]*m[0] + m[1]*m[1]) for m in measurements])
    
    matrix_1 = matrix([[sum_x_2, sum_x_y, sum_x],[sum_x_y, sum_y_2, sum_y],[sum_x, sum_y, n]])
    matrix_2 = matrix([[sum_1],[sum_2],[sum_3]])
    abc = matrix_1.inverse() * matrix_2
    print abc.value
    A = abc.value[0][0]
    B = abc.value[1][0]
    C = abc.value[2][0]
    
    h  = A/2.
    k = B/2.
    r = (pow(4*C + A*A + B*B,.5))/2.
    return h, k, r   
    


def estimate_next_pos(measurement, OTHER = None):
    """Estimate the next (x, y) position of the wandering Traxbot
    based on noisy (x, y) measurements."""

    # You must return xy_estimate (x, y), and OTHER (even if it is None) 
    # in this order for grading purposes.
    
    if OTHER == None:
        OTHER = {'measurements': [measurement]}
    else:
        OTHER['measurements'].append(measurement)

    angle = test_target.heading + test_target.turning
    
    
    if len(OTHER['measurements']) >= 3:
        h, k, r = get_h_k_r_for_circle(OTHER['measurements'])
        print h, k, r
        circle_angle = test_target.heading - (pi/2.)
        print "measurement: ", measurement
        print "x on circle: ", r * math.cos(circle_angle) + h
        print "y on circle: ", r * math.sin(circle_angle) + k
        x = r * math.cos(circle_angle) + h
        y = r * math.sin(circle_angle) + k
        xy_estimate = [x, y]
        # print "estimage ", xy_estimate
    else:
        xy_estimate = measurement
    
    
    
    


    return xy_estimate, {'prev': measurement, 'measurements': OTHER['measurements']} 

# A helper function you may find useful.
def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

# This is here to give you a sense for how we will be running and grading
# your code. Note that the OTHER variable allows you to store any 
# information that you want. 
def demo_grading(estimate_next_pos_fcn, target_bot, OTHER = None):
    localized = False
    distance_tolerance = 0.01 * target_bot.distance 
    ctr = 0
    # if you haven't localized the target bot, make a guess about the next
    # position, then we move the bot and compare your guess to the true
    # next position. When you are close enough, we stop checking.
    while not localized and ctr <= 10: 
        ctr += 1
        measurement = target_bot.sense()
        position_guess, OTHER = estimate_next_pos_fcn(measurement, OTHER)
        target_bot.move_in_circle()
        true_position = (target_bot.x, target_bot.y)
        error = distance_between(position_guess, true_position)
        if error <= distance_tolerance:
            print "You got it right! It took you ", ctr, " steps to localize."
            localized = True
        if ctr == 10:
            print "Sorry, it took you too many steps to localize the target."
    return localized

# This is a demo for what a strategy could look like. This one isn't very good.
def naive_next_pos(measurement, OTHER = None):
    """This strategy records the first reported position of the target and
    assumes that eventually the target bot will eventually return to that 
    position, so it always guesses that the first position will be the next."""
    if not OTHER: # this is the first measurement
        OTHER = measurement
    xy_estimate = OTHER 
    return xy_estimate, OTHER

# This is how we create a target bot. Check the robot.py file to understand
# How the robot class behaves.
test_target = robot(2.1, 4.3, 0.5, 2*pi / 40.0, 1.5)
test_target.set_noise(0.0, 0.0, 0.0)

dt = .1

# P =  matrix([[0,0,0,0],[0,0,0,0],[0,0,1000,0],[0,0,0,1000]])# initial uncertainty: 0 for positions x and y, 1000 for the two velocities
# F =  matrix([[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]])# next state function: generalize the 2d version to 4d
# H =  matrix([[1,0,0,0],[0,1,0,0]])# measurement function: reflect the fact that we observe x and y but not the two velocities
# R =  matrix([[0.1, 0],[0, 0.1]])# measurement uncertainty: use 2x2 matrix with 0.1 as main diagonal
# I =  matrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])# 4d identity matrix

# u = matrix([[0.], [0.], [0.], [0.]]) # external motion



P =  matrix([[0,0], [0,0]])# initial uncertainty: 0 for positions x and y, 1000 for the two velocities
F =  matrix([[1,dt],[0,1]])# next state function: generalize the 2d version to 4d
H =  matrix([[1,0]])# measurement function: reflect the fact that we observe x and y but not the two velocities
R =  matrix([[0.1]])# measurement uncertainty: use 2x2 matrix with 0.1 as main diagonal
I =  matrix([[1,0],[0,1]])# 4d identity matrix

u = matrix([[0.], [0.]]) # external motion

demo_grading(estimate_next_pos, test_target)
