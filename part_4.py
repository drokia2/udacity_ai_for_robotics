# ----------
# Part Four
#
# Again, you'll track down and recover the runaway Traxbot. 
# But this time, your speed will be about the same as the runaway bot. 
# This may require more careful planning than you used last time.
#
# ----------
# YOUR JOB
#
# Complete the next_move function, similar to how you did last time. 
#
# ----------
# GRADING
# 
# Same as part 3. Again, try to catch the target in as few steps as possible.

from robot import *
from math import *
from matrix import *
import random
import time

def convert_measurement_into_polar_coordinates(measurement, prev_point):
    dx = measurement[0] - prev_point[0]
    dy = measurement[1] - prev_point[1]
    m_heading = atan2(dy, dx)
    if m_heading < 0:
        m_heading += 2*pi
    m_distance = distance_between(prev_point, measurement)
    return m_heading, m_distance

def kalman_prediction(x, P, Z):
    y = Z.transpose() - (H * x)
    S = H * P * H.transpose() + R
    K = P * H.transpose() * S.inverse()
    x = x + (K * y)
    P = (I - (K * H)) * P

    # prediction
    x = (F * x) + u
    P = F * P * F.transpose()
    return x, P

def estimate_target_position(measurement, OTHER= None):
    if OTHER != None:
        m_heading, m_distance = convert_measurement_into_polar_coordinates(measurement, OTHER['prev'])
        print "*****************"
        print "m_heading orig: ", m_heading

        if 'x' not in OTHER.keys():
            x = matrix([[m_heading], [m_distance], [0.], [0.]]) # initial state (location and velocity)
            P =  matrix([[measurement_noise,0,0,0],[0,measurement_noise,0,0],[0,0,1000,0],[0,0,0,1000]])# initial uncertainty: 0 for positions x and y, 1000 for the two velocities
        else :
            x = OTHER['x']
            P = OTHER['P']

        while m_heading < (x.value[0][0] - pi): # off by a rotation
            m_heading += 2*pi

        print "m_heading after: ", m_heading
    
        Z = matrix([[m_heading, m_distance]])
        x, P = kalman_prediction(x, P, Z)
        
        heading = x.value[0][0]
        distance = x.value[1][0]
        print "heading: ", heading
        print "distance: ", distance


        new_x = distance * cos(heading)
        new_y = distance * sin(heading)
        xy_estimate = [measurement[0] + new_x, measurement[1] + new_y]
        OTHER = {'x': x, 'P': P, 'prev': measurement} 
    else:
        xy_estimate = measurement
        OTHER = {'prev': measurement} 
    return xy_estimate, OTHER  

def turning_and_distance_to_target_position(xy_estimate, hunter_position, hunter_heading, max_distance):
    distance_from_estimate = distance_between(xy_estimate, hunter_position)
    dy = xy_estimate[1] - hunter_position[1]
    dx = xy_estimate[0] - hunter_position[0]
    angle_to_estimate = atan2(dy, dx)

    angle_to_estimate = angle_trunc(angle_to_estimate)
    
    turning = angle_to_estimate - hunter_heading
    turning = angle_trunc(turning)

    distance = min(distance_from_estimate, max_distance) 
    return turning, distance

def next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER = None):
    # This function will be called after each time the target moves. 

    # The OTHER variable is a place for you to store any historical information about
    # the progress of the hunt (or maybe some localization information). Your return format
    # must be as follows in order to be graded properly.
    
    xy_estimate, OTHER = estimate_target_position(target_measurement, OTHER)
    turning, distance = turning_and_distance_to_target_position(xy_estimate, hunter_position, hunter_heading, max_distance)
    
    return turning, distance, OTHER

def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def demo_grading(hunter_bot, target_bot, next_move_fcn, OTHER = None):
    """Returns True if your next_move_fcn successfully guides the hunter_bot
    to the target_bot. This function is here to help you understand how we 
    will grade your submission."""
    max_distance = 0.98 * target_bot.distance # 0.98 is an example. It will change.
    separation_tolerance = 0.02 * target_bot.distance # hunter must be within 0.02 step size to catch target
    caught = False
    ctr = 0
    #For Visualization
    import turtle
    window = turtle.Screen()
    window.bgcolor('white')
    chaser_robot = turtle.Turtle()
    chaser_robot.shape('arrow')
    chaser_robot.color('blue')
    chaser_robot.resizemode('user')
    chaser_robot.shapesize(0.3, 0.3, 0.3)
    broken_robot = turtle.Turtle()
    broken_robot.shape('turtle')
    broken_robot.color('green')
    broken_robot.resizemode('user')
    broken_robot.shapesize(0.3, 0.3, 0.3)
    size_multiplier = 15.0 #change size of animation
    chaser_robot.hideturtle()
    chaser_robot.penup()
    chaser_robot.goto(hunter_bot.x*size_multiplier, hunter_bot.y*size_multiplier-100)
    chaser_robot.showturtle()
    broken_robot.hideturtle()
    broken_robot.penup()
    broken_robot.goto(target_bot.x*size_multiplier, target_bot.y*size_multiplier-100)
    broken_robot.showturtle()
    measuredbroken_robot = turtle.Turtle()
    measuredbroken_robot.shape('circle')
    measuredbroken_robot.color('red')
    measuredbroken_robot.penup()
    measuredbroken_robot.resizemode('user')
    measuredbroken_robot.shapesize(0.1, 0.1, 0.1)
    broken_robot.pendown()
    chaser_robot.pendown()
    #End of Visualization
    # We will use your next_move_fcn until we catch the target or time expires.
    while not caught and ctr < 1000:

        if ctr > 60:
            time.sleep(5)
        # Check to see if the hunter has caught the target.
        hunter_position = (hunter_bot.x, hunter_bot.y)
        target_position = (target_bot.x, target_bot.y)
        separation = distance_between(hunter_position, target_position)
        if separation < separation_tolerance:
            print "You got it right! It took you ", ctr, " steps to catch the target."
            caught = True

        # The target broadcasts its noisy measurement
        target_measurement = target_bot.sense()

        # This is where YOUR function will be called.
        turning, distance, OTHER = next_move_fcn(hunter_position, hunter_bot.heading, target_measurement, max_distance, OTHER)

        # Don't try to move faster than allowed!
        if distance > max_distance:
            distance = max_distance

        # We move the hunter according to your instructions
        hunter_bot.move(turning, distance)

        # The target continues its (nearly) circular motion.
        target_bot.move_in_circle()
        #Visualize it
        measuredbroken_robot.setheading(target_bot.heading*180/pi)
        measuredbroken_robot.goto(target_measurement[0]*size_multiplier, target_measurement[1]*size_multiplier-100)
        measuredbroken_robot.stamp()
        broken_robot.setheading(target_bot.heading*180/pi)
        broken_robot.goto(target_bot.x*size_multiplier, target_bot.y*size_multiplier-100)
        chaser_robot.setheading(hunter_bot.heading*180/pi)
        chaser_robot.goto(hunter_bot.x*size_multiplier, hunter_bot.y*size_multiplier-100)
        #End of visualization
        ctr += 1            
        if ctr >= 1000:
            print "It took too many steps to catch the target."
    return caught



def angle_trunc(a):
    """This maps all angles to a domain of [-pi, pi]"""
    while a < 0.0:
        a += pi * 2
    return ((a + pi) % (pi * 2)) - pi

def get_heading(hunter_position, target_position):
    """Returns the angle, in radians, between the target and hunter positions"""
    hunter_x, hunter_y = hunter_position
    target_x, target_y = target_position
    heading = atan2(target_y - hunter_y, target_x - hunter_x)
    heading = angle_trunc(heading)
    return heading

def naive_next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER):
    """This strategy always tries to steer the hunter directly towards where the target last
    said it was and then moves forwards at full speed. This strategy also keeps track of all 
    the target measurements, hunter positions, and hunter headings over time, but it doesn't 
    do anything with that information."""
    if not OTHER: # first time calling this function, set up my OTHER variables.
        measurements = [target_measurement]
        hunter_positions = [hunter_position]
        hunter_headings = [hunter_heading]
        OTHER = (measurements, hunter_positions, hunter_headings) # now I can keep track of history
    else: # not the first time, update my history
        OTHER[0].append(target_measurement)
        OTHER[1].append(hunter_position)
        OTHER[2].append(hunter_heading)
        measurements, hunter_positions, hunter_headings = OTHER # now I can always refer to these variables
    
    heading_to_target = get_heading(hunter_position, target_measurement)
    heading_difference = heading_to_target - hunter_heading
    turning =  heading_difference # turn towards the target
    distance = max_distance # full speed ahead!
    return turning, distance, OTHER

target = robot(0.0, 10.0, 0.0, 2*pi / 30, 1.5)
measurement_noise = .05*target.distance
target.set_noise(0.0, 0.0, measurement_noise)

hunter = robot(-10.0, -10.0, 0.0)

dt = .1

P =  matrix([[0,0,0,0],[0,0,0,0],[0,0,1000,0],[0,0,0,1000]])# initial uncertainty: 0 for positions x and y, 1000 for the two velocities
F =  matrix([[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]])# next state function: generalize the 2d version to 4d
H =  matrix([[1,0,0,0],[0,1,0,0]])# measurement function: reflect the fact that we observe x and y but not the two velocities
R =  matrix([[measurement_noise, 0],[0, measurement_noise]])# measurement uncertainty: use 2x2 matrix with 0.1 as main diagonal
I =  matrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])# 4d identity matrix

u = matrix([[0.], [0.], [0.], [0.]]) # external motion


print demo_grading(hunter, target, next_move)





