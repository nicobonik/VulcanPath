import numpy as np
import serial
import pygame
import time
import math

import DriveConstants
from ParametricSplineUtil import parametric_spline_interpolate
from Point import Point
from Point import PathPoint
import threading

from PID import PID
from Pose import Pose
from tvc import SplineTrajectory, TrajectoryFollower

ser = serial.Serial('/dev/cu.usbmodem14101', 9600, timeout=0.1)

x = 0.0
y = 0.0
theta = 0.0
running = True
runAuto = False
turn_pid = PID(50, 0.1, -1, -150, 150)
velo_pid = PID(2, 0, 0, 0, 255)
point_list = []
elapsed_time = 1
last_time = 0
target_pose = Pose(0, 0, 0)


def robot():
    radius = 2.75 / 2.0
    wheelBase = 10.25
    ticksPerRev = 160.0

    lastRightPosition = 0.0
    lastLeftPosition = 0.0
    velocity_avg_cycles = 5
    velocity_avg_temp = 0
    velocity_avg_counter = 0
    corrected_velocity = 0

    start = False

    global x, last_time, elapsed_time
    global y
    global theta

    global running
    global runAuto
    global point_list
    global target_pose

    point_list = [PathPoint(0, 0, 1, 10), PathPoint(15, 10, 1, 10), PathPoint(25, -10, 1, 10), PathPoint(50, 5, 1, 10)]
    interpolated_spline = parametric_spline_interpolate([0.0, 15.0, 20.0], [0.0, 10.0, 5.0])
    traj = SplineTrajectory(interpolated_spline)
    trajectory_follower = TrajectoryFollower(traj)

    time.sleep(3)
    ser.flush()
    velocity = 0.00

    while running:
        time_start = time.time_ns()
        powers = ""
        powerLeft = 0
        powerRight = 0

        keys = pygame.key.get_pressed()
        if keys[pygame.K_LEFT]:
            powerLeft -= 255
            powerRight += 255
        if keys[pygame.K_RIGHT]:
            powerLeft += 255
            powerRight -= 255
        if keys[pygame.K_UP]:
            powerLeft += 255
            powerRight += 255
        if keys[pygame.K_DOWN]:
            powerLeft -= 255
            powerRight -= 255

        powerList = (powerLeft, powerRight)

        if runAuto:
            if not start:
                trajectory_follower.start()
                start = True
            # last_point = point_list[len(point_list) - 1]
            # follow_point = find_follow_point(point_list)
            # # if math.hypot(last_point.x - x, last_point.y - y) <= follow_point.lookahead + 0.5:
            # #     # print("stopping")
            # #     follow_point.speed = 0
            # powerList = moveToPoint(follow_point.x, follow_point.y, 30 * follow_point.speed, x, y, theta)
            if trajectory_follower.running:

                trajectory_follower.run_async()
                target_pose.set_pose(trajectory_follower.follower_pose)
                powerList = drive_with_velocity(DriveConstants.max_velocity,
                                trajectory_follower.trajectory.get_spline_angle(trajectory_follower.get_elapsed_time()),
                                corrected_velocity, theta)
                # powerList = moveToPoint(target_pose.x, target_pose.y, DriveConstants.max_velocity * (ticksPerRev / (2.0 * math.pi * radius)), x, y, theta)

        if not runAuto:
            turn_pid.reset()
            velo_pid.reset()
            start = False

        powerLeft = clamp(powerList[0], -255, 255)
        powerRight = clamp(powerList[1], -255, 255)

        powers += str(powerLeft)
        powers += ","
        powers += str(powerRight)
        powers += "\n"

        ser.write(powers.encode('utf-8'))
        # print("writing")

        input = ser.readline().decode('utf-8').strip()
        # print("reading")
        # print(str(input))

        inputList = input.split(", ")
        leftPosition = float(inputList[0])
        rightPosition = float(inputList[1])
        # print(leftPosition)

        vl = float(leftPosition - lastLeftPosition)
        vr = float(rightPosition - lastRightPosition)

        velocity = (vl + vr) / 2.0
        dTheta = (vl - vr) / (wheelBase * (ticksPerRev / (2.0 * math.pi * radius)))

        x = x + (velocity * math.cos(theta + (dTheta / 2.0))) * ((2.0 * math.pi * radius) / ticksPerRev)
        y = y + (velocity * math.sin(theta + (dTheta / 2.0))) * ((2.0 * math.pi * radius) / ticksPerRev)
        theta = theta + dTheta

        if velocity_avg_counter < velocity_avg_cycles:
            velocity_avg_temp += velocity
            velocity_avg_counter += 1
        else:
            velocity_avg_temp += velocity
            corrected_velocity = velocity_avg_temp / 5
            velocity_avg_temp = 0
            velocity_avg_counter = 0
            elapsed_time = ((time.time_ns() - last_time) / 5000000000)
            last_time = time.time_ns()
            corrected_velocity = (corrected_velocity * ((2.0 * math.pi * radius) / ticksPerRev)) / elapsed_time

        # velocity_dt = velocity * (ticksPerRev / (2.0 * math.pi * radius))
        # print(str((time.time_ns() - last_time) / 1000000000))
        # print(str(x) + ", " + str(y) + ", " + str(theta))

        lastLeftPosition = leftPosition
        lastRightPosition = rightPosition
        print((time.time_ns() - time_start) / 1000000000)


def main():
    pygame.init()

    screen = pygame.display.set_mode((1000, 500))

    global running
    global runAuto
    global point_list
    global target_pose

    # bot = pygame.rect.Rect(200, 200, 50, 50)
    interpolated_spline = parametric_spline_interpolate([0.0, 15.0, 20.0], [0.0, 10.0, 5.0])
    points = np.linspace(0, len(interpolated_spline.get_knots()) - 1, 100)
    xy_points = []
    for point in points:
        xy_points.append(interpolated_spline.get(point))

    multiplier = 5

    while running:
        global x
        global y
        global theta

        screen.fill((128, 128, 128))
        bot = pygame.rect.Rect(200 + (x * multiplier), 200 + (y * multiplier), 50, 50)
        sim = pygame.rect.Rect(200 + (target_pose.x * multiplier), 200 + (target_pose.y*multiplier), 50, 50)
        # print(str(bot.center))
        # for i in range(len(point_list) - 1):
        #     p1 = point_list[i]
        #     p2 = point_list[i + 1]
        #     pygame.draw.line(screen, (128, 0, 0), (225 + (p1.x * multiplier), 225 + (p1.y * multiplier)),
        #                      (225 + (p2.x * multiplier), 225 + (p2.y * multiplier)), 3)
        for i in range(len(points) - 1):
            pygame.draw.line(screen, (128, 0, 0),
                             (225 + (multiplier * xy_points[i][0]), 225 + (multiplier * xy_points[i][1])),
                             (225 + (multiplier * xy_points[i + 1][0]), 225 + (multiplier * xy_points[i + 1][1])),
                             3)

        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:
                    x = 0.0
                    y = 0.0
                    theta = 0.0
                if event.key == pygame.K_s:
                    runAuto = not runAuto
            if event.type == pygame.QUIT:
                running = False

        pygame.draw.rect(screen, (0, 128, 0), sim)
        pygame.draw.line(screen, (0, 0, 128), sim.center, (sim.center[0] + 60 * math.cos(target_pose.heading), sim.center[1] + 60*math.sin(target_pose.heading)), 3)
        pygame.draw.rect(screen, (0, 0, 128), bot)
        # print(bot.center[0])
        pygame.draw.line(screen, (0, 128, 0), bot.center,
                         (bot.center[0] + 60 * (math.cos(theta)), bot.center[1] + 60 * (math.sin(theta))), 3)
        pygame.display.flip()
        # time.sleep(0.01)


def clamp(num, min_value, max_value):
    return max(min(num, max_value), min_value)


def drive_with_velocity(target_velocity, target_heading, velocity, heading):
    velocity_output = velo_pid.update(target_velocity, velocity) + 1.5*(target_velocity * (160 / (2.0 * math.pi * 1.375)))
    heading_output = turn_pid.update(target_heading, heading)
    left_power = velocity_output + heading_output
    right_power = velocity_output - heading_output
    # print(velo_pid.lastError)
    return left_power, right_power


def moveToPoint(x, y, speed, robotX, robotY, robotTheta):
    distanceToPoint = math.hypot(x - robotX, y - robotY)
    absoluteAngleToPoint = math.atan2(y - robotY, x - robotX)
    relativeAngleToPoint = absoluteAngleToPoint - robotTheta

    turn_speed = 2

    vd = distanceToPoint * speed
    if speed == 0:
        turn_speed = 0
    w = turn_pid.update(absoluteAngleToPoint, robotTheta) * turn_speed
    # w = relativeAngleToPoint *

    left_power = vd + w
    right_power = vd - w

    return left_power, right_power


def find_follow_point(path):
    global x
    global y
    global theta

    follow_point = PathPoint(0, 0, 0, 0)
    circle_intersections = []

    if len(path) == 2:
        path.append(path[len(path) - 1])

    for i in range(len(path) - 1):
        start = path[i]
        end = path[i + 1]

        if path.index(end) == len(path) - 1:
            if end.x < start.x:
                circle_intersections = line_circle_intersect(start.to_point(), end.to_point(), end.lookahead,
                                                             Point(x, y), False, True)
            else:
                circle_intersections = line_circle_intersect(start.to_point(), end.to_point(), end.lookahead,
                                                             Point(x, y), True, False)
        else:
            circle_intersections = line_circle_intersect(start.to_point(), end.to_point(), end.lookahead, Point(x, y),
                                                         False, True)

        closest_angle = 1000000
        for intersection in circle_intersections:
            angle = math.atan2(intersection.y - y, intersection.x - x)
            d_theta = abs(angle - theta)
            if d_theta < closest_angle:
                closest_angle = d_theta
                follow_point.set_pathpoint(end)
                follow_point.set_point(intersection)

        if path.index(end) == len(path) - 1:
            motion_profile_distance = 15
            min_speed = 0
            distance_to_target = math.hypot(end.x - x, end.y - y)
            if distance_to_target < motion_profile_distance:
                # print("changing speed")
                m = (1 - min_speed) / motion_profile_distance
                follow_point.speed *= m * (distance_to_target - motion_profile_distance) + 1

    # print(str(follow_point.x))
    return follow_point


def line_circle_intersect(line_point_1, line_point_2, radius, circle_center, min_box, max_box):
    # print("yeet")
    intersections = []
    try:
        m = (line_point_2.y - line_point_1.y) / (line_point_2.x - line_point_1.x)
        b = line_point_1.y - (m * line_point_1.x)
        quadratic_a = math.pow(m, 2) + 1
        quadratic_b = 2.0 * ((m * b) - (m * circle_center.y) - circle_center.x)
        quadratic_c = math.pow(circle_center.y, 2) - math.pow(radius, 2) + math.pow(circle_center.x, 2) - (
                    2.0 * b * circle_center.y) + math.pow(b, 2)
        if line_point_2.x - line_point_1.x == 0:
            x1 = line_point_2.x
            y1 = math.sqrt(
                (-1.0 * math.pow(x1, 2)) + (2.0 * x1 * circle_center.x) - math.pow(circle_center.y, 2) + math.pow(
                    radius, 2)) + circle_center.y

            min_y = min(line_point_1.y, line_point_2.y)
            max_y = max(line_point_1.y, line_point_2.y)
            if min_box and max_box:
                if min_y < y1 < max_y:
                    intersections.append(Point(x1, y1))
            else:
                if max_box:
                    if y1 < max_y:
                        intersections.append(Point(x1, y1))
                else:
                    if min_box:
                        if y1 > min_y:
                            intersections.append(Point(x1, y1))
            y2 = (-1.0 * math.sqrt(
                (-1.0 * math.pow(x1, 2)) + (2.0 * x1 * circle_center.x) - math.pow(circle_center.y, 2) + math.pow(
                    radius, 2))) + circle_center.y
            if min_box and max_box:
                if min_y < y1 < max_y:
                    intersections.append(Point(x1, y2))
            else:
                if max_box:
                    if y1 < max_y:
                        intersections.append(Point(x1, y2))
                else:
                    if min_box:
                        if y1 > min_y:
                            intersections.append(Point(x1, y2))

        else:
            if line_point_2.y == line_point_1.y:
                x1 = circle_center.x + math.sqrt(
                    (-1.0 * math.pow(line_point_2.y, 2)) + (2.0 * line_point_2.y * circle_center.y) - math.pow(
                        circle_center.y, 2) + radius)
                y1 = line_point_2.y
                min_y = min(line_point_1.y, line_point_2.y)
                max_y = max(line_point_1.y, line_point_2.y)
                if min_box and max_box:
                    if min_y < y1 < max_y:
                        intersections.append(Point(x1, y1))
                else:
                    if max_box:
                        if y1 < max_y:
                            intersections.append(Point(x1, y1))
                    else:
                        if min_box:
                            if y1 > min_y:
                                intersections.append(Point(x1, y1))

                x2 = circle_center.x - math.sqrt(
                    (-1.0 * math.pow(line_point_2.y, 2)) + (2.0 * line_point_2.y * circle_center.y) - math.pow(
                        circle_center.y, 2) + radius)
                if min_box and max_box:
                    if min_y < y1 < max_y:
                        intersections.append(Point(x2, y1))
                else:
                    if max_box:
                        if y1 < max_y:
                            intersections.append(Point(x2, y1))
                    else:
                        if min_box:
                            if y1 > min_y:
                                intersections.append(Point(x2, y1))
            else:
                x1 = (-quadratic_b + math.sqrt(math.pow(quadratic_b, 2) - (4.0 * quadratic_a * quadratic_c))) / (
                            2.0 * quadratic_a)
                y1 = (m * x1) + b
                min_y = min(line_point_1.y, line_point_2.y)
                max_y = max(line_point_1.y, line_point_2.y)
                if min_box and max_box:
                    if min_y < y1 < max_y:
                        intersections.append(Point(x1, y1))
                else:
                    if max_box:
                        if y1 < max_y:
                            intersections.append(Point(x1, y1))
                    else:
                        if min_box:
                            if y1 > min_y:
                                intersections.append(Point(x1, y1))
                x2 = (-quadratic_b - math.sqrt(math.pow(quadratic_b, 2) - (4.0 * quadratic_a * quadratic_c))) / (
                            2.0 * quadratic_a)
                y2 = (m * x2) + b
                if min_box and max_box:
                    if min_y < y2 < max_y:
                        intersections.append(Point(x2, y2))
                else:
                    if max_box:
                        if y2 < max_y:
                            intersections.append(Point(x2, y2))
                    else:
                        if min_box:
                            if y2 > min_y:
                                intersections.append(Point(x2, y2))
    except Exception:
        pass
    return intersections


if __name__ == "__main__":
    robot_thread = threading.Thread(target=robot)
    robot_thread.start()
    # call the main function
    main()
