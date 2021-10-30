import math

import pygame
import numpy as np

from ParametricSplineUtil import parametric_spline_interpolate
from Pose import Pose
from tvc import SplineTrajectory, TrajectoryFollower

runAuto = False
robot_pose = Pose(0, 0, 0)

def main():
    pygame.init()

    global runAuto
    global robot_pose

    # define trajectory with an interpolated spline and initialize a follower
    interpolated_spline = parametric_spline_interpolate([0.0, 30.0, 40.0, 20.0, 0.0], [0.0, 20.0, 5.0, 10.0, 0.0])
    traj = SplineTrajectory(interpolated_spline)
    trajectory_follower = TrajectoryFollower(traj)

    # setup for drawing the spline on screen
    points = np.linspace(0, len(interpolated_spline.get_knots()) - 1, 100)
    xy_points = []
    for point in points:
        xy_points.append(interpolated_spline.get(point))

    screen = pygame.display.set_mode((1000, 500))
    multiplier = 10
    offset = 225
    running = True
    start = False
    while running:

        screen.fill((128, 128, 128))

        bot = pygame.rect.Rect(200 + (robot_pose.x * multiplier), 200 + (robot_pose.y * multiplier), 50, 50)

        if runAuto:
            if not start:
                trajectory_follower.start()
                start = True
            if trajectory_follower.running:
                trajectory_follower.run_async()
                robot_pose.set_pose(trajectory_follower.follower_pose)
        else:
            start = False

        for i in range(len(points) - 1):
            pygame.draw.line(screen, (128, 0, 0), (offset + (multiplier * xy_points[i][0]), offset + (multiplier * xy_points[i][1])),
                             (offset + (multiplier * xy_points[i+1][0]), offset + (multiplier * xy_points[i+1][1])), 3)

        pygame.draw.rect(screen, (0, 0, 128), bot)
        pygame.draw.line(screen, (0, 128, 0), bot.center, (bot.center[0] + 60 * math.cos(robot_pose.heading), bot.center[1] + 60 * math.sin(robot_pose.heading)), 2)

        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_s:
                    # print(trajectory_follower.running)
                    runAuto = not runAuto
            if event.type == pygame.QUIT:
                running = False

        pygame.display.flip()


if __name__ == '__main__':
    main()
