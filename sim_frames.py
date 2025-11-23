import math
import random
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from frames_math import (
    rotation_matrix_2d,
    transform_point_2d,
    compose_transform_2d,
)


# --------------------------------------------------
# Simulation parameters
# --------------------------------------------------

DT = 0.1          # time step (seconds, just conceptual)
STEPS = 200       # number of frames in the animation

# Robot pose in MAP frame (x, y, theta)
robot_x = 0.0
robot_y = 0.0
robot_theta = 0.0  # radians


# --------------------------------------------------
# Fixed transforms
# --------------------------------------------------
# 1) map -> odom (we keep it identity here for simplicity)
R_map_odom = rotation_matrix_2d(0.0)
t_map_odom = (0.0, 0.0)

# 2) base_link -> camera (camera mounted on robot)
# camera is 0.3 m ahead and 0.1 m above base_link, rotated slightly downwards
R_base_camera = rotation_matrix_2d(0.0)  # assume camera faces same direction as base_link in 2D
t_base_camera = (0.3, 0.1)

# Ball position in camera frame (as if detected by vision)
p_ball_camera = (0.6, -0.2)


# --------------------------------------------------
# Matplotlib setup
# --------------------------------------------------
fig, ax = plt.subplots()
ax.set_aspect('equal', 'box')
ax.set_xlim(-2, 6)
ax.set_ylim(-2, 6)
ax.set_title("Coordinate Frames: map, odom, base_link, camera")

# Artists (things we update every frame)
map_origin_plot, = ax.plot([], [], 'ko', label='map origin')
odom_origin_plot, = ax.plot([], [], 'bo', label='odom origin')
base_origin_plot, = ax.plot([], [], 'ro', label='base_link origin')
camera_origin_plot, = ax.plot([], [], 'go', label='camera origin')

ball_map_plot, = ax.plot([], [], 'yo', label='ball (map frame)')
ball_camera_plot, = ax.plot([], [], 'yx', label='ball (camera ray)')

# Arrows for axes
map_x_arrow = ax.arrow(0, 0, 0, 0)   # placeholders, will replace
map_y_arrow = ax.arrow(0, 0, 0, 0)
odom_x_arrow = ax.arrow(0, 0, 0, 0)
odom_y_arrow = ax.arrow(0, 0, 0, 0)
base_x_arrow = ax.arrow(0, 0, 0, 0)
base_y_arrow = ax.arrow(0, 0, 0, 0)
camera_x_arrow = ax.arrow(0, 0, 0, 0)
camera_y_arrow = ax.arrow(0, 0, 0, 0)

# We will override arrows by drawing fresh each frame,
# so keep track of them in a list for easy removal.
axis_arrows = []


def draw_frame_axes(origin_x, origin_y, R, length=0.5, color_x='r', color_y='g'):
    """
    Draw x and y axes for a frame given its origin and rotation matrix R.
    Returns the arrow artists.
    """
    (r00, r01), (r10, r11) = R

    # x-axis direction (first column of R)
    x_dir = (r00, r10)
    # y-axis direction (second column of R)
    y_dir = (r01, r11)

    x_arrow = ax.arrow(origin_x, origin_y,
                       x_dir[0] * length, x_dir[1] * length,
                       head_width=0.05, head_length=0.1, fc=color_x, ec=color_x)
    y_arrow = ax.arrow(origin_x, origin_y,
                       y_dir[0] * length, y_dir[1] * length,
                       head_width=0.05, head_length=0.1, fc=color_y, ec=color_y)
    return x_arrow, y_arrow


def init():
    """Initialization for FuncAnimation."""
    map_origin_plot.set_data([0.0], [0.0])  # map origin at (0,0)
    return (
        map_origin_plot, odom_origin_plot, base_origin_plot, camera_origin_plot,
        ball_map_plot, ball_camera_plot
    )


def update(frame_idx):
    global robot_x, robot_y, robot_theta, axis_arrows

    # Clear old arrows
    for art in axis_arrows:
        art.remove()
    axis_arrows = []

    # --------------------------------------------------
    # 1) Random-ish robot motion in MAP frame
    # --------------------------------------------------
    # Small random forward + slight rotation
    v = 0.05  # forward step
    omega = (random.random() - 0.5) * 0.1  # small random turn

    robot_theta += omega
    robot_x += v * math.cos(robot_theta)
    robot_y += v * math.sin(robot_theta)

    # --------------------------------------------------
    # 2) Build transforms for this frame
    # --------------------------------------------------
    # map -> odom (keep identity in this sim)
    # R_map_odom, t_map_odom defined above

    # odom -> base_link (use robot pose in odom frame; here odom == map)
    R_odom_base = rotation_matrix_2d(robot_theta)
    t_odom_base = (robot_x, robot_y)

    # Compose for map -> base_link
    R_map_base, t_map_base = compose_transform_2d(R_map_odom, t_map_odom,
                                                  R_odom_base, t_odom_base)

    # Compose for map -> camera
    R_map_camera, t_map_camera = compose_transform_2d(R_map_base, t_map_base,
                                                      R_base_camera, t_base_camera)

    # --------------------------------------------------
    # 3) Transform ball from camera frame -> map frame
    # --------------------------------------------------
    p_ball_map = transform_point_2d(R_map_camera, t_map_camera, p_ball_camera)

    # Also camera origin in map frame (transform (0,0) from camera to map)
    camera_origin_map = transform_point_2d(R_map_camera, t_map_camera, (0.0, 0.0))

    # base_link origin in map frame = t_map_base
    base_origin_map = t_map_base

    # odom origin in map frame = t_map_odom
    odom_origin_map = t_map_odom

    # --------------------------------------------------
    # 4) Update plots
    # --------------------------------------------------
    # Origins
    map_origin_plot.set_data([0.0], [0.0])
    odom_origin_plot.set_data([odom_origin_map[0]], [odom_origin_map[1]])
    base_origin_plot.set_data([base_origin_map[0]], [base_origin_map[1]])
    camera_origin_plot.set_data([camera_origin_map[0]], [camera_origin_map[1]])

    # Ball
    ball_map_plot.set_data([p_ball_map[0]], [p_ball_map[1]])

    # Draw a short line from camera origin in direction of ball in camera frame (just for visualization)
    # In camera frame, ball vector = p_ball_camera; rotate that into map frame and draw direction
    p_ball_dir_map = transform_point_2d(R_map_camera, (0.0, 0.0), p_ball_camera)
    cam_x, cam_y = camera_origin_map
    ax.plot([cam_x, p_ball_dir_map[0]], [cam_y, p_ball_dir_map[1]], 'y--', linewidth=0.5)

    # Axes for map (identity)
    R_map = rotation_matrix_2d(0.0)
    axis_arrows.extend(draw_frame_axes(0.0, 0.0, R_map, length=0.7, color_x='k', color_y='k'))

    # Axes for odom (same as map in this simple sim)
    axis_arrows.extend(draw_frame_axes(odom_origin_map[0], odom_origin_map[1],
                                       R_map_odom, length=0.5, color_x='b', color_y='b'))

    # Axes for base_link
    axis_arrows.extend(draw_frame_axes(base_origin_map[0], base_origin_map[1],
                                       R_map_base, length=0.5, color_x='r', color_y='r'))

    # Axes for camera
    axis_arrows.extend(draw_frame_axes(camera_origin_map[0], camera_origin_map[1],
                                       R_map_camera, length=0.4, color_x='g', color_y='g'))

    return (
        map_origin_plot, odom_origin_plot, base_origin_plot, camera_origin_plot,
        ball_map_plot
    )


if __name__ == "__main__":
    ax.legend(loc='upper right')
    anim = FuncAnimation(fig, update, init_func=init, frames=STEPS, interval=50, blit=False)
    plt.show()
