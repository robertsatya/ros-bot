define kinect ranger
(
  sensor
  (
    range_max 17.5
    fov 338.0
    samples 1280
  )
  # generic model properties
  color "red"
  size [ 0.06 0.15 0.03 ]
)

# define hokuyo ranger
# (
#   sensor(
#     range [0 4.0]
#     fov 270.25
#     samples 1481
#   )
#   color "black"
#   size [ 0.05 0.05 0.1 ]
#   ctrl "lasernoise"
# )


define turtlebot position
(
  pose [ 0.0 0.0 0.0 0.0 ]

  odom_error [0.01 0.01 0.01 0.02 0.5 1.0]

  size [ 0.2552 0.2552 0.40 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  drive "diff"
  color "gray"

 kinect(pose [ -0.1 0.0 -0.11 0.0 ])
#  hokuyo(pose [0.0 0.000 0 0.000])
)
