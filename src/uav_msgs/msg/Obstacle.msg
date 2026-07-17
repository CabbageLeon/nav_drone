# Obstacle list from perception module — placeholder for future
std_msgs/Header header

# Individual obstacle bounding cylinder
geometry_msgs/Point center      # ENU, center of obstacle
float32 radius                   # bounding cylinder radius [m]
float32 height                   # bounding cylinder height [m]
float32 confidence               # detection confidence [0, 1]
