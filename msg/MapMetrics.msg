std_msgs/Header header         # header for time and sequence
uint8 map                  # type of submitted map, defined by constants:
# MAP_MULTI=0 # separate maps per robot
# MAP_PARTIAL=1 # maps covering a portion of covered area
# MAP_MERGED=2 # single merged map covering full extent of covered area
float64 coverage  #  inliers compared to ground truth, %
float64 error    # outliers compared to ground truth, %
int64 size      # number of points
float64 rate     # updates per second, Hz
float64 density  # avg points per surface patch for each patch covered

