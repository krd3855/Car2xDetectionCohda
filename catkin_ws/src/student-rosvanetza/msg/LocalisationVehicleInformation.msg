Header header

# data fields are ITS vehicle information (see TS 102 894-2 v1.2.1)
# reported confidence represents 2x sigma, i.e. half width of 95% confidence interval

# horizontal velocity (parallel to ground)
float64 speed                                   # meter per second
float64 speed_confidence

# direction of horizontal velocity
float64 heading                                 # degree: north, clockwise
float64 heading_confidence

# curvature (inverse of radius), positive for left turns
float64 curvature                               # inverse meter
float64 curvature_confidence

