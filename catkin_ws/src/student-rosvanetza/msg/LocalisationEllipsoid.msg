Header header

# geodetic position (WGS84)
float64 latitude                    # degree (N positive, S negative)
float64 longitude                   # degree (E positive, W negative)

# position confidence ellipse
float32 semi_minor_confidence       # meter
float32 semi_major_confidence       # meter
float32 semi_major_orientation      # degree: north, clockwise

# altitude above WGS84 reference ellipsoid
float32 altitude                    # meter
float32 altitude_confidence
