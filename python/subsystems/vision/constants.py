import math
from typing import Final

from wpimath.units import meters, radians

# Basic filtering thresholds
maxAmbiguity: Final[float] = 0.3
maxZError: Final[float] = 0.75

# Standard deviation baselines, for 1-meter distance and 1 tag
# (Adjusted automatically based on distance and # of tags)
linearStdDevBaseline: Final[meters] = 0.02
angularStdDevBaseline: Final[radians] = 0.06

# Standard deviations multipliers for each camera
# (Adjust to trust some cameras more than others
cameraStdDevFactors: Final[tuple[float, ...]] = (1.0, 0.8, 0.8)

# Multipliers to apply for MegaTag 2 observations
linearStdDevMegatag2Factor: Final[float] = 0.5
angularStdDevMegatag2Factor: Final[float] = math.inf # Ignore because the robot angle is given to LL
