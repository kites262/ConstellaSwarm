from typing import Annotated

import numpy as np
from numpy.typing import NDArray

Vec3 = Annotated[NDArray[np.float64], 3]
