import math

from pyauto import auto

from tobm.sim_models.generic.driver import Driver

l4_core = auto.world.get_ontology(auto.Ontology.L4_Core.value)

with l4_core:

    class TCrossingDriver(Driver):
        """
        Implements some parametrization for the vehicle drivers on the T-crossing to achieve a more realistic behavior.
        """

        _MAX_DISTANCE_FOR_TURNING = 14.5  # m

        def _get_yaw_rate_smoothing_factor(self, degrees_per_second: int | float) -> float:
            return max(0.1, 1 / (1 + math.exp(- 0.1 * (degrees_per_second - 23))))
