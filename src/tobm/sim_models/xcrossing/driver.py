import math

from shapely.geometry import Point

from pyauto import auto

from tobm.sim_models.generic.driver import Driver

l4_core = auto.world.get_ontology(auto.Ontology.L4_Core.value)

with l4_core:

    class XCrossingDriver(Driver):
        """
        Implements some parametrization for the vehicle drivers on the X-crossing to achieve a more realistic behavior.
        """

        _MAX_DISTANCE_FOR_TURNING = 17  # m

        def _get_yaw_rate_smoothing_factor(self, degrees_per_second: int | float) -> float:
            return max(0.1, 1 / (1 + math.exp(- 0.1 * (degrees_per_second - 20))))

        def _get_yaw_rate_to_reach_point(self, target: Point, delta_t: float | int, max_yaw_rate: float,
                                         distance: float | int = None, ignore_max_distance_for_turning: bool = False):
            # X-crossing driver shall always directly turn on lanes (because they can be very long on the X-crossing),
            # but respect the max. distance for initiating a turn when in Turning mode (for better turning behavior).
            return super()._get_yaw_rate_to_reach_point(target, delta_t, max_yaw_rate, distance,
                                                        self.driving_mode.value != Driver.DrivingMode.Turning.value)
