import math
from functools import cache
from itertools import chain

import owlready2

from shapely.geometry import Polygon

from pyauto.extras.owl import simulate
from pyauto import auto
from tobm.sim_models.generic.driver import Driver

l4_de = auto.world.get_ontology(auto.Ontology.L4_DE.value)

with l4_de:

    class Bicyclist(Driver):
        """
        Simulation model for a bicyclist. It is mainly based on the Driver simulation model but sets different target
        speeds and lanes it may want to reach.
        """

        _MAX_PET_FOR_BRAKING_WHEN_INTERSECTING = 3  # s
        _DISTANCE_WAYPOINT_TURNING = 0.4  # m, overrides generic driver parameter

        def __init__(self, name=None, namespace=None, is_a=None, *args):
            Driver.__init__(self, name=None, namespace=None, is_a=None, *args)
            self.default_target_speed = 4.5  # m/s
            self.crossing_speed = 3.5  # m/s
            self.crossing_relevant_distance = 12  # m

        @cache
        def get_current_lane(self):
            if len(self.drives) > 0:
                geo = self.drives[0].get_geometry()
            else:
                geo = self.get_geometry()

            cur_lanes = []
            largest = 0
            world = self.namespace.world
            l1co = world.get_ontology(auto.Ontology.L1_Core.value)
            l1de = world.get_ontology(auto.Ontology.L1_DE.value)
            for lane in self._get_relevant_lanes():
                area = geo.intersection(lane.get_geometry()).area
                if area > largest:
                    cur_lanes = [lane]
                    largest = area
                elif area > 0 and math.isclose(area, largest):
                    cur_lanes.append(lane)

            smallest = None
            smallest_area = 0
            for lane in cur_lanes:
                lane_area = lane.get_geometry().area
                if smallest is None or lane_area < smallest_area:
                    smallest = lane
                    smallest_area = lane_area

            return smallest

        def _get_relevant_lanes(self):
            world = self.namespace.world
            l1c = world.get_ontology(auto.Ontology.L1_Core.value)
            l1d = world.get_ontology(auto.Ontology.L1_DE.value)
            res = []
            lanes = chain(world.search(type=l1c.Lane), world.search(type=l1d.Way))
            invalid_lane_types = {l1d.Pedestrian_Ford, l1d.Pedestrian_Crossing, l1c.Non_Driveable_Lane}
            for lane in lanes:
                valid = True
                for invalid_lane_type in invalid_lane_types:
                    if isinstance(lane, invalid_lane_type):
                        valid = False
                        break
                if valid:
                    res.append(lane)
            return res

        def _get_target_end_of_lane(self, lane):
            end = lane.get_end(self.has_yaw, *self.get_geometry().centroid.coords, length=0.3).centroid
            return end.x, end.y

        def _get_yaw_rate_smoothing_factor(self, degrees_per_second: int | float) -> float:
            return max(0.2, 1 / (1 + math.exp(- 0.15 * (degrees_per_second - 10))))
