import logging
from functools import cache
from itertools import chain

from pyauto import auto, extras

from tobm.sim_models.generic.bicyclist import Bicyclist

logger = logging.getLogger(__name__)

l4_de = auto.world.get_ontology(auto.Ontology.L4_DE.value)

with l4_de:
    class TCrossingBicyclist(Bicyclist):
        """
        A bicyclist simulation model specifically for the T-crossing setting. It implements a mode in which it drives
        on the pedestrian crossing if one is near.
        """

        _PEDESTRIAN_CROSSING_TURNING_THRESHOLD = 1.8  # m
        _WALKWAY_TURNING_THRESHOLD = 5  # m
        _DISTANCE_WAYPOINT_FOLLOWING_LANE = 8  # m

        def __init__(self, name=None, namespace=None, is_a=None, *args):
            Bicyclist.__init__(self, name=name, namespace=namespace, is_a=is_a, *args)
            self.crossing_speed = 2.0  # m/s

        def get_target_speed(self) -> float | int:
            s = super().get_target_speed()
            # No matter what was decided before - never drive more than max. crossing speed
            if self.intersects_pedestrian_crossing():
                s = min(self.crossing_speed, s)
                if s == self.crossing_speed:
                    logger.debug("Additional speed mode: reduced due to pedestrian crossing to " + str(s))
            return s

        def intersects_pedestrian_crossing(self):
            intersects_crossing = False
            for i in self.namespace.world.search(
                    type=self.namespace.world.get_ontology(auto.Ontology.L1_DE.value).Pedestrian_Crossing):
                if i.get_geometry().intersects(self.get_geometry()):
                    intersects_crossing = True
                    break
            return intersects_crossing

        def _get_target_follow_lane(self, lane):
            # Ensures the T-crossing bicyclist drives right after leaving the pedestrian crossing
            if str(lane) == "l1_de.walkway4":
                walkway7s = list(self.namespace.world.search(iri="*walkway7"))
                if len(walkway7s) > 0:
                    lane = walkway7s[0]
                    logger.debug("Overriding target from walkway4 to walkway7 to enable right turn")
            return super()._get_target_follow_lane(lane)

        @cache
        def _get_close_pedestrian_crossings(self):
            return list(
                filter(
                    lambda x: x.get_distance(self) <= self._PEDESTRIAN_CROSSING_TURNING_THRESHOLD
                              and extras.utils.in_front_of(x.get_geometry().centroid,
                                                           self.get_geometry().centroid,
                                                           self.has_yaw) and not x.intersects(self),
                    sorted(
                        self.namespace.world.search(type=self.namespace.world.get_ontology(
                            auto.Ontology.L1_DE.value).Pedestrian_Crossing),
                        key=lambda x: x.get_distance(self)
                    )
                )
            )

        @cache
        def _get_close_walkways(self):
            walkways = []
            cur_lane = self.get_current_lane()
            if super()._approaches_end_of_lane(cur_lane):
                walkways = list(filter(lambda x: x != cur_lane and
                                                 x.get_distance(self) <= self._WALKWAY_TURNING_THRESHOLD and
                                                 extras.utils.in_front_of(x.get_geometry().centroid,
                                                                          self.get_geometry().centroid,
                                                                          self.has_yaw, 60),
                                       sorted(self.namespace.world.search(type=self.namespace.world.get_ontology(
                                              auto.Ontology.L1_DE.value).Walkway),
                                              key=lambda x: x.get_distance(self))))
            return walkways

        def _get_relevant_lanes(self):
            world = self.namespace.world
            l1c = world.get_ontology(auto.Ontology.L1_Core.value)
            l1d = world.get_ontology(auto.Ontology.L1_DE.value)
            return chain(world.search(type=l1c.Lane), world.search(type=l1c.Crossing_Site), world.search(type=l1d.Way))

        def _get_next_lane(self, cur_lane):
            # Chooses pedestrian crossings if one is near
            if self.next_lane is None:
                if len(self._get_close_pedestrian_crossings()) > 0:
                    return self._get_close_pedestrian_crossings()[0]
                elif len(self._get_close_walkways()) > 0:
                    return self._get_close_walkways()[0]
            return super()._get_next_lane(cur_lane)

        def _approaches_end_of_lane(self, lane, length=None):
            # Recognizes that being close to a pedestrian crossing is approaching the end of the lane (to be able to
            # switch over to pedestrian crossings). Otherwise, uses the standard implementation.
            if len(self._get_close_pedestrian_crossings()) > 0 or len(self._get_close_walkways()) > 0:
                return True
            else:
                return super()._approaches_end_of_lane(lane)
