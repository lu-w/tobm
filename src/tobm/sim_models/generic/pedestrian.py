import math
import logging

import owlready2
import sympy

from enum import Enum
from functools import cache

from shapely.geometry import Polygon, Point, LineString

from pyauto.extras.owl import simulate
from pyauto import auto

from tobm.sim_models.generic.dynamical_object import Dynamical_Object

logger = logging.getLogger(__name__)

l4_core = auto.world.get_ontology(auto.Ontology.L4_Core.value)

with (l4_core):

    class Pedestrian(Dynamical_Object):
        """
        Implements a simulation model for a pedestrian using the following behavior:
        - If pedestrian has not finished crossing (i.e., is in crossing road mode and does not intersect target):
              stays in crossing road mode
        - Else if pedestrian can cross a road (i.e., is close enough to some driveable lane and random successful):
              selects a target & goes into crossing road mode
        - Else if pedestrian intersects some walkway:
              goes into walkway walking mode
        - Else:
              goes into unrestricted walking mode
        - If there is some object the pedestrian has to yield to, it goes into yielding mode until it has passed
        """

        _DISTANCE_WAYPOINT_FOLLOWING_WALKWAY = 4  # m
        _CROSSING_ROAD_THRESHOLD = 2  # m
        _CROSSING_WALKWAY_TARGET_SIZE = 0.5  # m
        _END_OF_WALKWAY_LENGTH = 1 # m
        _TARGET_RANDOMNESS_STDEV = 0.1  # m
        _MAX_DISTANCE_FOR_TURNING = 100 # m
        _CLOSE_TO_CROSSING_THRESHOLD = 5  # m
        _MAX_DISTANCE_FOR_YIELDING_MODE = 10  # s
        _MAX_PET_FOR_YIELDING = 4  # s
        _CROSSING_ROAD_PROBABILITY = 7e-3  # probability
        _MINIMUM_WALKWAY_AREA_FOR_WALKING = 8 # m
        # Pedestrians yield to individuals with those (indirect) types that have height > 0
        _YIELD_FOR_TYPES = {"l4_core.Pedestrian", "l4_de.Bicycle", "l1_core.L1_Entity", "l1_core.L2_Entity"}
        _YIELD_FOR_NONMOVING_TYPES = {"l4_core.Vehicle"}

        class WalkingMode(Enum):
            UnrestrictedWalking = 0
            WalkwayWalking = 1
            CrossingRoad = 2
            Stopping = 3

        class SpeedMode(Enum):
            UnrestrictedWalking = 0
            Stopping = 1
            Yielding = 2

        ########################################################
        # Overwritten simulation methods from Dynamical_Object #
        ########################################################

        def __init__(self, name=None, namespace=None, is_a=None, *args):
            Dynamical_Object.__init__(self, name=None, namespace=None, is_a=None, *args)
            self.walking_mode = Pedestrian.WalkingMode.UnrestrictedWalking
            self.speed_mode = Pedestrian.SpeedMode.UnrestrictedWalking
            self.selected_crossing_target = None
            self.target_end_of_walkway = None
            self.currently_crossing_lane = False
            self.last_walkway = None

        def get_target(self) -> tuple:
            """
            Overwrites the dynamical object's simulation method to deliver a pedestrian-specific target point.
            :return: A triple of a target point and the distance to this point (to increase performance).
            """
            new_self = self.mapping[self]
            self.update_state(new_self)

            distance = None

            if new_self.walking_mode.value == Pedestrian.WalkingMode.UnrestrictedWalking.value:
                walkway = self._get_closest_walkable_walkway()
                if walkway:
                    logger.debug("Target mode: walking back to walkable walkway")
                    t_x, t_y = self._get_target_walking_back(walkway.get_geometry())
                else:
                    logger.debug("Target mode: unrestricted walking")
                    t_x, t_y, distance = Dynamical_Object.get_target(self)
                t_x, t_y = self._add_randomness_to_target(t_x, t_y)
            elif new_self.walking_mode.value == Pedestrian.WalkingMode.WalkwayWalking.value:
                logger.debug("Target mode: walkway walking")
                t_x, t_y = self._add_randomness_to_target(
                    *self.get_target_following_polygon(self._get_intersecting_walkway().get_geometry(),
                                                       target_distance=Pedestrian._DISTANCE_WAYPOINT_FOLLOWING_WALKWAY)
                )
            elif new_self.walking_mode.value == Pedestrian.WalkingMode.Stopping.value:
                logger.debug("Target mode: stopping at end of walkway")
                t_x, t_y = self._get_target_end_of_walkway()
            elif new_self.walking_mode.value == Pedestrian.WalkingMode.CrossingRoad.value:
                logger.debug("Target mode: crossing road")
                t = self._get_crossing_target()
                t_x = t.centroid.x
                t_y = t.centroid.y
            else:
                raise Exception(str(self) + " is in undefined walking mode.")

            if distance is None:
                distance = self.get_geometry().distance(Point(t_x, t_y))
            # Yielding overwrites distance
            if new_self.speed_mode.value == Pedestrian.SpeedMode.Yielding.value:
                logger.debug("Target mode overwrite: yielding")
                distance = min(distance, self.get_geometry().distance(Point(self._get_target_yielding())))

            logger.debug("Target is " + str(t_x) + ", " + str(t_y) + " (distance : " + str(distance) + ")")

            return t_x, t_y, distance

        def get_target_speed(self) -> float | int:
            """
            Overwrites the dynamical object's simulation method to deliver a pedestrian-specific target speed.
            :return: The target speed.
            """
            new_self = self.mapping[self]
            if new_self.speed_mode.value == Pedestrian.SpeedMode.Stopping.value:
                speed = 0
                logger.debug("Speed mode: 0 m/s due to stopping at end of walkway")
            elif new_self.speed_mode.value == Pedestrian.SpeedMode.Yielding.value:
                speed = 0
                logger.debug("Speed mode: 0 m/s due to yielding")
            else:
                speed = Dynamical_Object.get_target_speed(self, warning=False)
                logger.debug("Speed mode: default mode of " + str(speed) + " m/s")
            return speed

        ##################
        # Helper methods #
        ##################

        def update_state(self, new_self):
            """
            Updates state of `new_self` based on the state of `self`.
            :param new_self: The representation of self at the next simulation step (will be modified in-place).
            """

            if self.walking_mode.value == Pedestrian.WalkingMode.CrossingRoad.value and \
                    not self.get_geometry().intersects(self.selected_crossing_target):
                new_self.walking_mode = Pedestrian.WalkingMode.CrossingRoad
                new_self.selected_crossing_target = self._get_crossing_target()
                logger.debug("Pedestrian mode: Deciding to keep crossing road to reach " +
                             str(new_self.selected_crossing_target.centroid) + " +- " +
                             str(Pedestrian._CROSSING_WALKWAY_TARGET_SIZE))
            elif self._close_to_some_driveable_lane() and \
                    self.namespace.world._random.random() < Pedestrian._CROSSING_ROAD_PROBABILITY and \
                    self.walking_mode.value not in {Pedestrian.WalkingMode.UnrestrictedWalking.value,
                                            Pedestrian.WalkingMode.Stopping.value} and \
                    self._get_intersecting_road() is not None and \
                    self._get_furthest_walkway_of_current_road() is not None:
                logger.debug("Pedestrian mode: Deciding to cross road")
                new_self.walking_mode = Pedestrian.WalkingMode.CrossingRoad
                new_self.selected_crossing_target = None
                new_self.selected_crossing_target = self._get_crossing_target()
            elif self._get_intersecting_walkway() is not None and not self._is_close_to_crossing() and \
                    self._has_reached_end_of_walkway(self._END_OF_WALKWAY_LENGTH * 3):
                new_self.target_end_of_walkway = self._get_target_end_of_walkway()
                new_self.walking_mode = Pedestrian.WalkingMode.Stopping
                new_self.speed_mode = Pedestrian.SpeedMode.Stopping
                logger.debug("Pedestrian mode: Stopping at end of walkway " + str(self._get_intersecting_walkway()))
            elif self._get_intersecting_walkway() is not None and \
                    self._get_intersecting_walkway().get_geometry().area > self._MINIMUM_WALKWAY_AREA_FOR_WALKING:
                new_self.walking_mode = Pedestrian.WalkingMode.WalkwayWalking
                new_self.last_walkway = self._get_intersecting_walkway()
                logger.debug("Pedestrian mode: Deciding to follow current walkway " +
                             str(self._get_intersecting_walkway()))
            else:
                logger.debug("Pedestrian mode: Unrestricted walking")
                new_self.walking_mode = Pedestrian.WalkingMode.UnrestrictedWalking

            if self._get_yielding_intersection_point():
                new_self.speed_mode = Pedestrian.SpeedMode.Yielding

            if new_self.walking_mode.value != Pedestrian.WalkingMode.Stopping:
                new_self.target_end_of_walkway = None

            if new_self.last_walkway is None:
                new_self.last_walkway = self.last_walkway
            logger.debug("Last walkway = " + str(new_self.last_walkway))

        def _add_randomness_to_target(self, t_x, t_y):
            added_t_x = self.namespace.world._np_random.normal(0, scale=self._TARGET_RANDOMNESS_STDEV)
            added_t_y = self.namespace.world._np_random.normal(0, scale=self._TARGET_RANDOMNESS_STDEV)
            return t_x + added_t_x, t_y + added_t_y

        def _close_to_some_driveable_lane(self):
            for i in self.namespace.world.search(
                    type=self.namespace.world.get_ontology(auto.Ontology.L1_Core.value).Driveable_Lane):
                if self.get_distance(i) < self._CROSSING_ROAD_THRESHOLD:
                    return True
            return False

        def _get_crossing_target(self):
            def get_target(k):
                    p_yaw_k = (geom_self.x + math.cos(math.radians(self.has_yaw + 80 + k)),
                                geom_self.y + math.sin(math.radians(self.has_yaw + 80 - k)))
                    line = sympy.Line(*geom_self.coords, p_yaw_k)
                    poly = sympy.Polygon(*walkway.get_geometry().exterior.coords)
                    target = LineString(line.intersection(poly)).\
                        centroid.buffer(self._CROSSING_WALKWAY_TARGET_SIZE)
                    return target

            if self.selected_crossing_target is None or self.get_geometry().intersects(self.selected_crossing_target):
                walkway = self._get_furthest_walkway_of_current_road()
                geom_self = self.get_geometry().centroid
                k = 0
                while self.selected_crossing_target is None and k < 90:
                    target = get_target(k)
                    k += 10
                    if not target.is_empty:
                        self.selected_crossing_target = target
                if self.selected_crossing_target is None or self.selected_crossing_target.is_empty:
                    logger.debug("WARNING: using default walkway center for computing crossing target")
                    self.selected_crossing_target = walkway.get_geometry().centroid
                logger.debug("  Selected target " + str(self.selected_crossing_target.centroid) +
                             " for crossing to walkway " + str(walkway))
            return self.selected_crossing_target

        def _get_target_yielding(self):
            p = self._get_yielding_intersection_point()
            if p is None:
                p = self.get_geometry().centroid
                logger.warning("Could not determine yielding target, using default point instead.")
            return p

        @cache
        def _get_furthest_walkway_of_current_road(self):
            walkway = None
            furthest_distance = None
            for i in self._get_intersecting_road().has_lane:
                if not self.intersects(i) and \
                        self.namespace.world.get_ontology(auto.Ontology.L1_DE.value).Walkway in i.is_a:
                    distance = i.get_distance(self)
                    if walkway is None or distance > furthest_distance:
                        walkway = i
                        furthest_distance = distance
            return walkway

        @cache
        def _get_intersecting_road(self):
            for i in self.namespace.world.search(
                    type=self.namespace.world.get_ontology(auto.Ontology.L1_Core.value).Road):
                if i.intersects(self):
                    return i
            return None

        @cache
        def _get_intersecting_walkway(self):
            for i in self.namespace.world.search(
                    type=self.namespace.world.get_ontology(auto.Ontology.L1_DE.value).Walkway):
                if i.intersects(self):
                    return i
            return None

        def _is_close_to_crossing(self):
            for i in self.namespace.world.search(
                    type=self.namespace.world.get_ontology(auto.Ontology.L1_DE.value).Crossing):
                if i.get_distance(self) <= self._CLOSE_TO_CROSSING_THRESHOLD:
                    return True
            return False

        def _get_closest_walkable_walkway(self):
            closest_walkway = None
            closest_walkway_dist = None
            for w in self.namespace.world.search(
                    type=self.namespace.world.get_ontology(auto.Ontology.L1_DE.value).Walkway):
                if str(w) != str(self.last_walkway):
                    dist = w.get_distance(self)
                    if w.get_geometry().area >= self._MINIMUM_WALKWAY_AREA_FOR_WALKING and \
                            (closest_walkway_dist is None or dist < closest_walkway_dist):
                        closest_walkway = w
                        closest_walkway_dist = dist
            logger.debug("Walking to closest walkway: " + str(closest_walkway))
            return closest_walkway

        def _has_reached_end_of_walkway(self, length=None):
            end = self._get_end_of_walkway(length)
            reached_end = False
            if end is not None:
                reached_end = self.get_geometry().intersects(end)
            return reached_end

        def _get_end_of_walkway(self, length=None):
            walkway = self._get_intersecting_walkway()
            end = None
            if walkway is not None:
                if length is None:
                    length = self._END_OF_WALKWAY_LENGTH
                end = walkway.get_end(self.has_yaw, *self.get_geometry().centroid.coords, length=length)
            return end

        def _get_target_end_of_walkway(self):
            if self.target_end_of_walkway is None:
                length = self._END_OF_WALKWAY_LENGTH
                end = self._get_end_of_walkway(length).centroid.coords[0]
                target_end_of_walkway = self._add_randomness_to_target(*end)
                while not self.is_target_free(target_end_of_walkway):
                    logger.debug("Pedestrian target " + str(target_end_of_walkway) + " is not free, moving back.")
                    length += self._END_OF_WALKWAY_LENGTH
                    end = self._get_end_of_walkway(length).centroid.coords[0]
                    target_end_of_walkway = self._add_randomness_to_target(*end)
                self.target_end_of_walkway = target_end_of_walkway
            return self.target_end_of_walkway

        def _get_target_walking_back(self, geom: Polygon):
            walkway_points = sorted(set(geom.exterior.coords), key=lambda p: self.get_geometry().distance(Point(p)))
            return LineString([walkway_points[0], walkway_points[1]]).centroid.coords[0]

        @cache
        def _get_yielding_intersection_point(self):
            int_objs = sorted(self.get_intersecting_objects(
                horizon=self._MAX_DISTANCE_FOR_YIELDING_MODE, delta_t=0.25),
                key=lambda x: self.get_geometry().distance(x[3])
            )
            for obj, t_self, t_other, p_int in int_objs:
                if t_self + t_other <= self._MAX_DISTANCE_FOR_YIELDING_MODE and (
                        (l4_core.Pedestrian not in obj.is_a and
                         0 <= t_self - t_other <= self._MAX_PET_FOR_YIELDING) or
                        (len(self._YIELD_FOR_NONMOVING_TYPES.intersection(set([str(x) for x in obj.INDIRECT_is_a]))) > 0
                         and obj.has_speed is None or obj.has_speed == 0) or
                        (len(self._YIELD_FOR_TYPES.intersection(set([str(x) for x in obj.INDIRECT_is_a]))) > 0
                         and abs(t_self - t_other) <= self._MAX_PET_FOR_YIELDING)):
                    logger.debug("  -> yielding to " + str(obj) + " with predicted int. point " + str(p_int))
                    return p_int
            return None
