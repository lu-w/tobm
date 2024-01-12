import math
import logging

import owlready2

from enum import Enum
from functools import cache

import sympy
from shapely.geometry import Polygon, Point, LineString
from shapely.affinity import translate

from pyauto import auto, extras

from tobm.sim_models.generic.dynamical_object import Dynamical_Object

logger = logging.getLogger(__name__)

l4_core = auto.world.get_ontology(auto.Ontology.L4_Core.value)

with l4_core:
    class Driver(Dynamical_Object):
        """
        Main simulation model of TOBM for anything that drives a vehicle.
        We have a state machine with 5 modes:
        - Lane following: If we are on a lane, the next waypoint is chosen according to the center of that lane in
            front of the vehicle. If the end of a lane is reached, and the driver can turn, it randomly decides on a
            successor lane and switches to Turning mode. If it can not turn, it goes into the DeadEnd mode.
        - Turning: Keeps turning with the same waypoint at the beginning of the new lane until the new lane is reached.
        - DeadEnd: Just gracefully stops at the end of the lane.
        - Roaming: If the driver is currently on no lane at all, it just drives straight.
        - Undefined: This mode is a fallback and shall not be reached. An exception is thrown.
        The driver selects the speed based on the target mode. Speed is also implemented as a state machine:
        - Unrestricted: Just chose the maximum possible speed.
        - Turning: A speed suitable for turning is selected.
        - Following: Implements a following mode, i.e., adapts the speed to the object in front.
        - Yielding: If an intersecting path is predicted and the driver is later at the intersection point than the
            intersecting object, then speed is reduced to yield.
        - Stopping: For stopping at the end of lanes.
        - Undefined: This mode is a fallback and shall not be reached. An exception is thrown.
        """

        class DrivingMode(Enum):
            LaneFollowing = 0
            Turning = 1
            DeadEnd = 2
            Roaming = 3
            Undefined = 4

        class SpeedMode(Enum):
            Unrestricted = 0
            Turning = 1
            Following = 2
            Yielding = 3
            Stopping = 4
            Undefined = 5

        _DISTANCE_WAYPOINT_FOLLOWING_LANE = 4  # m
        _DISTANCE_WAYPOINT_ROAMING = 5  # m
        _DISTANCE_WAYPOINT_TURNING = 0.1  # m
        _MAX_DISTANCE_FOR_YIELDING_MODE = 15  # s
        _MAX_PET_FOR_BRAKING_WHEN_INTERSECTING = 6  # s
        _MAX_THW_FOR_BRAKING_WHEN_FOLLOWING = 2  # s
        _MAX_DELTA_T_FOR_LEAD_VEHICLES = 10  # s
        _PROBABILITY_NO_SIGNAL_DURING_TURN = 0.97  # probability

        #######################################################
        # Overwritten simulation methods from Dynamica_Object #
        #######################################################

        def __init__(self, name=None, namespace=None, is_a=None, *args):
            Dynamical_Object.__init__(self, name=None, namespace=None, is_a=None, *args)

            self.driving_mode = Driver.DrivingMode.Undefined
            self.speed_mode = Driver.SpeedMode.Undefined
            self.next_lane = None
            self.lead_vehicle = None
            self.intersecting_object = None
            self.tti_object = None
            self.tti_self = None
            self.p_int = None

            # Values can also be used to define driving profiles
            self.default_target_speed = 10  # m/s
            self.crossing_speed = 1.5  # m/s
            self.turning_speed = 1.25  # m/s
            self.crossing_relevant_distance = 15  # m

        def get_target(self) -> tuple:
            """
            Overwrites the dynamical object's simulation method to deliver a driver-specific target point.
            :return: A triple of a target point and the distance to this point (to increase performance).
            """
            new_self = self.mapping[self]
            cur_lane = self.get_current_lane()
            self.update_state(cur_lane, new_self)
            logger.debug("Currently on lane " + str(cur_lane))

            if not self._intersects_some_road_infrastructure():
                t_x, t_y = self._get_target_no_road()
                logger.debug("Target mode: no road")
            elif new_self.driving_mode.value == Driver.DrivingMode.LaneFollowing.value:
                logger.debug("Target mode: lane following")
                t_x, t_y = self._get_target_follow_lane(cur_lane)
            elif new_self.driving_mode.value == Driver.DrivingMode.Turning.value:
                logger.debug("Target mode: targeting successor lane")
                t_x, t_y = self._get_target_in_next_lane(cur_lane)
                self.set_turn_signal(cur_lane)
            elif new_self.driving_mode.value == Driver.DrivingMode.DeadEnd.value:
                logger.debug("Target mode: dead end")
                t_x, t_y = self._get_target_end_of_lane(cur_lane)
            elif new_self.driving_mode.value == Driver.DrivingMode.Roaming.value:
                logger.debug("Target mode: roaming")
                t_x, t_y = self._get_target_roaming()
            else:
                raise Exception(str(self) + " is in undefined driving mode.")

            # Special case: Yielding overwrites distance to target if distance is less than previously computed target
            if len(self.drives) > 0:
                self_geom = self.drives[0].get_geometry()
            else:
                self_geom = self.get_geometry()
            distance = self_geom.distance(Point(t_x, t_y))
            if new_self.speed_mode.value == Driver.SpeedMode.Yielding.value:
                logger.debug("Target mode overwrite: yielding")
                distance = min(distance, self_geom.distance(Point(self._get_target_yielding(t_x, t_y))))
            # Special case: Vehicle following overwrites target if distance is less than previously computed target
            elif new_self.speed_mode.value == Driver.SpeedMode.Following.value:
                following_point = Point(self._get_target_follow_vehicle())
                logger.debug("Following point is: " + str(following_point))
                following_distance = self_geom.distance(following_point)
                if following_distance < distance:
                    logger.debug("Target mode overwrite: vehicle following")
                    t_x, t_y = following_point.coords[0]
                    distance = following_distance

            logger.debug("Target is " + str(t_x) + ", " + str(t_y) + " (distance: " + str(distance) + ")")
            return t_x, t_y, distance

        def get_target_speed(self) -> float | int:
            """
            Overwrites the dynamical object's simulation method to deliver a driver-specific target speed.
            :return: The target speed.
            """
            new_self = self.mapping[self]

            if new_self.speed_mode.value == Driver.SpeedMode.Unrestricted.value:
                s = self.get_default_speed()
                logger.debug("Speed mode: default speed " + str(s) + " m/s")
            elif new_self.speed_mode.value == Driver.SpeedMode.Following.value:
                lead_veh = self.get_lead_vehicle()
                if lead_veh is not None:
                    if lead_veh.has_speed and self.has_speed and \
                            self.get_distance(lead_veh) / self.has_speed < self._MAX_THW_FOR_BRAKING_WHEN_FOLLOWING:
                        s = lead_veh.has_speed * 0.75
                    else:
                        s = lead_veh.has_speed
                else:
                    s = self.get_default_speed()
                    logger.debug("WARNING: Vehicle following mode but defaulting to normal speed")
                logger.debug("Speed mode: vehicle following " + str(s) + " m/s")
            elif new_self.speed_mode.value == Driver.SpeedMode.Yielding.value:
                int_obj, tti_obj, tti_self, _ = self.get_intersecting_object()
                s = 0
                logger.debug("Speed mode: yielding to " + str(int_obj) + " with PET = " + str(tti_self - tti_obj) +
                             " in " + str(tti_self) + " s -> " + str(s) + " m/s")
            elif new_self.speed_mode.value == Driver.SpeedMode.Stopping.value:
                s = 0
                logger.debug("Speed mode: stopping, slowing down to 0 m/s")
            else:
                raise Exception(str(self) + " is in undefined speed mode.")

            # No matter what was decided before - never drive more than max. crossing speed
            if self.close_to_crossing():
                s = min(self.crossing_speed, s)
                if s == self.crossing_speed:
                    logger.debug("Additional speed mode: reduced due to crossing to " + str(s))

            return s

        ##################
        # Helper methods #
        ##################

        def update_state(self, cur_lane, new_self):
            """
            Updates the state machine of the driver, for both target and speed mode as described in the class interface.
            :param cur_lane: The current lane the driver is on (to increase performance).
            :param new_self: The individual in the new scene that is currently simulated. This is the object into which
                the new target and speed modes are written to.
            """
            # DRIVING MODE
            if self.driving_mode.value == Driver.DrivingMode.Turning.value:
                # Check if we finished trning (we require >40% of vehicle on lane to say that we finished turning)
                veh_geom = self.drives[0].get_geometry()
                if self.next_lane is not None and \
                        veh_geom.intersection(self.next_lane.get_geometry()).area / veh_geom.area > 0.4:
                    new_self.next_lane = None
                    new_self.driving_mode = Driver.DrivingMode.LaneFollowing
                else:
                    # Otherwise, 'remember' that we are in turning mode and the previously selected next lane
                    new_self.next_lane = self.next_lane
                    new_self.driving_mode = Driver.DrivingMode.Turning
            elif cur_lane is None:
                new_self.driving_mode = Driver.DrivingMode.Roaming
            elif self.driving_mode.value in {Driver.DrivingMode.LaneFollowing.value, Driver.DrivingMode.DeadEnd.value}:
                eol = self._approaches_end_of_lane(cur_lane)
                if eol:
                    next_lane = self._get_next_lane(cur_lane)
                    if next_lane is not None:
                        self.next_lane = next_lane
                        new_self.next_lane = self.next_lane
                        new_self.driving_mode = Driver.DrivingMode.Turning
                    else:
                        new_self.driving_mode = Driver.DrivingMode.DeadEnd
                else:
                    new_self.driving_mode = Driver.DrivingMode.LaneFollowing
            else:
                new_self.driving_mode = Driver.DrivingMode.LaneFollowing

            # SPEED MODE
            int_obj, tti_obj, tti_self, p_int = self.get_intersecting_object()
            # Propagate intersecting object to new self if this is still valid
            if int_obj in [x[0] for x in
                           self.get_intersecting_objects(horizon=self._MAX_DISTANCE_FOR_YIELDING_MODE, delta_t=0.25)]:
                new_self.intersecting_object = int_obj
                new_self.tti_object = tti_obj
                new_self.tti_self = tti_self
                new_self.p_int = p_int
            # If x is not anymore intersecting, we can move on
            else:
                new_self.intersecting_object = None
                new_self.tti_object = None
                new_self.tti_self = None
                new_self.p_int = None

            yielding = int_obj is not None # TODO append "and int_obj.get_speed() > 0" to avoid "sliding situations"?
            following = self.get_lead_vehicle() is not None
            stopping = new_self.driving_mode.value == Driver.DrivingMode.DeadEnd.value or \
                       not self._intersects_some_road_infrastructure()
            if following:
                new_self.speed_mode = Driver.SpeedMode.Following
            elif yielding:
                logger.debug("yielding to " + str(int_obj) + " with PET = " + str(tti_self - tti_obj) +
                             " and TTI = " + str(tti_self))
                new_self.speed_mode = Driver.SpeedMode.Yielding
            elif stopping:
                new_self.speed_mode = Driver.SpeedMode.Stopping
            else:
                new_self.speed_mode = Driver.SpeedMode.Unrestricted

        def close_to_crossing(self):
            crossing_close = False
            for i in self.namespace.world.search(
                    type=self.namespace.world.get_ontology(auto.Ontology.L1_Core.value).Junction):
                if self.get_distance(i) < self.crossing_relevant_distance and i.in_front_of(self):
                    crossing_close = True
                    break
            return crossing_close

        def get_default_speed(self):
            if self.driving_mode.value == Driver.DrivingMode.Turning.value:
                s = self.turning_speed
            else:
                s = self.get_maximum_speed()
            if s is not None:
                s = min(s, self.default_target_speed)
            else:
                s = self.default_target_speed
            return s

        @cache
        def get_lead_vehicle(self):
            for obj in self.namespace.world.search(
                    type=self.namespace.world.get_ontology(auto.Ontology.Physics.value).Dynamical_Object):
                if obj.has_geometry() and self.get_current_lane() is not None and \
                        self.get_current_lane().has_geometry() and obj.has_yaw is not None and \
                        self.has_yaw is not None and obj.get_geometry().area > 0 and \
                        (obj.get_geometry().intersection(self.get_current_lane().get_geometry()).area / \
                         obj.get_geometry().area > 0.5) and \
                        obj.in_front_of(self) and -70 < self.has_yaw - obj.has_yaw < 70 and \
                        (self.lead_vehicle is None or
                         self.drives[0].get_distance(obj) < self.drives[0].get_distance(self.lead_vehicle)) and \
                        self.drives[0].get_distance(obj) < max(2, self._MAX_DELTA_T_FOR_LEAD_VEHICLES * self.has_speed):
                    self.lead_vehicle = obj
            return self.lead_vehicle

        def get_intersecting_object(self):
            if self.intersecting_object is None and len(self.drives) > 0:
                for obj, t_self, t_other, p_int in \
                        self.get_intersecting_objects(horizon=self._MAX_DISTANCE_FOR_YIELDING_MODE, delta_t=0.25):
                        if t_self is not None:
                            logger.debug(str(self) + " t = " + str(t_self) + " intersects with " + str(obj) + " t = " +
                                         str(t_other) + " @ " + str(p_int))
                        if t_self is not None and t_other is not None and \
                                 t_self + t_other <= self._MAX_DISTANCE_FOR_YIELDING_MODE and \
                                 (0 <= t_self - t_other <= self._MAX_PET_FOR_BRAKING_WHEN_INTERSECTING) and \
                                 (self.tti_self is None or t_self + t_other < self.tti_self + self.tti_object) and \
                                 extras.utils.in_front_of(p_int, self.get_centroid(), self.has_yaw):
                            logger.debug("  -> chose to yield to " + str(obj))
                            self.intersecting_object = obj
                            self.tti_object = t_other
                            self.tti_self = t_self
                            self.p_int = p_int
            return self.intersecting_object, self.tti_object, self.tti_self, self.p_int

        @cache
        def get_current_lane(self):
            if len(self.drives) > 0:
                geo = self.drives[0].get_geometry()
            else:
                geo = self.get_geometry()

            cur_lane = None
            largest = 0

            for lane in self.namespace.world.search(
                    type=self.namespace.world.get_ontology(auto.Ontology.L1_Core.value).Driveable_Lane):
                geo_l = lane.get_geometry()
                ins = geo.intersection(geo_l)
                if ins.area > largest or (largest == 0 and geo.intersection(geo_l).area > 0.5):
                    largest = ins.area
                    cur_lane = lane

            if cur_lane is not None:
                _ = getattr(cur_lane, "has_successor_lane")  # required due to caching (forces property to load)
            return cur_lane

        def set_turn_signal(self, cur_lane):
            l6_de = self.namespace.world.ontology(auto.Ontology.L6_DE)
            com = self.namespace.world.ontology(auto.Ontology.Communication)
            next_lane = self._get_next_lane(cur_lane)
            if self.namespace.world._random.random() < self._PROBABILITY_NO_SIGNAL_DURING_TURN:
                self.drives[0].is_a.append(owlready2.Not(com.delivers_signal.some(l6_de.Turn_Signal)))
            else:
                if str(cur_lane) in [str(l) for l in next_lane.is_lane_left_of]:
                    signal = l6_de.Left_Turn_Signal()
                    self.drives[0].delivers_signal.append(signal)
                elif str(cur_lane) in [str(l) for l in next_lane.is_lane_right_of]:
                    signal = l6_de.Right_Turn_Signal()
                    self.drives[0].delivers_signal.append(signal)

        def _intersects_some_road_infrastructure(self):
            intersects = False
            for infrastructure in self.namespace.world.search(
                    type=self.namespace.world.get_ontology(auto.Ontology.L1_Core.value).L1_Entity):
                if self.intersects(infrastructure):
                    intersects = True
                    break
            return intersects

        def _approaches_end_of_lane(self, lane, length=None):
            if len(self.drives) > 0 and self.drives[0] is not None:
                geo = self.drives[0].get_geometry()
                geo_c = self.drives[0].get_centroid()
                geo_length = self.drives[0].has_length
                self_end = self.drives[0].get_end((self.has_yaw + 180) % 360, *geo_c.coords, length=0.3).centroid
                # Length of vehicle may be to little to find end by just looking in 100 or 110° field of view. Moves
                # reference point back a bit (1.5 meters) for those vehicles.
                if geo_length <= 1.5:
                    self_end = Point(self_end.x + 1.5 * math.cos(math.radians((self.has_yaw + 180) % 360)),
                                     self_end.y + 1.5 * math.sin(math.radians((self.has_yaw + 180) % 360)))
            else:
                geo = self.get_geometry()
                geo_length = self.has_length
                self_end = self.get_centroid()
            if length is None:
                if len(lane.has_successor_lane) == 0 or geo_length <= 1.5:
                    length = max(1, (geo_length or 0) * 0.75 * self.has_speed)
                else:
                    length = 0.3
            end = lane.get_end(self.has_yaw, *self_end.coords, length=length)
            at_end_of_lane = geo.intersects(end)
            is_leaving_lane = True
            if len(lane.has_successor_lane) > 0:
                lane_geo = lane.get_geometry()
                if lane_geo.area > 6:
                    is_leaving_lane = lane_geo.intersection(geo).area / geo.area < 0.5
                else:
                    # small lanes can be left directly
                    is_leaving_lane = True
            return at_end_of_lane and is_leaving_lane

        @cache
        def _get_next_lane(self, cur_lane):
            if self.next_lane is None and len(cur_lane.has_successor_lane) > 0:
                # right now, no weights when chossing a successor lane
                lanes = cur_lane.has_successor_lane
                suc = self.namespace.world._random.choice(sorted(lanes, key=str))
            elif self.next_lane is not None:
                suc = self.next_lane
            else:
                suc = None
            return suc

        def _get_target_in_next_lane(self, cur_lane):
            suc = self._get_next_lane(cur_lane)
            logger.debug("-> next lane is " + str(suc))
            geo_l = suc.get_geometry()
            left, right, front, back = extras.utils.split_polygon_into_boundaries(geo_l)
            geo = self.get_geometry()
            if front.distance(geo) > back.distance(geo):
                beg = back
                if len(left.coords) > 1 and len(right.coords) > 1:
                    p_next_l = left.coords[1]
                    p_next_r = right.coords[1]
            else:
                beg = front
                if len(left.coords) > 1 and len(right.coords) > 1:
                    p_next_l = left.coords[-2]
                    p_next_r = right.coords[-2]
            if len(left.coords) > 1 and len(right.coords) > 1:
                beg = translate(beg, xoff=0.05 * (p_next_l[0] + p_next_r[0]) / 2,
                                     yoff=0.05 * (p_next_l[1] + p_next_r[1]) / 2)
            centroid = beg.centroid.buffer(self._DISTANCE_WAYPOINT_TURNING).intersection(geo_l).centroid
            if centroid.is_empty:
                logger.warning("Empty intersection between estimated beginning of next lane and next lane.")
                return beg.centroid.x, beg.centroid.y
            else:
                return centroid.x, centroid.y

        def _get_target_follow_lane(self, lane):
            return self.get_target_following_polygon(lane.get_geometry(),
                                                     target_distance=self._DISTANCE_WAYPOINT_FOLLOWING_LANE)

        def _get_target_end_of_lane(self, lane):
            end = lane.get_end(self.has_yaw, *self.get_centroid().coords).centroid
            return end.x, end.y

        def _get_target_no_road(self):
            c = self.get_centroid().xy
            return c[0][0], c[1][0]

        def _get_target_roaming(self):
            c = self.get_centroid().xy
            x = c[0][0]
            y = c[1][0]
            if self.has_yaw is not None:
                return self._DISTANCE_WAYPOINT_ROAMING * math.cos(math.radians(self.has_yaw)) + x, \
                       self._DISTANCE_WAYPOINT_ROAMING * math.sin(math.radians(self.has_yaw)) + y
            else:
                return x, y

        def _get_target_yielding(self, t_x_orig, t_y_orig):
            geom_s = self.get_geometry()
            geom_s_c = self.get_centroid()
            _, _, _, p_int = self.get_intersecting_object()
            dist = geom_s.distance(Point(p_int))
            space_to_leave_before_intersection_point = 3.5  # m
            if dist < space_to_leave_before_intersection_point:
                space_to_leave_before_intersection_point = dist * 0.4
            t_x, t_y = 0, 0
            line = sympy.geometry.Segment2D(*list(geom_s_c.coords), (t_x_orig, t_y_orig))
            rad = dist - space_to_leave_before_intersection_point
            circ = sympy.geometry.Circle(*list(geom_s_c.coords), rad)
            targ = circ.intersection(line)
            if len(targ) == 0 or rad < 0:
                t_x = t_x_orig
                t_y = t_y_orig
                logger.debug("Computed yielding target by intersection point")
            elif self.has_length and self.has_width and rad < max(self.has_width, self.has_length) * 0.75:
                t_x = geom_s_c.x
                t_y = geom_s_c.y
                logger.debug("Computed yielding target by centroid - original target is too close - immediate action "
                             "required")
            else:
                t_x = targ[0][0]
                t_y = targ[0][1]
                logger.debug("Computing yielding target by " + str(space_to_leave_before_intersection_point) +
                      " m before intersection point")
            return t_x, t_y

        def _get_target_follow_vehicle(self):
            lead_veh = self.get_lead_vehicle()
            geom_l = lead_veh.get_geometry()
            geom_l_c = lead_veh.get_centroid()
            geom_s = self.drives[0].get_geometry()
            geom_s_c = self.drives[0].get_centroid()
            centroid_line = LineString([geom_l_c, geom_s_c])
            lead_int = geom_l.exterior.intersection(centroid_line)
            self_int = geom_s.exterior.intersection(centroid_line)
            dist = lead_int.distance(self_int)
            s = sympy.Symbol("s")
            line = sympy.geometry.line.Line2D(*lead_int.coords, *self_int.coords).arbitrary_point(s)
            hw = max(0.5, self._MAX_THW_FOR_BRAKING_WHEN_FOLLOWING * self.has_speed)  # Minimum HW is 0.5 m
            x, y = line.subs(s, hw / dist if hw < dist else 1)
            return float(x), float(y)
