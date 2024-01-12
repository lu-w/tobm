import math
import logging

import owlready2

from shapely.affinity import translate, rotate
from shapely.geometry import Point, Polygon, LineString
import sympy

from pyauto.extras import utils
from pyauto.extras.owl import simulate
from pyauto import auto

logger = logging.getLogger(__name__)

physics = auto.world.get_ontology(auto.Ontology.Physics.value)

with physics:

    class Dynamical_Object(owlready2.Thing):
        """
        Dynamical objects are the entry point to TOBM simulations. Here, central methods are defined for computing
        the new state of some individual after a simulation, including their geometrical shape, their yaw, their speed,
        and their position.
        Concrete simulation models shall overwrite the methods introduced here. If this is not done, a default
        simulation model will be used, which just assume the next target to be 10 m in front of the object and use the
        maximum possible speed.
        In general, overwrite the following methods:
        - get_target: The next waypoint
        - get_target_speed: The desired speed to have at the next waypoint
        Optionally, __init__ can be overwritte if new state variables have to be introduced.
        """

        _MAX_DISTANCE_FOR_TURNING = 12  # m, may needs to be adapted based on the concrete geometry of the intersection.

        ########################################################################
        # METHODS REPRESENTING DEFAULT SIMULATION MODELS - OVERWRITE IF NEEDED #
        ########################################################################

        def __init__(self, name=None, namespace=None, is_a=None, *args):
            owlready2.Thing.__init__(self, name=name, namespace=namespace, is_a=is_a, *args)
            """"
            Default initializer. Is called once every scene before simulation. Concrete models can overwrite this if
            e.g. new variables have to be added to the model (e.g. for storing modes).
            """
            self.has_yaw_rate = 0
            self.has_acceleration = 0
            self.has_speed = 0
            self.mapping = {}

        def get_target(self) -> tuple:
            """
            Default implementation: target is 10 m in front (of self's yaw). Concrete models shall overwrite this.
            :return: A triple of a target point and the distance to this point (to increase performance).
            """
            c = self.get_geometry().centroid.coords[0]
            x = c[0]
            y = c[1]
            if self.has_yaw is not None:
                return 10 * math.cos(math.radians(self.has_yaw)) + x, 10 * math.sin(math.radians(self.has_yaw)) + y, 10
            else:
                return x, y, 0.001

        def get_target_speed(self, warning=True) -> float | int:
            """
            Default implementation: target speed is max. speed. Concrete models shall overwrite this.
            :param warning: Whether to print warnings if this default simulation model is used.
            :return: The target speed.
            """
            if warning:
                logger.warning("Using default simulation model for " + str(self))
            s = self.get_maximum_speed()
            if s is None:
                s = 0
            return s

        ########################################################################
        # END DEFAULT SIMULATION METHODS                                       #
        ########################################################################

        def simulate(self, mapping: dict[owlready2.NamedIndividual, owlready2.NamedIndividual], delta_t: float | int):
            """
            Overwrites the default pyauto simulation method. Computes all required data (speed, acceleration, etc.) to
            reach the current desired way point and target speed.
            Note that this also handles the case of drivers driving vehicles - vehicles are automatically updated.
            :param mapping: A mapping from old to new individuals, from the last to the current time point.
            :param delta_t: The time difference of the simulation step.
            """
            logger.debug("========== Simulating " + str(self) + " ==========")
            self.mapping = mapping
            new_self = mapping[self]
            t_x, t_y, distance = self.get_target()
            target = Point(t_x, t_y)
            if self.has_speed > self._RELEVANT_LOWEST_SPEED:
                yaw_rate = self._get_new_yaw_rate(delta_t, target)
                # avoids overwriting anything that may have been set by some other simulation model
                if yaw_rate is not None:
                    new_self.has_yaw_rate = yaw_rate
            acceleration = self._get_new_acceleration(delta_t, target, distance)
            if acceleration is not None:  # same as above
                new_self.has_acceleration = acceleration
            for v in new_self.drives:  # propagates control input to the driven vehicle(s)
                v.has_yaw_rate = new_self.has_yaw_rate
                v.has_acceleration = new_self.has_acceleration
            # updates new object's speed, yaw, and geometry in new scene
            new_self.update_speed(self, delta_t)
            new_self.update_yaw(self, delta_t)
            new_self.update_position(self, delta_t)

        def get_optimized_attr(self, attr: str, opt_func=max):
            if hasattr(self, attr):
                attrs = [x for y in self.is_a for x in getattr(self, attr)]
                if len(attrs) == 0:
                    attrs = [x for y in self.INDIRECT_is_a if hasattr(y, attr) for x in getattr(y, attr)]
                if len(attrs) == 0:
                    for veh in self.drives:
                        attrs += [x for y in veh.INDIRECT_is_a if hasattr(y, attr) for x in getattr(y, attr)]
                if len(attrs) > 0:
                    return opt_func(attrs)
            return 0

        def get_maximum_speed(self):
            return self.get_optimized_attr("has_maximum_speed")

        def get_maximum_acceleration(self):
            return self.get_optimized_attr("has_maximum_acceleration")

        def get_maximum_deceleration(self):
            return self.get_optimized_attr("has_maximum_deceleration", min)

        def get_maximum_yaw_rate(self):
            return self.get_optimized_attr("has_maximum_yaw_rate")

        def has_reached_maximum_speed(self):
            max_speed = self.get_maximum_speed()
            return (max_speed is not None and self.has_speed > max_speed)

        def _get_yaw_rate_to_reach_point(self, target: Point, delta_t: float | int, max_yaw_rate: float,
                                         distance: float | int = None, ignore_max_distance_for_turning: bool = False):
            yr = 0
            alpha = self.compute_angle_between_yaw_and_point((target.x, target.y))
            if alpha > 180:
                alpha = 360 - alpha
            # Ignores minor turning angles - just drive straight until it becomes not so minor.
            if self.has_speed != 0 and 0.5 < alpha < 179.5:
                sgn = 1  # sgn = 1 -> left turn, sgn = -1 -> right turn
                if self.is_point_right_of((target.x, target.y)):
                    sgn = -1
                if distance is None:
                    if hasattr(self, "drives") and len(self.drives) > 0:
                        self_geo = self.drives[0].get_geometry()
                    else:
                        self_geo = self.get_geometry()
                    distance = self_geo.distance(target)
                if not distance:
                    distance = 0.001
                if ignore_max_distance_for_turning or distance <= self._MAX_DISTANCE_FOR_TURNING:
                    t_p = distance / self.has_speed
                    yr = sgn *  max_yaw_rate * self._get_yaw_rate_smoothing_factor(alpha / t_p)
                    # Use lower yaw rates at very low speeds
                    if self.has_speed < 0.85:
                        yr *= self.has_speed / 0.85
                else:
                    logger.debug("Not close enough to initiate turn - will continue to go straight for a bit")
                    yr = 0
                logger.debug("Deciding to turn by yaw rate " + str(yr) + " out of max yaw rate " + str(max_yaw_rate))
            return yr

        def _get_yaw_rate_smoothing_factor(self, degrees_per_second: int | float) -> float:
            return max(0.1, 1 / (1 + math.exp(- 0.1 * (degrees_per_second - 30))))

        def _get_new_acceleration(self, delta_t: float | int, target: Point, distance: float | int=None) -> float:
            ac = 0
            max_deceleration = self.get_maximum_deceleration()
            max_acceleration = self.get_maximum_acceleration()
            target_speed = self.get_target_speed()
            if distance is None:
                geom_s = self.get_geometry()
                if len(self.drives) > 0:
                    geom_d = self.drives[0].get_geometry()
                else:
                    geom_d = geom_s
                p_int = LineString([geom_s.centroid, target]).intersection(geom_d)
                distance = 0
                if isinstance(p_int, list) and len(p_int) > 0:
                    distance = p_int[0].distance(target)
                elif p_int is not None:
                    distance = p_int.distance(target)
            if not distance:
                distance = 0.001
            if target_speed is None:
                ac = - self.has_speed ** 2 / distance
            else:
                ac = (target_speed - self.has_speed) ** 2 / distance
                if target_speed - self.has_speed < 0:
                    ac = max(-ac, self.get_maximum_deceleration())
                if ac > 0:
                    ac = min(ac, self.get_maximum_acceleration())
                logger.debug("Needing to accelerate by " + str(ac) + " m/s^2 to reach " + str(distance) +
                             " m and reduce/increase speed by " + str(target_speed - self.has_speed) + " m/s")
            if ac > 0 and max_deceleration is not None and max_acceleration is not None and \
                    self.has_reached_maximum_speed():
                ac = 0
            logger.debug("Deciding to acc. by " + str(ac))
            return ac

        def _get_new_yaw_rate(self, delta_t: float | int, target: Point, distance: float | int=None) -> float:
            myr = self.get_maximum_yaw_rate()
            yr = 0
            if myr is not None and self.has_speed is not None and \
                    self.has_speed > self._RELEVANT_LOWEST_SPEED:
                yr = self._get_yaw_rate_to_reach_point(target, delta_t, myr, distance)
            return yr

        def is_target_free(self, target: tuple[int | float, int | float]):
            geom = self.get_geometry()
            geom_c = geom.centroid
            geom_t = translate(geom, xoff=geom_c.x - target[0], yoff=geom_c.y - target[1])
            for obj in self.namespace.world.search(
                    type=self.namespace.world.get_ontology(auto.Ontology.Physics.value).Spatial_Object):
                if self != obj and obj.has_height and obj.get_geometry().intersects(geom_t):
                    return False
            return True

        def update_yaw(self, prev_self: owlready2.NamedIndividual, delta_t: float | int):
            """
            Updates the yaw of the prevous object based on the computed yaw rate and speed.
            Has to be called after `update_speed`.
            :param prev_self: The previous state of the current object.
            :param delta_t: The simulation's time difference.
            """
            if prev_self.has_yaw is not None:
                if self.has_yaw_rate is not None and self.has_speed > self._RELEVANT_LOWEST_SPEED:
                    self.has_yaw = (prev_self.has_yaw + self.has_yaw_rate * delta_t) % 360
                else:
                    self.has_yaw = prev_self.has_yaw

        def update_speed(self, prev_self: owlready2.NamedIndividual, delta_t: float | int):
            """
            Updates the speed of the prevous object based on the computed acceleration.
            Assumption: new speed and yaw of `self` has been set before
            :param prev_self: The previous state of the current object.
            :param delta_t: The simulation's time difference.
            """
            if prev_self.has_speed is not None and self.has_acceleration is not None:
                # We do not allow negative speeds.
                self.has_speed = max(0, prev_self.has_speed + self.has_acceleration * delta_t)
            if self.has_speed < self._RELEVANT_LOWEST_SPEED:
                self.has_acceleration = 0
                self.has_yaw_rate = 0

        def update_position(self, prev_self: owlready2.NamedIndividual, delta_t: float | int):
            """
            Updates the geometry of the prevous object based on the computed yaw and speed, i.e., it writes into
            this object's hasGeometry property.
            Assumption: new speed and yaw of `self` have been set before.
            :param prev_self: The previous state of the current object.
            :param delta_t: The simulation's time difference.
            """
            if self is not None and self.has_yaw is not None and self.has_speed is not None and \
                    self.has_geometry() and self.has_speed > self._RELEVANT_LOWEST_SPEED:
                xoff = math.cos(math.radians(self.has_yaw)) * self.has_speed * delta_t
                yoff = math.sin(math.radians(self.has_yaw)) * self.has_speed * delta_t
                zoff = 0
                s_geo = self.get_geometry()
                if len(self.drives) > 0:
                    d_geo = self.drives[0].get_geometry()
                else:
                    d_geo = s_geo
                if isinstance(d_geo, Polygon):
                    length = self.has_length
                    if not length and len(self.drives) > 0:
                        length = self.drives[0].has_length
                    if not length:
                        length = 0
                    length *= 0.4
                    inv_yaw = math.radians(self.has_yaw + 180) % 360
                    center = Point(d_geo.centroid.x + length * math.cos(inv_yaw),
                                   d_geo.centroid.y + length * math.sin(inv_yaw))
                else:
                    center = d_geo.centroid
                new_geo = rotate(s_geo, angle=self.has_yaw - prev_self.has_yaw, origin=center)
                new_geo = translate(new_geo, xoff=xoff, yoff=yoff, zoff=zoff)
                self.hasGeometry[0].asWKT = [new_geo.wkt]
