import math
import logging

import owlready2
import shapely.geometry
import shapely.affinity
import sympy

from shapely import geometry

from pyauto.models.scene import Scene
from pyauto.models.scenery import Scenery
from pyauto.auto import Ontology
from pyauto.extras import utils

from tobm.scenarios.scenario import BenchmarkScenario

logger = logging.getLogger(__name__)


class XCrossing(BenchmarkScenario):
    """
    X-Crossing Benchmark.
    """

    MAX_SPAWN_TRIES = 10

    def __init__(self, file: str, duration: int | float, delta_t: int | float, seed: int = 0, scaling_factor: int = 1,
                 sim_models: list[str] | str = "tobm.sim_models.generic.*", to_augment: bool = True):
        self._name = "TOBM X-Crossing (n=" + str(scaling_factor) + ", s=" + str(seed) + ")"
        super().__init__(file, duration, delta_t, seed, scaling_factor, sim_models, to_augment)

    def _generate_config_from_scaling_factor(self, scaling_factor: int) -> BenchmarkScenario.BenchmarkConfig:
        """
        Creates a configuration file for X-Crossing given the scaling factor. Sets the following parameters:
        - length of crossing arms
        - number of vehicles
        - number of pedestrians
        :param scaling_factor: The scaling factor of the benchmark (positive, non-zero).
        :return: A configuration of parameters for the benchmark as a dict from strings (name of the parameter).
        """
        config = super()._generate_config_from_scaling_factor(scaling_factor)
        # Each scenario will contain 3 * n many traffic participants (and an appropriate road width for that)
        config["number_of_vehicles"] = scaling_factor * 2
        config["number_of_pedestrians"] = scaling_factor
        config["road_length"] = 10 + scaling_factor * 10
        return config

    def __str__(self):
        return self._name

    def _get_sim_models(self) -> list[str]:
        """
        :return: A list of additional simulation models that shall be loaded (besides the generic ones).
        """
        return ["tobm.sim_models.xcrossing.*"]

    def _create_scenery(self) -> Scenery:
        """
        Fills static scenery for X-crossing.
        :return: Scenery for X-crossing benchmark.
        """
        statics = super()._create_scenery()
        statics._name = "X-Crossing Scenery"

        l1_core = statics.ontology(Ontology.L1_Core)
        l1_de = statics.ontology(Ontology.L1_DE)
        geom = statics.ontology(Ontology.GeoSPARQL)

        if "road_length" in self._config.keys():
            road_length = self._config["road_length"]
        else:
            road_length = 30
        road_width = 20
        walkway_width = 2.5
        lane_width = 3.5
        divider_width = 1
        pedestrian_ford_width = 2.3
        pedestrian_ford_margin = 1
        traffic_light_diameter = 0.2
        traffic_light_height = 3.5
        traffic_light_margin = 1

        def create_arm(road_length, road_width, rotation):
            if rotation >= 315 or rotation < 45:
                x = road_width / 2 + road_length / 2
                y = 0
            elif 45 <= rotation < 135:
                x = 0
                y = road_width / 2 + road_length / 2
            elif 135 <= rotation < 225:
                x = - (road_width / 2 + road_length / 2)
                y = 0
            else:
                x = 0
                y = - (road_width / 2 + road_length / 2)
            road = l1_de.Urban_Road()
            road.set_geometry(x, y, road_length, road_width, rotate=rotation)
            cross_section = road.cross_section((l1_de.Walkway, walkway_width/road_width),
                                               (l1_core.Driveable_Lane, lane_width/road_width),
                                               (l1_core.Driveable_Lane, lane_width/road_width),
                                               (l1_de.Divider_Lane, divider_width/road_width),
                                               (l1_core.Driveable_Lane, lane_width/road_width),
                                               (l1_core.Driveable_Lane, lane_width/road_width),
                                               (l1_de.Walkway, walkway_width/road_width))
            def add_marker_between(lane1, lane2):
                marker_width = 0.2
                _, right, _, _ = utils.split_polygon_into_boundaries(lane1.get_geometry())
                marker_wkt = right.buffer(marker_width / 2, cap_style=2).wkt
                marker = l1_de.Line_Marker()
                marker_geom = geom.Geometry()
                marker_geom.asWKT = [marker_wkt]
                marker.hasGeometry = [marker_geom]

            def add_marker_in_front(lane):
                marker_width = 0.4
                _, _, front, back = utils.split_polygon_into_boundaries(lane.get_geometry())
                medium = sympy.Segment(*back.centroid.coords, *front.centroid.coords)
                x = sympy.Symbol("x")
                marker_point = medium.arbitrary_point(x).subs(
                    {x: (pedestrian_ford_width + pedestrian_ford_margin * 2) / lane.has_length})
                marker_shapely = back.buffer(marker_width / 2, cap_style=2)
                x = marker_point.x - marker_shapely.centroid.x
                y = marker_point.y - marker_shapely.centroid.y
                marker_shapely = shapely.affinity.translate(marker_shapely, x, y)
                marker = l1_de.Wait_Line()
                marker_geom = geom.Geometry()
                marker_geom.asWKT = [marker_shapely.wkt]
                marker.hasGeometry = [marker_geom]

            add_marker_between(cross_section[1], cross_section[2])
            add_marker_between(cross_section[4], cross_section[5])
            add_marker_in_front(cross_section[1])
            add_marker_in_front(cross_section[2])

            def add_pedestrian_ford_to_lane_fronts(*lanes):
                assert(len(lanes) > 0)
                _, _, front, back = utils.split_polygon_into_boundaries(lanes[0].get_geometry())
                width = lanes[0].has_width
                for lane in lanes[1:]:
                    _, _, f, b = utils.split_polygon_into_boundaries(lane.get_geometry())
                    front = front.union(f)
                    back = back.union(b)
                    width += lane.has_width
                medium = sympy.Segment(*back.centroid.coords, *front.centroid.coords)
                x = sympy.Symbol("x")
                ford_point = medium.arbitrary_point(x).subs(
                    {x: (pedestrian_ford_width / 2 + pedestrian_ford_margin) / lane.has_length})
                ford = l1_de.Pedestrian_Ford()
                ford.set_geometry(*ford_point, pedestrian_ford_width, width, rotate=rotation)

            add_pedestrian_ford_to_lane_fronts(cross_section[1], cross_section[2])
            add_pedestrian_ford_to_lane_fronts(cross_section[4], cross_section[5])

            def add_traffic_light_to_front(obj):
                _, _, front, back = utils.split_polygon_into_boundaries(obj.get_geometry())
                medium = sympy.Segment(*back.centroid.coords, *front.centroid.coords)
                x = sympy.Symbol("x")
                ford_point = medium.arbitrary_point(x).subs(
                    {x: (traffic_light_diameter / 2 + traffic_light_margin) / obj.has_length})
                traffic_light_shapely = geometry.Point(*ford_point).buffer(traffic_light_diameter)
                traffic_light = l1_de.Vehicle_Traffic_Light()
                traffic_light.is_a.append(l1_de.Broken_Light_Sign)
                traffic_light_geom = geom.Geometry()
                traffic_light_geom.asWKT = [traffic_light_shapely.wkt]
                traffic_light.hasGeometry = [traffic_light_geom]
                traffic_light.has_width = traffic_light_diameter
                traffic_light.has_length = traffic_light_diameter
                traffic_light.has_height = traffic_light_height

            add_traffic_light_to_front(cross_section[0])
            add_traffic_light_to_front(cross_section[3])

            return road, cross_section

        degrees = {"east": 0, "north": 90, "west": 180, "south": 270}
        arms = dict()
        for deg in degrees.keys():
            arms[deg] = create_arm(road_length, road_width, rotation=degrees[deg])

        crossing = l1_de.Crossing()
        crossing.set_geometry(0, 0, road_width, road_width)
        crossing.set_roads(*[x[0] for x in arms.values()])

        def create_walkway_connection(arm):
            if arm == "east":
                x = road_width / 2
                y = -road_width / 2
            elif arm == "north":
                x = road_width / 2
                y = road_width / 2
            elif arm == "west":
                x = -road_width / 2
                y = road_width / 2
            else:
                x = -road_width / 2
                y = -road_width / 2
            walkway_connection = l1_de.Walkway()
            walkway_connection.set_shapely_geometry(geometry.Point(x, y).buffer(walkway_width).
                                                    intersection(crossing.get_geometry()))
            return walkway_connection

        for i, arm in enumerate(arms.keys()):
            arm1 = arms[arm][1]
            arm2 = arms[list(arms.keys())[(i + 1) % len(list(arms.keys()))]][1]
            arm3 = arms[list(arms.keys())[(i + 2) % len(list(arms.keys()))]][1]
            arm4 = arms[list(arms.keys())[(i + 3) % len(list(arms.keys()))]][1]
            arm1_walkway = arm1[-1]
            arm2_walkway = arm2[0]
            walkway = create_walkway_connection(arm)
            walkway.has_successor_lane = [arm1_walkway, arm2_walkway]
            arm1_walkway.has_successor_lane = [walkway]
            arm2_walkway.has_successor_lane = [walkway]
            arm1[1].has_successor_lane = [arm2[-2], arm2[-3], arm3[-2], arm3[-3]]
            arm1[2].has_successor_lane = [arm4[-2], arm4[-3]]

        return statics

    def _fill_initial_scene(self) -> Scene:
        """
        Fills initial scene for the X-crossing.
        :return: Initial scene for the X-crossing benchmark.
        """
        sc = super()._fill_initial_scene()

        l4_co = sc.ontology(Ontology.L4_Core)
        l4_de = sc.ontology(Ontology.L4_DE)

        if "number_of_vehicles" in self._config.keys():
            num_vehicles = self._config["number_of_vehicles"]
        else:
            num_vehicles = 4
        if "number_of_pedestrians" in self._config.keys():
            num_pedestrians = self._config["number_of_pedestrians"]
        else:
            num_pedestrians = 2

        for i in range(num_vehicles):
            vehicle = l4_de.Passenger_Car("veh" + str(i + 1))
            succ = vehicle.spawn(length=self._random.uniform(3.9, 4.4), width=self._random.uniform(1.6, 1.9),
                                 height=self._random.uniform(1.5, 2), speed=self._random.uniform(2, 4),
                                 driver=l4_co.XCrossingDriver, max_number_of_tries=self.MAX_SPAWN_TRIES)
            if not succ:
                owlready2.destroy_entity(vehicle)
                logger.warning("Could not spawn vehicle no. " + str(i + 1) + " after " + str(self.MAX_SPAWN_TRIES) +
                               " tries due to spawn collisions")

        for i in range(num_pedestrians):
            ped = l4_co.Pedestrian("ped" + str(i + 1))
            succ = ped.spawn(width=self._random.uniform(0.3, 0.7), length=self._random.uniform(0.3, 0.7),
                             height=self._random.uniform(1.6, 2.0), speed=self._random.uniform(0.5, 1.0),
                             max_number_of_tries=self.MAX_SPAWN_TRIES,
                             offset=l4_co.Pedestrian._END_OF_WALKWAY_LENGTH * 5)
            if not succ:
                owlready2.destroy_entity(ped)
                logger.warning("Could not spawn pedestrian no. " + str(i + 1) + " after " + str(self.MAX_SPAWN_TRIES) +
                               " tries due to spawn collisions")

        return sc
