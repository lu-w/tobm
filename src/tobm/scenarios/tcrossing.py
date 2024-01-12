import math
import logging

import owlready2

from shapely import geometry, affinity

from pyauto.models.scene import Scene
from pyauto.models.scenery import Scenery
from pyauto.auto import Ontology

from tobm.scenarios.scenario import BenchmarkScenario

logger = logging.getLogger(__name__)


class TCrossing(BenchmarkScenario):
    """
    T-Crossing benchmark.
    """

    MAX_SPAWN_TRIES = 10

    def __init__(self, file: str, duration: int | float, delta_t: int | float, seed: int = 0, scaling_factor: int = 1,
                 sim_models: list[str] | str = "tobm.sim_models.generic.*", to_augment: bool = True):
        self._name = "TOBM T-Crossing (n=" + str(scaling_factor) + ", s=" + str(seed) + ")"
        super().__init__(file, duration, delta_t, seed, scaling_factor, sim_models, to_augment)

    def _generate_config_from_scaling_factor(self, scaling_factor: int) -> BenchmarkScenario.BenchmarkConfig:
        """
        Creates a configuration file for T-Crossing given the scaling factor. Sets the following parameters:
        - number of vehicles
        - length of crossing arms
        :param scaling_factor: The scaling factor of the benchmark (positive, non-zero).
        :return: A configuration of parameters for the benchmark as a dict from strings (name of the parameter).
        """
        config = super()._generate_config_from_scaling_factor(scaling_factor)
        # Each scenario will contain 5 * n many traffic participants (and an appropriate road width for that)
        config["number_of_vehicles"] = 2 * scaling_factor
        config["number_of_pedestrians"] = scaling_factor
        config["number_of_bicycles"] = scaling_factor
        config["road_length"] = 5 + scaling_factor * 15
        # Parking vehicles are only located on wester arm -> one every 15m
        config["number_of_parking_vehicles"] = scaling_factor
        return config

    def __str__(self):
        return self._name

    def _get_sim_models(self) -> list[str]:
        """
        :return: A list of additional simulation models that shall be loaded (besides the generic ones).
        """
        return ["tobm.sim_models.tcrossing.*"]

    def _create_scenery(self) -> Scenery:
        """
        Fills static scenery for T-crossing.
        :return: Scenery for T-crossing benchmark.
        """
        statics = super()._create_scenery()
        statics._name = "T-Crossing Scenery"

        if "road_length" in self._config.keys():
            road_length = self._config["road_length"]
        else:
            road_length = 30
        walkway_width = 2.5
        bikeway_width = 2.0
        lane_width = 3.5
        pedestrian_crossing_margin = 3
        pedestrian_crossing_width = 3.5

        road_width = walkway_width * 2 + lane_width * 2
        south_road_width = walkway_width * 2 + lane_width * 2 + bikeway_width * 2
        walkway_ratio = walkway_width / road_width
        lane_ratio = lane_width / road_width
        south_walkway_ratio = walkway_width / south_road_width
        south_lane_ratio = lane_width / south_road_width
        south_bikeway_ratio = bikeway_width / south_road_width

        l1_core = statics.ontology(Ontology.L1_Core)
        l4_core = statics.ontology(Ontology.L4_Core)
        l1_de = statics.ontology(Ontology.L1_DE)
        l4_de = statics.ontology(Ontology.L4_DE)

        road_east = l1_de.Urban_Road()
        road_east.set_geometry(road_length / 2 + south_road_width / 2, 0, road_length, road_width)
        w1, l1, l2, w2 = road_east.cross_section((l1_de.Walkway, walkway_ratio), (l1_core.Driveable_Lane, lane_ratio),
                                                (l1_core.Driveable_Lane, lane_ratio), (l1_de.Walkway, walkway_ratio))
        road_east.is_a.append(l1_core.has_lane.exactly(2, l1_core.Driveable_Lane))

        road_west = l1_de.Urban_Road()
        road_west.set_geometry(-road_length / 2 - south_road_width / 2, 0, road_length, road_width, rotate=180)
        w3, l3, l4, w4 = road_west.cross_section((l1_de.Walkway, walkway_ratio), (l1_core.Driveable_Lane, lane_ratio),
                                                 (l1_core.Driveable_Lane, lane_ratio), (l1_de.Walkway, walkway_ratio))
        road_west.is_a.append(l1_core.has_lane.exactly(2, l1_core.Driveable_Lane))

        road_south = l1_de.Urban_Road()
        road_south.set_geometry(0, -road_length / 2 - road_width / 2, road_length, south_road_width, rotate=270)
        w5, b1, l5, l6, b2, w7 = road_south.cross_section((l1_de.Walkway, south_walkway_ratio),
                                                          (l1_de.Bikeway_Lane, south_bikeway_ratio),
                                                          (l1_core.Driveable_Lane, south_lane_ratio),
                                                          (l1_core.Driveable_Lane, south_lane_ratio),
                                                          (l1_de.Bikeway_Lane, south_bikeway_ratio),
                                                          (l1_de.Walkway, south_walkway_ratio))

        pedestrian_crossing = l1_de.Pedestrian_Crossing()
        pedestrian_crossing.set_geometry(
            -south_road_width / 2 - pedestrian_crossing_margin - pedestrian_crossing_width / 2, 0,
            lane_width * 2, pedestrian_crossing_width, rotate=90
        )
        road_west.add_lane(pedestrian_crossing)

        crossing = l1_de.Crossing()
        crossing.set_geometry(0, -walkway_width / 2, south_road_width, road_width - walkway_width)
        crossing.set_roads(road_west, road_east, road_south)

        northern_walkway_at_crossing = l1_de.Walkway()
        northern_walkway_at_crossing.set_geometry(0, lane_width + walkway_width / 2, south_road_width, walkway_width)

        western_walkway_at_crossing = l1_de.Walkway()
        western_walkway_at_crossing.set_shapely_geometry(
            geometry.Point(-lane_width - walkway_width - bikeway_width, -lane_width - walkway_width).
            buffer(walkway_width).intersection(crossing.get_geometry()))
        eastern_walkway_at_crossing = l1_de.Walkway()
        eastern_walkway_at_crossing.set_shapely_geometry(
            geometry.Point(lane_width + walkway_width + bikeway_width, -lane_width - walkway_width).
            buffer(walkway_width).intersection(crossing.get_geometry()))

        l1.has_successor_lane = [l4, l6]
        l3.has_successor_lane = [l2, l6]
        l5.has_successor_lane = [l2, l4]
        w4.has_successor_lane = [northern_walkway_at_crossing]
        pedestrian_crossing.has_successor_lane = [w4]
        northern_walkway_at_crossing.has_successor_lane = [w1]
        western_walkway_at_crossing.has_successor_lane = [w3]

        if "number_of_parking_vehicles" in self._config.keys():
            num_of_parking_vehicles = self._config["number_of_parking_vehicles"]
        else:
            num_of_parking_vehicles = 1

        for i in range(num_of_parking_vehicles):
            vehicle = l4_core.Vehicle()  # intentionally not a parking vehicle - it's an inference task
            succ = vehicle.spawn(speed=0, max_number_of_tries=self.MAX_SPAWN_TRIES, spawn_lane=w3, driver=None,
                                 offset=8)
            if not succ:
                owlready2.destroy_entity(vehicle)
            else:
                vehicle.has_yaw = (vehicle.has_yaw + 180) % 360
                geom = vehicle.get_geometry()
                shifted_geom = affinity.translate(geom, yoff=walkway_width / 4)
                vehicle.hasGeometry[0].asWKT[0] = shifted_geom.wkt

        return statics

    def _fill_initial_scene(self) -> Scene:
        """
        Fills initial scene for the T-crossing.
        :return: Initial scene for the T-crossing benchmark.
        """
        sc = super()._fill_initial_scene()
        l4_de = sc.ontology(Ontology.L4_DE)
        l4_co = sc.ontology(Ontology.L4_Core)

        l4_co = sc.ontology(Ontology.L4_Core)

        if "number_of_vehicles" in self._config.keys():
            num_vehicles = self._config["number_of_vehicles"]
        else:
            num_vehicles = 4
        if "number_of_pedestrians" in self._config.keys():
            num_pedestrians = self._config["number_of_pedestrians"]
        else:
            num_pedestrians = 4
        if "number_of_bicycles" in self._config.keys():
            num_bicycles = self._config["number_of_bicycles"]
        else:
            num_bicycles = 4

        for i in range(num_vehicles):
            vehicle = l4_de.Passenger_Car()
            succ = vehicle.spawn(length=self._random.uniform(3.9, 4.4), width=self._random.uniform(1.6, 1.9),
                                 height=self._random.uniform(1.5, 2), speed=self._random.uniform(2, 4),
                                 driver=l4_co.TCrossingDriver, max_number_of_tries=self.MAX_SPAWN_TRIES)
            if not succ:
                owlready2.destroy_entity(vehicle)
                logger.warning("Could not spawn vehicle no. " + str(i) + " after " + str(self.MAX_SPAWN_TRIES) +
                               " tries due to spawn collisions")

        if num_bicycles > 0:
            # Fixed bicyclist
            bic = l4_de.Bicycle()
            bic.set_geometry(-4.5, -12.5, length=1, width=0.5, rotate=90)
            bic.has_speed = 2.7
            bic.has_height = 1.0
            bic.has_yaw = 90
            hum = bic.add_driver(l4_de.TCrossingBicyclist)
            hum.has_height = bic.has_height + 0.8

        for i in range(num_bicycles - 1):
            bic = l4_de.Bicycle()
            hum = bic.spawn(width=self._random.uniform(0.4, 0.6), length=self._random.uniform(0.9, 1.1),
                            height=self._random.uniform(0.8, 1.2), speed=self._random.uniform(2, 4),
                            driver=l4_de.Bicyclist, max_number_of_tries=self.MAX_SPAWN_TRIES)
            if hum is not None:
                hum.has_height = bic.has_height + 0.8
            else:
                owlready2.destroy_entity(bic)
                logger.warning("Could not spawn bicycle no. " + str(i + 1) + " after " + str(self.MAX_SPAWN_TRIES) +
                               " tries due to spawn collisions")

        for i in range(num_pedestrians):
            ped = l4_co.Pedestrian()
            succ = ped.spawn(width=self._random.uniform(0.3, 0.7), length=self._random.uniform(0.3, 0.7),
                             height=self._random.uniform(1.6, 2.0), speed=self._random.uniform(0.5, 1.0),
                             max_number_of_tries=self.MAX_SPAWN_TRIES,
                             offset=l4_co.Pedestrian._END_OF_WALKWAY_LENGTH * 5)
            if not succ:
                owlready2.destroy_entity(ped)
                logger.warning("Could not spawn pedestrian no. " + str(i) + " after " + str(self.MAX_SPAWN_TRIES) +
                               " tries due to spawn collisions")

        return sc
