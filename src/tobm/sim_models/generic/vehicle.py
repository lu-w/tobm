import owlready2

from shapely.geometry import Point

from pyauto import auto

l4_core = auto.world.get_ontology(auto.Ontology.L4_Core.value)

with l4_core:

    class Vehicle(owlready2.Thing):
        """
        This class serves only to avoid setting accelerations and yaw rates for vehicles, as they are handled by drivers
        """
        pass

        def _get_new_acceleration(self, delta_t: float | int, target: Point, distance: float | int=None) -> float:
            # Acceleration is set prior by driver
            pass

        def _get_new_yaw_rate(self, delta_t: float | int, target: Point, distance: float | int=None) -> float:
            # Yaw rate is set prior by driver
            pass
