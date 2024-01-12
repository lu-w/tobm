import logging

from pyauto import auto
from pyauto.models.scenario import Scenario
from pyauto.models.scene import Scene
from pyauto.models.scenery import Scenery

logger = logging.getLogger(__name__)

class BenchmarkScenario(Scenario):
    """
    A benchmark scenario is a pyauto scenario with special functionality for setting up scenery and the intial scenes,
    as well as benchmark generation.
    Concrete benchmarks shall overwrite:
    - _create_scenery to create the static scenery.
    - _fill_initial_scene to create the initial scene from which the simulation is started.
    - _generate_config_from_scaling_factor (optional, if the scaling factor has implications on the benchmark size).
    - _get_sim_models (optional, if new simulation models shall be included).
    """
    class BenchmarkConfig(dict):
        # This may be extended in the future. Right now, it is just a dictionary of configuration values.
        pass

    def __init__(self, file: str, duration: int | float, delta_t: int | float, seed: int = 0, scaling_factor: int = 1,
                 sim_models: list[str] | str = "tobm.sim_models.generic.*", to_augment: bool = True):
        """
        Creates a new benchmark.
        :param file: The file to which the benchmark is later saved (when calling generate()).
        :param duration: Duration (seconds) of the scenario.
        :param delta_t: Time (seconds) between discrete scenes in the scenario.
        :param seed: An optional seed for consistent random number generation.
        :param scaling_factor: An optional scaling factor the scenario which increases the size of the scenery.
        :param sim_models: A list or string of importable Python module(s). Will be imported in the given order. Using
            wildcards at the end is possible, e.g. "a.b.*", which then recursively imports *all* Python files located
            in the package's (sub)folder(s).
        :param to_augment: Whether to augment this scenario after simulation. By default, it does not augment on
            accident scenarios.
        """
        logger.info("Generating benchmark " + str(self))
        if isinstance(sim_models, str):
            sim_models = [sim_models]
        sim_models += self._get_sim_models()
        super().__init__(scene_number=1, name=__class__.__name__, load_cp=True, add_extras=True, more_extras=sim_models,
                         seed=seed)
        self._file = file
        self._simulation_duration = duration
        self._delta_t = delta_t
        self._scaling_factor = scaling_factor
        self._sim_models = sim_models
        self._to_augment = to_augment
        self._config = self._generate_config_from_scaling_factor(scaling_factor)

    def _generate_config_from_scaling_factor(self, scaling_factor: int) -> BenchmarkConfig:
        """
        Creates a configuration file for the benchmark's parameters (e.g. number of vehicles) given the scaling factor.
        Default implementation, overwrite if needed.
        :param scaling_factor: The scaling factor of the benchmark (positive, non-zero).
        :return: A configuration of parameters for the benchmark as a dict from strings (name of the parameter).
        """
        assert(scaling_factor > 0)
        return BenchmarkScenario.BenchmarkConfig()

    def _get_sim_models(self) -> list[str]:
        """
        Default implementation, overwrite if needed.
        :return: A list of additional simulation models that shall be loaded (besides the generic ones).
        """
        return []

    def _create_scenery(self) -> Scenery:
        """
        To be implemented by any specific benchmark.
        :returns: A scenery representing the scenery (i.e., static parts) of the benchmark.
        """
        scenery = Scenery(add_extras=True, more_extras=self._sim_models, load_cp=True)
        scenery._random = self._random
        scenery._np_random = self._np_random
        return scenery

    def _fill_initial_scene(self) -> Scene:
        """
        To be implemented by any specific benchmark. Shall fill the already created but empty initial scene.
        """
        return self[0]

    def _setup(self):
        """
        Sets up this benchmark by creating and setting the scenery and creating the initial scene.
        """
        logger.debug("Setting scenery for benchmark scenario " + str(self))
        self.set_scenery(self._create_scenery())
        logger.debug("Filling initial scene for benchmark scenario " + str(self))
        self._fill_initial_scene()

    def generate(self, save_on_accident: bool = False, kbs_file_name: str = None, scenery_file_name: str = None,
                 save_geometries: bool = False) -> bool:
        """
        Generates this benchmark and saves it to the previously given file location.
        Does not save geosparql.Geometry individuals.
        :param save_on_accident: Whether to save (and augment, if set) the benchmark if an accident happened.
        :param kbs_file_name: An optional custom file name for the .kbs file (used instead of the auto-generated one).
        :param scnerey_file_name: An optional custom file name for the scenery file (used instead of the auto-generated
            one).
        :param save_geometries: Whether to also save geometries in the output OWL files.
        :returns: True iff. the generated benchmark scenario contains an accident.
        """
        self._setup()
        accident = self.simulate(self._simulation_duration, self._delta_t,
                                 to_keep={"l1_core.has_road", "l1_core.has_lane", "l1_core.has_successor_lane",
                                          "l1_de.is_lane_left_of", "l1_de.is_lane_right_of",
                                          "l1_de.is_lane_parallel_to", "l1_core.has_predecessor_lane",
                                          "l4_core.drives", "l4_core.driven_by", "physics.has_width",
                                          "physics.has_length", "physics.has_height"},
                                 prioritize=["l4_core.Human_Driver", "l4_core.Automated_Driving_Function",
                                             "l4_core.drives", "l4_core.Vehicle"])
        if self._to_augment and (not accident or save_on_accident):
            self.augment()
        if self._file is not None and (not accident or save_on_accident):
            if not save_geometries:
                to_ignore = {"geosparql.Geometry"}
            else:
                to_ignore = None
            self.save_abox(self._file, to_ignore=to_ignore, kbs_file_name=kbs_file_name,
                           scenery_file_name=scenery_file_name)
        if accident:
            logger.info("Accident happened during simulation")
        if self._file and not save_on_accident and accident:
            logger.warning("Did *not* save to " + str(self._file))
        return accident
