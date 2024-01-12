import owlready2

from pyauto import auto

l4_core = auto.world.get_ontology(auto.Ontology.L4_Core.value)

with l4_core:
    class Automated_Driving_Function(owlready2.Thing):
        pass
