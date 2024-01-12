from typing import get_type_hints
from functools import lru_cache
from enum import Enum
import owlready2
import itertools
import logging
import tqdm

"""
This module enables an automatic augmentation of an OWL ontology as loaded by owlready2. Multiple augmentations are 
avoided if the module has performed certain augmentations before. The augmentation is triggered by calling 
do_augmentation().
"""

# Important - disables blocking of _propdict attribute by owlready2.
owlready2.SPECIAL_ATTRS.add("_propdict")

# Logging
logger = logging.getLogger(__name__)

####################
# Internal globals #
####################

_CREATED_REIFIED_OBJECT_PROPERTIES = set()
_CREATED_REIFIED_DATA_PROPERTIES = set()
_CREATED_OBJECT_PROPERTIES = set()

##############
# Decorators #
##############


class AugmentationType(Enum):
    CLASS_SUBSUMPTION = 0
    CLASS_EQUIVALENCE = 1
    OBJECT_PROPERTY = 2
    DATA_PROPERTY = 3
    REIFIED_OBJECT_PROPERTY = 4
    REIFIED_DATA_PROPERTY = 5


def augment_class(cls):
    setattr(cls, "_propdict", {})
    for methodname in dir(cls):
        if methodname in cls.__dict__:
            method = getattr(cls, methodname)
            if hasattr(method, '_prop'):
                cls._propdict.update({cls.__name__ + '.' + methodname: method._prop})
    return cls


def augment(*args):
    def wrapper(func):
        func._prop = args
        return func
    return wrapper


def concepts(*args):
    def wrapper(func):
        func._used_concepts = set(args)
        return func
    return wrapper

#########
# Utils #
#########

def get_type_hints_without_return(cls, func):
    h = get_type_hints(getattr(cls, func.split(".")[-1]))
    if "return" in h.keys():
        h.pop("return")
    return h.values()

def get_tuple_classes(cls, func):
    type_hints = list(get_type_hints_without_return(cls, func))
    if len(type_hints) > 0:
        dom_cls = type_hints[0]
    else:
        dom_cls = owlready2.Thing
    return cls, dom_cls


def get_triple_classes(cls, func):
    type_hints = list(get_type_hints_without_return(cls, func))
    if len(type_hints) > 0:
        dom_cls_1 = type_hints[0]
        if len(type_hints) > 1:
            dom_cls_2 = type_hints[1]
        else:
            dom_cls_2 = owlready2.Thing
    else:
        dom_cls_1 = owlready2.Thing
        dom_cls_2 = owlready2.Thing
    return cls, dom_cls_1, dom_cls_2


def get_sparql_result(onto, query_file):
    return [tuple(i) for i in onto.world.sparql(query_file.read())]


@lru_cache(maxsize=2048)
def get_search_space(onto, cls, func, sparql_location):
    if type(sparql_location) is str:
        search_space = get_search_space_sparql(onto, cls, func, sparql_location)
    else:
        num_args = len(list(get_type_hints(getattr(cls, func.split(".")[-1])).values())) + 1
        if num_args == 2:
            search_space = get_search_space_type_hints_tuple(onto, cls, func)
        elif num_args == 3:
            search_space = get_search_space_type_hints_triple(onto, cls, func)
        else:
            raise NotImplementedError("Getting augmentation search space by type hints is not supported for functions "
                                      "with more than 2 arguments")
    if logger.level == logging.DEBUG:
        search_space = tqdm.tqdm(search_space)
    return search_space


def get_cls_search_space(onto, cls):
    search_space = onto.search(type=cls)
    if logger.level == logging.DEBUG:
        search_space = tqdm.tqdm(search_space)
    return search_space


def get_search_space_sparql(onto, cls, func, sparql_location):
    try:
        with open(sparql_location, "r") as query_file:
            logger.debug("Executing SPARQL query located in " + str(sparql_location))
            search_space = get_sparql_result(onto, query_file)
            logger.debug("Search space size is " + str(len(search_space)))
            if len(search_space) > 0 and len(search_space[0]) == 2:
                cls, dom_cls = get_tuple_classes(cls, func)
                cls_list = list(onto.search(type=cls))
                dom_cls_list = list(onto.search(type=dom_cls))
                search_space = [x for x in search_space if x[0] in cls_list and x[1] in dom_cls_list]
            elif len(search_space) > 0 and len(search_space[0]) == 3:
                cls, dom_cls_1, dom_cls_2 = get_triple_classes(cls, func)
                cls_list = list(onto.search(type=cls))
                dom_cls_1_list = list(onto.search(type=dom_cls_1))
                dom_cls_2_list = list(onto.search(type=dom_cls_2))
                search_space = [x for x in search_space if x[0] in cls_list and x[1] in dom_cls_1_list and
                                x[2] in dom_cls_2_list]
            logger.debug("Pruned search space size is " + str(len(search_space)))
            return search_space
    except FileNotFoundError:
        logger.warning("Can not find SPARQL query file at " + str(sparql_location))
        return []


def get_search_space_type_hints_tuple(onto, cls, func):
    cls, dom_cls = get_tuple_classes(cls, func)
    return list(itertools.product(onto.search(type=cls), onto.search(type=dom_cls)))


def get_search_space_type_hints_triple(onto, cls, func):
    cls, dom_cls_1, dom_cls_2 = get_triple_classes(cls, func)
    return list(itertools.product(onto.search(type=cls), onto.search(type=dom_cls_1), onto.search(type=dom_cls_2)))

#################################
# Generic augmentation handlers #
#################################
# TODO list:
# - avoid multiple function calls by leveraging the axioms from the ontology:
#    - e.g. functional, transitive, ... for data properties


def _augment_class_subsumption(onto, cls, func, sub_cls=None, sparql=None):
    c = 0
    if not sub_cls:
        sub_cls = cls
    logger.debug("START augmenting " + str(cls) + " (class subsumption) by " + str(func))
    for inst in get_cls_search_space(onto, cls):
        if sub_cls not in getattr(inst, "is_a") and getattr(cls, func.split(".")[-1])(inst):
            getattr(inst, "is_a").append(sub_cls)
            c += 1
    logger.debug("DONE  augmenting " + str(cls) + " (class subsumption) -- " + str(c) + " instances.")
    return c


def _augment_class_equivalence(onto, cls, func, eq_cls=None, sparql=None):
    c = 0
    if not eq_cls:
        eq_cls = cls
    logger.debug("START augmenting " + str(cls) + " (class equivalence) by " + str(func))
    for inst in get_cls_search_space(onto, cls):
        eq = getattr(cls, func.split(".")[-1])(inst)
        if eq_cls not in getattr(inst, "is_a") and eq:
            getattr(inst, "is_a").append(eq_cls)
            c += 1
        elif eq_cls in getattr(inst, "is_a") and not eq:
            getattr(inst, "is_a").remove(eq_cls)
            c += 1
    logger.debug("DONE  augmenting " + str(cls) + " (class equivalence) -- " + str(c) + " instances.")
    return c


def _augment_data_property(onto, cls, func, prop, sparql=None):
    c = 0
    logger.debug("START augmenting " + str(cls) + "." + str(prop) + " (data property) by " + str(func))
    for inst in get_cls_search_space(onto, cls):
        if getattr(inst, prop) is None or getattr(inst, prop) == []:
            val = getattr(cls, func.split(".")[-1])(inst)
            if val is not None:
                setattr(inst, prop, val)
                c += 1
    logger.debug("DONE  augmenting " + str(cls) + "." + str(prop) + " (data property) -- " + str(c) + " instances.")
    return c


def _augment_object_property(onto, cls, func, prop, sparql=None):
    c = 0
    logger.debug("START augmenting " + str(cls) + "." + str(prop) + " (object property) by " + str(func))
    search_space = get_search_space(onto, cls, func, sparql)
    for i in search_space:
        if (i, prop) not in _CREATED_OBJECT_PROPERTIES and i[1] not in getattr(i[0], prop) and \
                getattr(cls, func.split(".")[-1])(*i):
            getattr(i[0], prop).append(i[1])
            _CREATED_OBJECT_PROPERTIES.add((i, prop))
            c += 1
    logger.debug("DONE  augmenting " + str(cls) + "." + str(prop) + " (object property) -- " + str(c) + " instances.")
    return c


def _augment_reified_data_property(onto, cls, func, rei_cls, property_from, property_to, prop, sparql=None):
    c = 0
    rei_insts = set()
    logger.debug("START augmenting " + str(rei_cls) + "." + str(prop) + " (reified data property) by " + str(func))
    search_space = get_search_space(onto, cls, func, sparql)
    for i in search_space:
        val = getattr(cls, func.split(".")[-1])(*i)
        if (i, rei_cls) not in _CREATED_REIFIED_DATA_PROPERTIES and val is not None:
            rei_ins = rei_cls()
            rei_insts.add(rei_ins)
            setattr(rei_ins, property_from, i[0])
            setattr(rei_ins, property_to, i[1])
            setattr(rei_ins, prop, val)
            _CREATED_REIFIED_DATA_PROPERTIES.add((i, rei_cls))
            c += 1
    logger.debug("DONE  augmenting " + str(rei_cls) + "." + str(prop) + " (reified data property) -- " + str(c) +
                 " tuples.")
    return c, rei_insts


def _augment_reified_object_property(onto, cls, func, rei_cls, properties: list, sparql=None):
    # TODO we assume all properties of rei_cls being non-functional -> perform type checking before assignments
    c = 0
    rei_insts = set()
    logger.debug("START augmenting " + str(cls) + " (reified object property) by " + str(func))
    search_space = get_search_space(onto, cls, func, sparql)
    for i in search_space:
        if (i, rei_cls) not in _CREATED_REIFIED_OBJECT_PROPERTIES:
            vals = getattr(cls, func.split(".")[-1])(*i)
            if vals and (not hasattr(vals, "__getitem__") or vals[0]):
                rei_ins = rei_cls()
                rei_insts.add(rei_ins)
                for j in range(min(len(properties), len(i))):
                    getattr(rei_ins, properties[j]).append(i[j])
                if len(properties) > len(i) and hasattr(vals, "__getitem__") and \
                        len(vals) > len(properties) - len(i):
                    vals = vals[1:]
                    for j in range(len(i), len(properties)):
                        getattr(rei_ins, properties[j]).append(vals[j - len(i)])
                _CREATED_REIFIED_OBJECT_PROPERTIES.add((i, rei_cls))
                c += 1
    logger.debug("DONE  augmenting " + str(rei_cls) + " (reified object property) -- " + str(c) + " instances.")
    return c, rei_insts


##############################
# Main augmentation function #
##############################

def do_augmentation(*ontologies: owlready2.Ontology) -> (int, set):
    """
    Performs the augmentation on all given ontologies. Note that there is no automated dependency analysis of the
    augmentations. Therefore, be careful about order of the ontologies (basic ontologies shall come first). The method
    is idempotent.
    :param ontologies: The ontologies whose ABoxes are to be augmented.
    :return: The number of augmentation (changes to the ABox) that were performed and a set of newly created individuals
    (for reification).
    """
    # Store the changes made during the current augmentation.
    changes = 0
    new_individuals = set()
    for onto in ontologies:
        # Clear caches for a fresh start.
        get_search_space.cache_clear()
        # Performs augmentation for all classes within the ontology.
        try:
            onto.classes()
        except TypeError:
            onto = onto.ontology
        for cls in onto.classes():
            # Do not augment subclasses that have inherited the augmentation.
            if hasattr(cls, "_propdict") and cls._propdict and "_propdict" in cls.__dict__:
                # Performs augmentation for each function tagged with the augment decorator.
                # TODO sort the keys by using a dependency graph
                for func in cls._propdict.keys():
                    c = 0
                    if cls._propdict[func] and len(cls._propdict[func]) > 0 and cls._propdict[func][0]:
                        # Handles different augmentation cases by dispatching
                        if cls._propdict[func][0] is AugmentationType.CLASS_SUBSUMPTION:
                            c = _augment_class_subsumption(onto, cls, func,
                                                           cls._propdict[func][1] if len(cls._propdict[func]) > 1
                                                           else None,
                                                           sparql=cls._propdict[func][2] if len(cls._propdict[func]) > 2
                                                           else None)
                        elif cls._propdict[func][0] is AugmentationType.CLASS_EQUIVALENCE:
                            c = _augment_class_equivalence(onto, cls, func,
                                                           cls._propdict[func][1] if len(cls._propdict[func]) > 1
                                                           else None,
                                                           sparql=cls._propdict[func][2] if len(cls._propdict[func]) > 2
                                                           else None)
                        elif cls._propdict[func][0] is AugmentationType.OBJECT_PROPERTY:
                            c = _augment_object_property(onto, cls, func, cls._propdict[func][1],
                                                     cls._propdict[func][2] if len(cls._propdict[func]) > 2 else None)
                        elif cls._propdict[func][0] is AugmentationType.DATA_PROPERTY:
                            c = _augment_data_property(onto, cls, func, cls._propdict[func][1],
                                                   cls._propdict[func][2] if len(cls._propdict[func]) > 2 else None)
                        elif cls._propdict[func][0] is AugmentationType.REIFIED_DATA_PROPERTY:
                            c, insts = _augment_reified_data_property(onto, cls, func, cls._propdict[func][1],
                                                           cls._propdict[func][2],
                                                           cls._propdict[func][3],
                                                           cls._propdict[func][4],
                                                           cls._propdict[func][5] if len(cls._propdict[func]) > 5
                                                                                  else None)
                            new_individuals = new_individuals.union(insts)
                        elif cls._propdict[func][0] is AugmentationType.REIFIED_OBJECT_PROPERTY:
                            c, insts = _augment_reified_object_property(onto, cls, func, cls._propdict[func][1],
                                                                 cls._propdict[func][2],
                                                                 cls._propdict[func][3] if len(cls._propdict[func]) > 3
                                                                 else None)
                            new_individuals = new_individuals.union(insts)
                    changes += c
    return changes, new_individuals


def reset():
    """
    Resets the caching such that the augmentation can be performed again completely from scratch if do_augmentation() is
    called.
    """
    _CREATED_OBJECT_PROPERTIES = dict()
    _CREATED_REIFIED_DATA_PROPERTIES = dict()
    _CREATED_REIFIED_OBJECT_PROPERTIES = dict()
    get_search_space.cache_clear()
