# Introduction

This python module enables an automatic ABox-augmentation of an OWL ontology as loaded by `owlready2`. 
The augmentations are performed by additional functions that extend the classes created by `owlready2`.
The functions are mapped to the to-be-augmented concepts from the OWL by using function decorators.

# Installation

Call `pip install .` from this folder after cloning this repository.

# Usage

*Note: functionality is only tested rudimentarily.*

## Interface

The augmentation can be triggered by calling `owlready2_augmentator.do_augmentation()`, which takes the to-be-agumented 
ontologies as its argument. If the module has performed an augmentation before, multiple augmentations are avoided
(calling `owlready2_augmentator.do_augmentation()` is hence idempotent). Note that this library only performs slight 
type checks. It is therefore the user's responsibility to ensure correct typing.

## Specifying Augmentations

Annotate every class and function that is to be augmented with the special decorators `@augment_class` and `@augment`.
Firstly, every class of the loaded ontology that is to be augmented shall be decorated with `@augment_class`.

### Example

```python
import owlready2

@augment_class
class My_OWL_Class(owlready2.Thing):
    pass
```

Use the parameterized decorator `@augment(...)` to tag augmentation functions. We allow six different augmentations, 
denoted by the `AugmentationType` enum:

- Class subsumption
- Class equivalence
- Object property
- Data property
- Reified object property
- Reified data property

Every augmentation is based on the to-be-implemented function of the augmented  class. 

## 1. Class subsumption.
Use `@augment` on a function returning a Boolean. For each instance in the ontology, the module will add the augmented 
class to the instance's is_a list iff. the augment function returns True.

### Example

```python
@augmented_class
class My_OWL_Class(owlready2.Thing):
    @augment(AugmentationType.CLASS_SUBSUMPTION)
    def my_aug_func(self):
        return True
```

## 2. Class equivalence.
Use `@augment` on a function returning a Boolean. For each instance in the ontology, the module will add the augmented 
class to the instance's is_a list iff. the augment function returns True and remove the class from the instance's is_a 
list otherwise.

### Example

```python
@augmented_class
class My_OWL_Class(owlready2.Thing):
    @augment(AugmentationType.CLASS_EQUIVALENCE)
    def my_aug_func(self):
        return True
```

## 3. Object property.
Use `@augment` with the name of the object property on a function returning a Boolean and with one additional parameter.
For each instance of the augmented class in the ontology and each instance of the type of the additional parameter 
(Thing if no type is specified), the module will add the object property between them iff. the augment function returns 
True.

### Example

```python
@augmented_class
class My_OWL_Class(owlready2.Thing):
    @augment(AugmentationType.OBJECT_PROPERTY, "my_object_property")
    def my_aug_func(self, other: My_Other_OWL_Class):
        return True
```

## 4. Data property.
Use `@augment` with the name of the data property on a function returning a fitting data value. For each instance of the
augmented class in the ontology that does not have the data property set, the module sets the data property to the value
returned by the augment function. 

### Example

```python
@augmented_class
class My_OWL_Class(owlready2.Thing):
    @augment(AugmentationType.DATA_PROPERTY, "my_data_property")
    def my_aug_func(self):
        return 1 + 2
```

## 5. Reified object property.
Allows storing relations between three entities (by reification). Use `@augment` with the reification class as specified
in owlready2 as well as a list of names of its object properties (e.g. from, over, to) on a function with a 
corresponding number of parameters (representing e.g. over and to). Note that the object properties of the reification 
class have to be given as a list (and are therefore variable in count). For instance of the augmented class in the 
ontology and each instances of the types of the other parameters (Thing if no type is specified), the module will add a 
new instance of the reification class and set the object properties to the respective instances iff. the augment 
function returns True. 

### Example

```python
@augmented_class
class My_OWL_Class(owlready2.Thing):
    @augment(AugmentationType.REIFIED_OBJECT_PROPERTY, My_Reif_Class, ["from_property", "over_property", "to_property"])
    def my_aug_func(self, other_1: My_Other_OWL_Class, other_2: My_Other_OWL_Class):
        return True
```

## 6. Reified data property.
Allows to store data properties that relate two entities (by reification). Use `@augment` with the reification class as 
specified in owlready2 as well as the names of its two object properties (from and to) and the name of the data property
on a function with one additional parameter. For each instance of the augmented class in the ontology and each instance 
of the type of the additional parameter (Thing if no type is specified), the module will add a new instance of the 
reification class and set the from and to properties to the respective instances. It sets the data property to the value
returned by the augment function.

### Example

```python
@augmented_class
class My_OWL_Class(owlready2.Thing):
    @augment(AugmentationType.REIFIED_DATA_PROPERTY, My_Reif_Class, "from_property", "to_property", "data_property")
    def my_aug_func(self, other: My_Other_OWL_Class):
        return 1 + 2
```

## Defining the search space for augmentation

By default, the search space - all possible instances, tuples, triples, etc. that may be annotated - is extracted 
automatically from the function signature by using type hints if present (see examples above). The results are cached 
between calls of the augmentation functions. 

For better performance in some use cases, we also allow the use of SPARQL queries that define the search space. 
Those are then executed on the ontology and the result is used as the search space. 
Since the default approach naively iterates over all combinations of instances, the use of SPARQL queries can 
incorporate domain semantics (e.g. exclude certain combinations) and can(!) therefore lead to performance improvements 
depending on size of the search space (e.g. triples or quadruples over large ABoxes). 
It is the responsibility of the user to check for type consistency. 

### Example

#### Defining the search space via type hints

Use standard python type hints.
```python
@augmented_class
class My_OWL_Class(owlready2.Thing):
    @augment(AugmentationType.OBJECT_PROPERTY, "my_object_property")
    def my_aug_func(self, other: My_Other_OWL_Class):
        return True
```

#### Defining the search space via SPARQL queries

Use the last entry of the decorator's parameters topass a SPARQL file.
```python
@augmented_class
class My_OWL_Class(owlready2.Thing):
    @augment(AugmentationType.OBJECT_PROPERTY, "my_object_property", "my_sparql_search.sparql")
    def my_aug_func(self, other):
        return True
```
with `my_sparql_search.sparql` is a file containing:
```sparql
PREFIX my: <http://purl.org/my/owl#>

SELECT ?c1 
       ?c1
WHERE
  {
    ?p a my:Parent  .
    ?p my:child ?c1 .
    ?p my:child ?c2 .
  }
```
therefore using only those tuples of two children from the same parent.
