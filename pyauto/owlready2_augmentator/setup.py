from distutils.core import setup

setup(
    name="owlready2_augmentator",
    version="0.1",
    description="Augmentator library for owlready2",
    author ="Lukas Westhofen",
    author_email="lukas.westhofen@dlr.de",
    py_modules=["owlready2_augmentator"],
    install_requires=["owlready2"]
)
