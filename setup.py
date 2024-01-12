from setuptools import setup, find_packages

setup(
    name="tobm",
    version="0.1",
    description="Temporal ontology benchmark based on A.U.T.O. and pyauto",
    package_dir={"": "src"},
    packages=find_packages(where="src"),
    author="Lukas Westhofen",
    author_email="lukas.westhofen@dlr.de",
    include_package_data=True,
    install_requires=[
        "numpy",
        "sympy",
        "owlready2==0.40",
        "pyauto"
    ],
    entry_points={
        'console_scripts': [
            'tobm = tobm.generate:main',
        ],
    },
)
