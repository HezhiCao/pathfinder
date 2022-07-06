import yaml
from glob import glob
from setuptools import find_packages, setup
from pybind11.setup_helpers import Pybind11Extension, build_ext


with open("config/config.yaml") as yamlfile:
    config = yaml.load(yamlfile, Loader=yaml.FullLoader)

path_finder = config['pathfinder']
print(f'compiling {path_finder} algorithm')

ext_modules = [
    Pybind11Extension(
        "path_finder",
        sorted(glob(f"{path_finder}/src/*.cpp")),
        include_dirs=[f"{path_finder}/include", "/usr/include/eigen3", ],
        extra_compile_args=["-g"],
    )
]

setup(
    name="path_finder",
    version="0.0.1",
    description="A collections of typical pathfinder algorithms",
    author="Hezhi Cao",
    author_email="caohezhi21@mail.ustc.edu.cn",
    # packages=find_packages(),
    cmdclass={"build_ext": build_ext},
    ext_modules=ext_modules,
)
