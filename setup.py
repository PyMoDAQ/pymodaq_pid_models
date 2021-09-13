from setuptools import setup, find_packages
import toml

config = toml.load('./plugin_info.toml')
SHORT_MODEL_NAME = config['plugin-info']['SHORT_PLUGIN_NAME']
MODEL_NAME = f"pymodaq_pid_models"


from pathlib import Path

with open(str(Path(__file__).parent.joinpath(f'src/{MODEL_NAME}/VERSION')), 'r') as fvers:
    version = fvers.read().strip()


with open('README.rst') as fd:
    long_description = fd.read()

setupOpts = dict(
    name=MODEL_NAME,
    description=config['plugin-info']['description'],
    long_description=long_description,
    license=config['plugin-info']['license'],
    url=config['plugin-info']['package-url'],
    author=config['plugin-info']['author'],
    author_email=config['plugin-info']['author-email'],
    classifiers=[
        "Programming Language :: Python :: 3",
        "Development Status :: 5 - Production/Stable",
        "Environment :: Other Environment",
        "Intended Audience :: Science/Research",
        "Topic :: Scientific/Engineering :: Human Machine Interfaces",
        "Topic :: Scientific/Engineering :: Visualization",
        "License :: CeCILL-B Free Software License Agreement (CECILL-B)",
        "Operating System :: OS Independent",
        "Topic :: Software Development :: Libraries :: Python Modules",
        "Topic :: Software Development :: User Interfaces",
    ], )


setup(
    version=version,
    packages=find_packages(where='./src'),
    package_dir={'': 'src'},
    include_package_data=True,
    entry_points={'pymodaq.pid_models': f"{SHORT_MODEL_NAME} = {MODEL_NAME}"},
    install_requires=['toml', ]+config['plugin-install']['packages-required'],
    **setupOpts
)


