import os
from glob import glob

from setuptools import setup

PKG = "jbj_sim"


def tree(src, dst):
    out = []
    for root, _, files in os.walk(src):
        if files:
            out.append((os.path.join(dst, os.path.relpath(root, src)), [os.path.join(root, f) for f in files]))
    return out


setup(
    name=PKG,
    version="0.1.0",
    packages=[PKG],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PKG]),
        ("share/" + PKG, ["package.xml"]),
        ("share/" + PKG + "/launch", glob("launch/*.py")),
        ("share/" + PKG + "/worlds", glob("worlds/*.sdf")),
    ] + tree("models", "share/" + PKG + "/models"),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Sumith Shridhar",
    maintainer_email="sumithshridhar@users.noreply.github.com",
    description="Autonomous tractor simulation for JBJ Machinery (MF tractors + retrofit kit)",
    license="All rights reserved",
    entry_points={"console_scripts": [
        "gnss_receiver = jbj_sim.gnss_receiver:main",
        "path_follower = jbj_sim.path_follower:main",
        "safety_supervisor = jbj_sim.safety_supervisor:main",
        "vehicle_interface = jbj_sim.vehicle_interface:main",
        "scenario_runner = jbj_sim.scenario_runner:main",
    ]},
)
