# Drake Concepts Tutorial

[![Build Status](https://travis-ci.org/gizatt/drake_periscope_tutorial.svg?branch=master)](https://travis-ci.org/gizatt/drake_periscope_tutorial)

*(Build tested both on [Drake binaries from 20180604](https://drake-packages.csail.mit.edu/drake/nightly/drake-20180604-xenial.tar.gz) and [latest Drake binaries](https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-xenial.tar.gz)).*

This repo contains tutorial info for Drake (see http://drake.mit.edu). It's a sort of code equivalent or companion to [this guide](https://docs.google.com/document/d/16gUlJtwtPeNNLs7vk6IbuXXYKyJTdhoEt8BnXbWg52Y/edit?usp=sharing).

## PREREQS

You'll have to install Drake (from binaries or source, your choice) following the instructions on the Drake website, or in [this guide](https://docs.google.com/document/d/16gUlJtwtPeNNLs7vk6IbuXXYKyJTdhoEt8BnXbWg52Y/edit?usp=sharing).

You'll also need to use [jupyter](http://jupyter-notebook-beginner-guide.readthedocs.io/en/latest/what_is_jupyter.html) to view the notebook (.ipynb) files, [graphviz](https://pypi.org/project/graphviz/), and [meshcat-python](https://github.com/rdeits/meshcat-python) to view the 3D visualizations in some of the examples, and SCAD, pyglet, and trimesh for mesh generation. You'll also need some more standard libraries (e.g. numpy). You can install all of these with

```
apt-get install graphviz openscad
pip install jupyter graphviz meshcat numpy matplotlib trimesh
```

And finally, you'll need to have the [Drake textbook example code](https://github.com/RussTedrake/underactuated) available and on your PYTHONPATH. You can pull it down with

```
git clone https://github.com/RussTedrake/underactuated ~/underactuated
```

and add it to your PYTHONPATH with

```
export PYTHONPATH=~/underactuated/src:$PYTHONPATH
```

(you probably want to add that to the end of your `~/.bashrc`).

## USE

To view the notebook files, use the command `jupyter notebook` from a terminal in the same directory (or a parent directory) as the notebook files. Then use the web-browser-based notebook browser to open up the notebook files, and use the notebook interface to play with the notebook. See a [guide like this one](http://jupyter-notebook-beginner-guide.readthedocs.io/en/latest/what_is_jupyter.html) for info on how to use the Jupyter notebook. Be sure to have both [Drake](http://drake.mit.edu/python_bindings.html) and the textbook code on your PYTHONPATH before launching jupyter!

To run the Kuka simulation, first run `meshcat-server` in a new terminal. It should report a web-url -- something like `127.0.0.1:7000/static/`. Open that in a browser -- this is your 3D viewer. Then run `python kuka_pydrake_sim.py` and you should see the arm spawn in the viewer before doing some movements.
