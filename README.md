# Drake Concepts Tutorial

[![Build Status](https://travis-ci.org/gizatt/drake_periscope_tutorial.svg?branch=master)](https://travis-ci.org/gizatt/drake_periscope_tutorial)

*(Build tested on [Drake binaries from 20180604](https://drake-packages.csail.mit.edu/drake/nightly/drake-20180604-xenial.tar.gz).)*

*This tutorial is now vastly out of date. See the Deprecation Information section for details of what you can trust here, and where else you should be looking.*

This repo contains tutorial info for Drake (see http://drake.mit.edu). It's a sort of code equivalent or companion to [this guide](https://docs.google.com/document/d/16gUlJtwtPeNNLs7vk6IbuXXYKyJTdhoEt8BnXbWg52Y/edit?usp=sharing).

## DEPRECATION INFORMATION

You can hopefully trust this repo to keep working, as it's under CI against pegged (i.e. old!) Drake + support-code versions. The system block workflow and system architecture for a simple manipulation system is still relevant and informative to peruse. However, a few core underlying things have changed:

- Modern Drake has transitioned from RigidBodyTree to MultiBodyPlant. The interfaces are similar, but MBP is the cleaner/faster/fancier "modern" version.
- Misc API in this repo has changed or gone out of date, especially as pertains to Abstract-valued input and State, but probably also lots of other little things.
- Many conveniences have been introduced in Drake that replace large chunks of the code here (like the Kuka controller code here).

To see a more modern example of this kind of system, you can check out *(updated Apr 10, 2019)*:
- [MIT's 6.881 coursework](https://manipulation.csail.mit.edu/) -- everything needed to follow along with the assignments, which are all based around a simulation of a Kuka arm doing manipulation tasks, should be public.
- The Drake [ManipulationStation](https://github.com/RobotLocomotion/drake/tree/master/examples/manipulation_station) example, which is the actively maintained kernel of what supported 6.881. The Python example files can be run as long as Drake is installed on your system (and is on your PYTHONPATH). The C++ examples can be run if you build Drake from source. Instructions for doing both of those things are [here](https://drake.mit.edu/installation.html).


## PREREQS

You'll have to install Drake (from binaries or source, your choice) following the instructions on the Drake website, or in [this guide](https://docs.google.com/document/d/16gUlJtwtPeNNLs7vk6IbuXXYKyJTdhoEt8BnXbWg52Y/edit?usp=sharing). Note that this does *not* work on recent Drake versions any more; for the best experience, use the [Drake binaries from 20180604](https://drake-packages.csail.mit.edu/drake/nightly/drake-20180604-xenial.tar.gz). For a codebase similar to this maintained against up-to-date Drake revision, see [this repo](https://github.com/gizatt/pydrake_kuka) or, better yet, the [codebase maintained by MIT's 6.881 course](https://github.com/RobotLocomotion/6-881-examples).

You'll also need to use [jupyter](http://jupyter-notebook-beginner-guide.readthedocs.io/en/latest/what_is_jupyter.html) to view the notebook (.ipynb) files, [graphviz](https://pypi.org/project/graphviz/), and [meshcat-python](https://github.com/rdeits/meshcat-python) to view the 3D visualizations in some of the examples. You'll also need some more standard libraries (e.g. numpy). You can install all of these with

```
apt-get install graphviz
pip install jupyter graphviz meshcat numpy matplotlib
```

And finally, you'll need to have the [Drake textbook example code](https://github.com/RussTedrake/underactuated) available and on your PYTHONPATH. Due to some deprecations, you'll need to checkout an old-ish version -- you can pull it down with

```
git clone https://github.com/RussTedrake/underactuated ~/underactuated && cd ~/underactuated && git checkout 17687cb52ff8febd77a8f881729317dff3ee8c67
```

and add it to your PYTHONPATH with

```
export PYTHONPATH=~/underactuated/src:$PYTHONPATH
```

(you probably want to add that to the end of your `~/.bashrc`).

## USE

To view the notebook files, use the command `jupyter notebook` from a terminal in the same directory (or a parent directory) as the notebook files. Then use the web-browser-based notebook browser to open up the notebook files, and use the notebook interface to play with the notebook. See a [guide like this one](http://jupyter-notebook-beginner-guide.readthedocs.io/en/latest/what_is_jupyter.html) for info on how to use the Jupyter notebook. Be sure to have both [Drake](http://drake.mit.edu/python_bindings.html) and the textbook code on your PYTHONPATH before launching jupyter!

To run the Kuka simulation, first run `meshcat-server` in a new terminal. It should report a web-url -- something like `127.0.0.1:7000/static/`. Open that in a browser -- this is your 3D viewer. Then run `python kuka_pydrake_sim.py` and you should see the arm spawn in the viewer before doing some movements.
