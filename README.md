# Drake Concepts Tutorial

This repo contains tutorial info for Drake (see http://drake.mit.edu).

## PREREQS

You'll have to install Drake (from binaries or source, your choice) following the instructions on the Drake website, or in [this guide](https://docs.google.com/document/d/16gUlJtwtPeNNLs7vk6IbuXXYKyJTdhoEt8BnXbWg52Y/edit?usp=sharing).

You'll also need to use [jupyter](http://jupyter-notebook-beginner-guide.readthedocs.io/en/latest/what_is_jupyter.html) to view the notebook (.ipynb) files, and [meshcat-python](https://github.com/rdeits/meshcat-python) to view the 3D visualizations in some of the examples. You can install both with

```
pip install jupyter meshcat
```

## USE

To view the notebook files, use the command `jupyter notebook` from a terminal in the same directory (or a parent directory) as the notebook files. Then use the web-browser-based notebook browser to open up the notebook files, and use the notebook interface to play with the notebook. See a [guide like this one](http://jupyter-notebook-beginner-guide.readthedocs.io/en/latest/what_is_jupyter.html) for info on how to use the Jupyter notebook. Be sure to [have Drake on your PYTHONPATH](http://drake.mit.edu/python_bindings.html) before launching jupyter!

To run the Kuka simulation, first run `meshcat-server` in a new terminal. It should report a web-url -- something like `127.0.0.1:7000/static/`. Open that in a browser -- this is your 3D viewer. Then run `python kuka_pydrake_sim.py` and you should see the arm spawn in the viewer before doing some movements.
