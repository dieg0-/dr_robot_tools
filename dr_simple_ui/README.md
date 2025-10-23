# dr_simple_ui

This package provides a simple web-based GUI though for basic teleoperation of a turtlebot3, although it can easily be used for any other robot-type. The GUI is based on [NiceGUI](https://nicegui.io/) and ROS2.

## Prepare

- Create a virtual environment for the required Python dependencies, for example:
  - ``python3 -m venv ui_venv``
  - ``source ui_venv/bin/activate``
  - ``pip install -r ~/path_to_your_package/requirements.txt``
  - ``deactivate``

- Build the package
  - ``cd ~/path_to_your_workspace``
  - ``colcon build --packages-select dr_simple_ui --symlink-install``

- Make sure sourcing is done in order, for example:
  - ``source ui_venv/bin/activate``
  - ``source /opt/ros/jazzy/setup.bash``
  - ``source instal/setup.bash``

- If the venv is not part of your **PYTHONPATH**, add it accordingly, for example:
  - ``PYTHONPATH=~/ui_venv/lib/python3.12/site-packages/:${PYTHONPATH}``
  - ``export PYTHONPATH``

## Usage

- Run the UI:
  - ``ros2 run dr_simple_ui ui``
- Open it on a browser on any device on the same network