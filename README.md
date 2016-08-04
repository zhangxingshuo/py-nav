# Robotic Navigation with Monocular Visual Localization and External Odometry
Navigation of a Raspberry Pi robot using monocular vision and Monte Carlo localization.

## Setup
Connect to the Raspberry Pi robot and iPad from the client computer using the code provided in the repository [py-robot](https://github.com/zhangxingshuo/py-robot). Run the scripts to ensure that the connection is established and working. You do not need to keep these running in the background; the `nav.py` script provides connection to both the Pi and the iPad.

The visual map must be constructed and stored in a folder called 'map' inside the root directory. Within map, the different locations must be numbered starting from `0`. 

## Usage
First, if using bag-of-words image retrieval for localization, the dictionaries for the map must be initialized. This is done through the matcher script (see the repository [py-mcl](https://github.com/zhangxingshuo/py-mcl) for specifics on indexes and image matching algorithms). Next, run

`python nav.py`

in CLI. Allow the script a minute to localize the robot. 

To navigate, simply click an arrow in any of the circles, and the robot will move to that location and pivot to face the desired direction.

## Credits
Harvey Mudd College Computer Science REU 2016

Ciante Jones: cjjones@hmc.edu

Chi-Yen Lee: johlee@hmc.edu

Andy Zhang: axzhang@hmc.edu

Professor Zach Dodds: zdodds@hmc.edu
