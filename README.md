## zoo

This repository contains configuration packages of various quadrupedal robots generated by CHAMP's [setup assistant](https://github.com/chvmp/champ_setup_assistant).

## Installation

You need to have [CHAMP](https://github.com/chvmp/champ) installed in your machine to make these robots walk.

cd into your workspace src directory then:

git clone https://github.com/Thomahawkuru/champ_spot.git
cd champ_spot
git submodule init
git submodule update

### Quick Start Guide

You can find the pre-configured robots in [configs](https://github.com/chvmp/robots/tree/master/configs) folder. There's an auto-generated README in every configuration package that contains instructions on how to run the demos.

Please take note that although the README may contain instructions how to run in Gazebo, only the following pre-configured robots work in Gazebo:

- Boston Dynamic's Spot

## Credits

The URDFs found in this repository have been forked/modified/linked from the following projects:

- [Boston Dynamic's Spot](https://github.com/clearpathrobotics/spot_ros)
