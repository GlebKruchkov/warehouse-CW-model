# Summary

Models for a innovative fully robotic warehouse for storing boxed goods.
A discrete multiagent simulation of the movement of shuttles in a warehouse for a given sequence of pallet shipments has been implemented. Different strategies for placement of boxes in various areas of a warehouse are evaluated, as well as optimal routing patterns for shuttles for a given topology of a warehouse. Also estimated is the optimal number of shuttles that maximizes the warehouse productivity.

# Keywords

robotic warehouse, boxes placement optimization, multiagent simulation, ROSS, C

# Usage

If you are creating your own model feel free to fork this repository.
As you develop, please replace "model" with appropriately descriptive names for your variables, files, and functions.

# Installation

This model can be built by ROSS by sym-linking it into the ROSS/models directory and building with `-DROSS_BUILD_MODELS=ON`

``` shell
git clone https://github.com/ROSS-org/ROSS
git clone https://github.com/ROSS-org/template-model
cd ROSS/models
ln -s ../../template-model ./
cd ../
mkdir build
cmake ../ -DROSS_BUILD_MODELS=ON
make
```
