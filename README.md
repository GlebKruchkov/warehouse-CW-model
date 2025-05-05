# Keywords

robotic warehouse, boxes placement optimization, multiagent simulation, ROSS, C

# Goals

- [x] Implementation of discrete multi-agent modeling of robot movement
goods in the warehouse for a given sequence of pallet shipment;

- [x] Development of algorithms for optimizing the placement of goods in a warehouse with a price
we speed up the processing of orders and efficient utilization of space;

# Model description

This is a model of an innovative automated warehouse with a potentially higher storage density compared to any existing systems, and with a productivity comparable to the best existing automated systems.

High storage density is achieved through deep storage channels. Compared to most existing systems where storage racks contain only one box per unit length (or a small number—2 or 3—of identical boxes), in the presented model, a unit length of racks consists of a deep channel containing many boxes, potentially of different products. Boxes are put in storage in one end of a channel, and retrieved from another end. Thus, each channel forms a queue of boxes, and only the first box in this queue is available for selection for fulfillment.

Robots move around the warehouse according to Dijkstra’s algorithm, while delivering boxes of orders (TEST3-SIMSIM). Robots cannot overtake each other.

During the simulation, robots exchange messages, receiving information about the state of other robots and further targeted actions.

# Current model configuration

All the input data of the model is presented in the `model_input` folder.

`model_input/TEST3-SIMSIM` - еntrance orders that are received by robots.

`model_input/bots_starting_positions.csv` - the starting positions of the robots at the start of the simulation.

`model_input/config.h` - all other input parameters (the algorithm was tested using the simulation method for the warehouse described in the table for N=15 robots and G=50 of various types of boxes (SKU) placed in the warehouse; in the warehouse there are 8 boxes in each channel, and 10 channels in each section. There are a total of 15 sections in the warehouse (5 for each level, 3 different levels. The total capacity of the warehouse is 8·10·15 = 1200 boxes).

All the output data of the model is presented in the `model_output` folder.

`model_output/full_actions_log.txt` - a complete log file with all the simulation events.

`model_output/sku_mileage.txt` - mileage of each type of box (SKU).

# Installation and usage

This model can be built by ROSS by sym-linking it into the ROSS/models directory and building with `-DROSS_BUILD_MODELS=ON`

``` shell
git clone https://github.com/ROSS-org/ROSS
git clone https://github.com/ROSS-org/warehouse-CW-model
cd ROSS/models
ln -s ../../warehouse-CW-model ./
cd ../
mkdir build
cd build
cmake ../ -DROSS_BUILD_MODELS=ON
make
./model
```

All information about the progress of the simulation will be recorded in the log (full_actions_log.txt)

