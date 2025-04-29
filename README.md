# Keywords

robotic warehouse, boxes placement optimization, multiagent simulation, ROSS, C

# Goals

- [x] Implementation of discrete multi-agent modeling of robot movement
goods in the warehouse for a given sequence of pallet shipment;

- [x] Development of algorithms for optimizing the placement of goods in a warehouse with a price
we speed up the processing of orders and efficient utilization of space;

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

