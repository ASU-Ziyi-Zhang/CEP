# cav-sumo XIL module


## Run XIL-based simulations with SUMO mixed traffic

From the parent **cav_sumo** directory, run the main XIL Python script

    python main_xil.py

This script launches SUMO with a multi-threaded server real-time listening for incoming client messages and embeds them into simulation. Check the options via help command

    python main_xil.py -h

Several options allow a user to specify all the previous simulation-related options. Additionally, the number of external clients, the server port, the server IP address can be specified.

By default the server will listen for all incoming messages from clients at `server_port=12345`.

#### Example client program
To perform SIL testing to check functionality, in a separate terminal, run the vehicle simulation script to launch a client.

    python -m ext.vehicle_sim

Pass `-h` as before to get help options.

## Message types

The XIL interface currently implements `bsm`, `sim`, and `spat` message types. The server that controls SUMO simulations expects to send each message type to a specific port for each unique client IP address that has made connection requests. So a client should listen for messages on the following ports.

    bsm: PORT=10001
    sim: PORT=10004
    spat: PORT=10007

Specific definitions for each message type can be found in the classes defined in **src/messages.py**. Each message has both required and optional fields, and a data type associated with each field.

## XIL Simulation Synchronization

The SUMO server will wait to receive connection requests from <--num_clients> before it attempts to start simulation. The `sim` message type uses an enum for several statuses. 

    class SimStatus(Enum):
        """ Defines possible xil simulation statuses """
        NOT_READY = -1 # Communicating and preparing but not ready yet
        WAITING = 0 # Communicating and ready but waiting for confirmation to start
        RUNNING = 1 # Communicating and ready, has received confirmation to start
        OFFLINE = -2 # Shutdown or will shutdown and will cease communication
        RESET = -3 # Request for the current scenario to be reset, the server program will reset but not need to be re-launched manually

The server will broadcast `WAITING` status once it is waiting for each client to confirm readiness (either a flag of `WAITING` or `RUNNING`). Once each client is ready, the server will begin to periodically publish the `RUNNING` status. Once the simulation naturally ends, or any client requests an `OFFLINE` status, the server will terminate simulation and publish an `OFFLINE` status before performing cleanup and closing the program.

If any client sends a `RESET` status while the server is running, the server will attempt to reset the current XIL simulation. There are a few limitations currently:
* If using the --GUI option, the GUI dialogue will need to be manually closed
* If the microsimulation or the server errors, the testing will need to be manually reset

## Simulink Interface

The **matlab/** folder contains source *.c codes that implement S-functions to serialize and deserialize appropriate `bsm` and `sim` messages. Serialization will convert to `uint8` data types, which can then be sent via UDP messaging to the SUMO server.

Compile the S-functions using `mex <filename>.c`.

* Make sure that the `mex` compiler has been appropriately setup with a C compiler first.

## XIL test scenarios

The scenario can be changed when launching **main_xil.py**. For example,

    python -m main_xil --scenario_folder dev/sumo_scenarios --scenario onramp20

will launch a scenario located under dev/sumo_scenarios that is the onramp scenario but traffic has been changed to follow a 20 mph speed limit.

## XIL test geometry

Currently, each SUMO scenario has a coordinate system as follows
_ _ _ _ _ x
|
|
|
-y

with the origin starting on the beginning of the left-most shoulder. So the `y=-0.5 LANEWIDTH` line is the center of the first lane, `y=-1.5 LANEWIDTH` line is the center of the second lane, etc.

Currently, the scenarios are set up so that the traffic inflow starts at the same time as the XIL simulation. This means that the network is empty when the ego vehicle starts XIL tests. In the 4lane scenario, this reduces interactions significantly, while in the onramp scenario, the onramp can allow for some traffic to enter downstream the ego.

## XIL Analysis

The data output from each test scenario run from a configuration file \<cav_sumo\>/\<scenario_folder\>/\<scenario\>\/*.sumocfg will be output to a \<cav_sumo\>/\<scenario_folder\>/\<scenario\>/\<output_dir\>/ folder.

The results can be plotted using

``python -m analysis_xil --scenario_folder <folder> --scenario <scenario> --output_dir <output_dir>``