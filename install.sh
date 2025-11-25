#!/bin/sh

## https://sumo.dlr.de/docs/Downloads.php
## https://docs.python.org/3/library/venv.html

## Check that python and pip are installed 
COMMAND=python # Default command to try
if [ -x "$(command -v python)" ]; then
    COMMAND=python

elif [ -x "$(command -v python3)" ]; then
    COMMAND=python3

elif [ -x "$(command -v py)" ]; then
    COMMAND=py

else
    echo "Could not find Python installed in path. Exiting."
    exit 1

fi

# PIP
if [ -x "$(command -v pip)" ]; then
    echo "pip found"

else
    echo "Could not find pip installed in path. Exiting."
    exit 1
    
fi

## Install SUMO
# Settings
USING_VENV=0 # 1 to install to virtual environment, 0 to install sumo globally

# Install
if [ $USING_VENV -eq 1 ]; then
    $COMMAND -m venv sumo_env
    cd sumo_env
    
    # Run the activate script
    if [[ "$OSTYPE" == "linux-gnu"* ]]; then
        echo "Linux GNU detected."
        . bin/activate

    elif [[ "$OSTYPE" == "darwin"* ]]; then
        echo "Mac OSX detected."
        . bin/activate

    elif [[ "$OSTYPE" == "cygwin" ]]; then
        echo "POSIX/Linux for Windows detected."
        Scripts\Activate.ps1

    else
        echo "Could not determine OS type. "
        echo "Manually activate the virutal environment through script bin/activate."
        echo "Manually install sumo to virtual environemtn through pip:"
        echo "    pip install eclipse-sumo"
        echo "    pip install traci"
        echo "    pip install libsumo"
        
        exit 1
    fi
    
    pip install eclipse-sumo
    pip install traci
    pip install libsumo

    pip install matplotlib
    pip install numpy
    pip install pandas
    pip install scipy
    
else
    pip install eclipse-sumo
    pip install traci
    pip install libsumo

    pip install matplotlib
    pip install numpy
    pip install pandas
    pip install scipy

fi

## Codegen
cd codegen
. install.sh
cd ..