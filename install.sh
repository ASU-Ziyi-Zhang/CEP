#!/bin/sh

## https://sumo.dlr.de/docs/Downloads.php
## https://docs.python.org/3/library/venv.html

# ------------------
## ANSI ESCAPE CODES
# ------------------
NOCOLOR='\033[m'
BLACK='\033[0;30m'
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'

info() {
    echo -e "${BLUE}[INFO] $1${NOCOLOR}"
}

success() {
    echo -e "${GREEN}[OK] $1${NOCOLOR}"
}

warn() {
    echo -e "${YELLOW}[WARN] $1${NOCOLOR}"
}

error() {
    echo -e "${RED}[ERROR] $1${NOCOLOR}"
}

## Check that python and pip are installed 
COMMAND=python # Default command to try
if [ -x "$(command -v python)" ]; then
    COMMAND=python

elif [ -x "$(command -v python3)" ]; then
    COMMAND=python3

elif [ -x "$(command -v py)" ]; then
    COMMAND=py

else
    error "Could not find Python installed in path. Exiting."
    return 1 || exit 1

fi

# PIP
if [ -x "$(command -v pip)" ]; then
    info "Found pip installed in path. Using pip to install sumo and dependencies."

else
    error "Could not find pip installed in path. Exiting."
    return 1 || exit 1
    
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
        info "Linux GNU detected."
        . bin/activate

    elif [[ "$OSTYPE" == "darwin"* ]]; then
        info "Mac OSX detected."
        . bin/activate

    elif [[ "$OSTYPE" == "cygwin" ]]; then
        info "POSIX/Linux for Windows detected."
        Scripts\Activate.ps1

    else

        error "Could not determine OS type. "
        error "Manually activate the virtual environment through script bin/activate."
        error "Manually install sumo to virtual environment through pip:"
        error "    pip install eclipse-sumo"
        error "    pip install traci"
        error "    pip install libsumo"
        
        return 1 || exit 1
    fi
    
    pip install -q eclipse-sumo
    pip install -q traci
    pip install -q libsumo

    pip install -q matplotlib
    pip install -q numpy
    pip install -q pandas
    pip install -q scipy
    
else
    pip install -q eclipse-sumo
    pip install -q traci
    pip install -q libsumo

    pip install -q matplotlib
    pip install -q numpy
    pip install -q pandas
    pip install -q scipy

fi

## Codegen
cd codegen
. install.sh
cd ..

## Done
success "Installation complete."