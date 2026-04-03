#! bin/sh

### Helper shell script to run a replay microsimulation

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

## Run replay microsimulation for a given output folder
## <sumo_scenario_folder>/<sumo_scenario>/output should contain the fcd.xml file to replay
scenario_folder="dev/sumo_scenarios" # Directory for the FCD file
scenario="roosevelt_simple" # Folder for the FCD file, should be in the format <sumo_scenario_folder>/<sumo_scenario>/output
replay_dir="output/" # Directory for the FCD file, should be in the format <sumo_scenario_folder>/<sumo_scenario>/<output>
replay_file="fcd.xml" # Name of the FCD file to replay, should be in the output folder of the simulation run
penetration=30 # CAV penetration rate for the simulation run that generated the FCD file, used for readding the vehicles

## Run replay microsimulation
info "Running replay microsimulation for scenario ${scenario} with FCD file ${replay_file}"

python main.py --replay --scenario_folder $scenario_folder --scenario $scenario --replay_dir $replay_dir --replay_file $replay_file --penetration $penetration --gui --realtime --no_inflow