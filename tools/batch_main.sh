#! bin/sh

### Helper shell script to run a batch of microsimulations

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

### ------------------
### Settings
### ------------------
is_plotting=0 # [0/1] 1 to plot results already in data folder by pen_array - 0 to run simulations from pen_array

pen_array=(0 10 20 30 40) # 15 20 25 30 35 40 45 50) # Array for CAV penetration cases to run/plot

# Structure should be <cd>/<scenario_folder>/<scenario>
# Output files should be <cd>/<scenario_folder>/<scenario>/output
# Log files will be saved in <cd>/logs/<timestamp>/<penetration> with veh_log_data.csv and sim_log_data.csv
scenario_folder=dev/sumo_scenarios
scenario=roosevelt_simple # Sumo scenario name

### ------------------
### Iterate
### ------------------
for pen in "${pen_array[@]}"; do
    pen_rate=0.$pen

    if [ $pen -ge 100 ]; then
        pen_rate=1
    fi
    
    if [ $is_plotting -ne 1 ]; then # Not plotting so Run simulations
        info "Running SUMO simulation with CAV penetration rate $pen_rate"

        # Run simulation
        python main.py --scenario_folder $scenario_folder --scenario $scenario --penetration $pen_rate --log_tl

        # Save data
        cp $scenario_folder/$scenario/output/fcd.xml $scenario_folder/$scenario/output/fcd_$pen.xml # Most recent file is fcd.xml - copy to a permanent file by penetration rate

    else # Plotting
        info "Plotting SUMO simulation with CAV penetration rate $pen_rate"

        # Plot
        python analysis.py --scenario_folder $scenario_folder --scenario $scenario --file fcd_$pen.xml

        # Save data
        cp figures/traffic_metrics.png figures/traffic_metrics_$pen.png
    fi
done

### ------------------
### Done
### ------------------
success "Done."