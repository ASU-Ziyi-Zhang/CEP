#! bin/sh

### Helper shell script to run a batch of microsimulations

### Settings
is_plotting=1 # [0/1] 1 to plot results already in data folder by pen_array - 0 to run simulations from pen_array

pen_array=(10 20 30) # 15 20 25 30 35 40 45 50) # Array for CAV penetration cases to run/plot

scenario_folder=sumo_scenarios
scenario=onramp # Sumo scenario name

### Iterate
for pen in "${pen_array[@]}"; do
    if [ $is_plotting -ne 1 ]; then # Not plotting so Run simulations
        echo "Running SUMO simulation with CAV penetration rate 0.$pen"

        # Run simulation
        python main.py --scenario_folder $scenario_folder --scenario $scenario --penetration 0.$pen

        # Save data
        cp $scenario_folder/$scenario/output/fcd.xml $scenario_folder/$scenario/output/fcd_$pen.xml # Most recent file is fcd.xml - copy to a permanent file by penetration rate

    else # Plotting
        echo ""
        echo "Plotting SUMO simulation with CAV penetration rate 0.$pen"

        # Plot
        python analysis.py --scenario_folder $scenario_folder --scenario $scenario --file fcd_$pen.xml

        # Save data
        cp figures/traffic_metrics.png figures/traffic_metrics_$pen.png
    fi
done