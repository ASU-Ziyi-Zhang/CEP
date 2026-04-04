
### Run this e.g
# from/folder/cav_sumo> . tools/batch_analysis_xil.sh
# Settings
# scenario_folder=sumo_scenarios
# scenario=onramp # Sumo scenario name

### cav_sumo/<scenario_folder>/scenario/<output_dir>

scenario_folder=tmp/SUMO 

# Run analysis
scenario=4lane20 # Sumo scenario name
output_dir=output_2025-06-26_14-42-23_4lane2nd
python -m analysis_xil --scenario_folder $scenario_folder --scenario $scenario --output_dir $output_dir


scenario=4lane30 # Sumo scenario name
output_dir=output_2025-06-26_14-47-42_4land3rd
python -m analysis_xil --scenario_folder $scenario_folder --scenario $scenario --output_dir $output_dir


scenario=4lane40 # Sumo scenario name
output_dir=output_2025-06-26_14-54-24_4lane4th
python -m analysis_xil --scenario_folder $scenario_folder --scenario $scenario --output_dir $output_dir


scenario=onramp20 # Sumo scenario name
output_dir=output_2025-06-26_15-00-52_onramp20
python -m analysis_xil --scenario_folder $scenario_folder --scenario $scenario --output_dir $output_dir


scenario=onramp30 # Sumo scenario name
output_dir=output_2025-06-26_15-09-13_video_onramp30
python -m analysis_xil --scenario_folder $scenario_folder --scenario $scenario --output_dir $output_dir


scenario=onramp40 # Sumo scenario name
output_dir=output_2025-06-26_15-15-05_onramp40
python -m analysis_xil --scenario_folder $scenario_folder --scenario $scenario --output_dir $output_dir


scenario=onramp30 # Sumo scenario name
output_dir=output_2025-06-26_15-33-27_onramp30
python -m analysis_xil --scenario_folder $scenario_folder --scenario $scenario --output_dir $output_dir


# Single analysis
# scenario=onramp20 # Sumo scenario name
# output_dir=output_2025-06-26_15-00-52_onramp20
# python -m analysis_xil --scenario_folder $scenario_folder --scenario $scenario --output_dir $output_dir
