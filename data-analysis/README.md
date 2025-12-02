The file carla_mttf_mttr.py takes in the jsonl files from the CARLA runs and calculates failure data. It saves this data to results.json, which is then used in carla_visualizer.py to create the graphs displaying the data.

carla_mttf_mttr.py example usage: \
python carla_mttf_mttr.py camera_blackout_tf_tf_agent.jsonl --waypoint-threshold 1.0 --speed-excess 1.0 --check-interval 5 --output results.json --run-name "tf_tf"

carla_visualizer.py example usage: \
python carla_visualizer.py results.json --output comprehensive_report.png \
python carla_visualizer.py results.json --show
