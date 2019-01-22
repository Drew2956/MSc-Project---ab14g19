# ctd_to_seawater_density

This repository contains a simple script based simulator for the Driftcam microballast dispenser. It uses existing vertical density profiles as input. The other input parameters are:

    - Drag coefficient
    - Ballast mass
    - Ballast dispensing rate
    - Driftcam mass
    - Driftcam volume

It does include a simple PID depth controller for testing purposes. The controller gains can be empirically adjusted for an improved behaviour. 

## How to use this code
*Note: before working or modifying this repository, please read the Ocean Perception Lab naming [conventions](https://github.com/ocean-perception/conventions).*

To install this repository, go to the directory you want it to be installed in and in a terminal/command prompt, type

    git clone https://github.com/ocean-perception/driftcam_simulations
    
To add changes, commit and push to a branch (usually master):

    git add -A
    git commit -m "Detailed message about the change"
    git push origin master

## Dependencies
Remember to check that you have a fully operative Python environment. Current version employs *plotly* for visualization purposes. It can be installed using *pip*

    pip install plotly

It will also need PyYAML parser library in order to read the configuration yaml files

    pip install pyyaml

## Testing ctd_to_seawater_density

To test if ctd_to_seawater_density is working

    cd driftcam_simulations/
    
Then run the Python script with the path to a sample data, using the configuration defined by config_file.yaml

    python driftcam_simulations.py 

Future implementations may support argument parsing as:

    python driftcam_simulations.py --parse data/density_profile --config config_file.yaml
    

