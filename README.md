# ctd_to_seawater_density

This repository contains a simple Python 3 simulator (**driftcam_simulator.py**) for the Driftcam micro-ballast dispenser. These are the inputs employed for the simulations:

    - Density profile (also fixed density option available)
    - Drag coefficient
    - Ballast mass
    - Ballast dispensing rate
    - Driftcam mass
    - Driftcam volume

It does include a simple PID depth controller for testing purposes. The controller gains can be modified using the conrresponding configuration.yaml file. 

## Installation instructions
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

Finally, Panda is being used to import the CTD and seafloor model profiles, and also to export resulting simulations to CSV files. In order to install this library, please use:

    pip install panda
    
If preferred, a package management system such as Conda can be used. Please refer to https://conda.io/en/latest/ 

## Using driftcam_simulator

An example of **driftcam_simulator** usage is:

    python driftcam_simulator.py --ctd data/ctd/interpolated/CTD02.csv --ballast 0.014 --transect data/transects/T1.csv  --output desired_output_name.txt
    
This example will read the CTD profile contained in __data/ctd/interpolated/CTD02.csv__, force a ballast diameter value of 14 mm (0.014 m) and import the seafloor depth model from the transect defined in __data/transects/T1.csv__. The resulting simulations will be stored in the __desired_output_name.txt__ file. 

To see information about how to call the script, please run it without any argument:

    python driftcam_simulations.py

Future implementations may support argument parsing as:

    python driftcam_simulations.py --parse data/density_profile --config config_file.yaml
    

