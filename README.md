# ai-racing
The programs associated with the development of drone racing powered my Microsoft's AirSim Drone Racing Lab

# Setup Instructions for Airismdroneracinglab environment:
1.	Download the “ADRL.zip” and “settings.json” files from the link below.                                                                
https://github.com/microsoft/AirSim-Drone-Racing-Lab/releases

3.	Extract “ADRL.zip” in an easily accessible folder and run the ADRL.exe file inside. While this will open a drone flight environment, we are just running it now to generate some files. If Windows Defender attempts to block the program, click “more info” and “run anyways.” If prompted with a window asking if you would like to do car physics simulation, select “No.” Once finished, close the window.

5.	Open a command-line interface inside a convenient folder. You will need to navigate to this folder somewhat frequently. Next, clone the ai-racing repository by running the following command. (This requires that you have git installed)
git clone https://github.com/MissouriMRR/ai-racing.git

7.	Future steps require Python to be installed (The download can be found online). Open the command line and run the following commands to install dependencies.
pip install airsimdroneracinglab
pip install backports.ssl_match_hostname

Python scripts can now be run interfacing with the drone using the airsimdroneracinglab API package.
Auto-documentation on the airsimdroneracinglab package can be found here:
https://microsoft.github.io/AirSim-Drone-Racing-Lab/

More information on the Airsim Drone Racing Lab can be found here:
https://github.com/microsoft/AirSim-Drone-Racing-Lab#quickstart
