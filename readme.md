# ROAR Carla Simlulation

### QuickStart


#### Windows:
1. Clone this repo
    - `git clone https://github.com/augcog/ROAR-Sim.git`
2. Download Carla Server package
    - [https://drive.google.com/drive/folders/1FlkNCNENaC3qrTub7mqra7EH7iSbRNiI](https://drive.google.com/drive/folders/1FlkNCNENaC3qrTub7mqra7EH7iSbRNiI)
    - put it OUTSIDE of the `ROAR-Sim` folder, doesn't matter where
3. Download data file 
    - make a file under ROAR-Sim called `data`
    - Download map waypoint file 
        - [https://drive.google.com/drive/folders/1FlkNCNENaC3qrTub7mqra7EH7iSbRNiI](https://drive.google.com/drive/folders/1FlkNCNENaC3qrTub7mqra7EH7iSbRNiI)
4. Check your file directory, it should be:
    - `ROAR-Sim`
        - `data`
            - `MY_WAYPOINT_FILE.txt`
        - `ROAR_simulation`
        - `runner.py`
        - ... other files and folders
5. Create virtual environment and install dependencies
    - `conda create -n ROAR python=3.7`
    - `conda activate ROAR`
    - `pip install -r requirements.txt`
6. Enjoy
    - Double click the `CarlaUE4.exe` file in the Carla Server package to launch the server
    - `python runner.py`
        
#### Linux:
Same as Windows, in step 6, just type in `./CarlaUE4.sh` to start the server
    