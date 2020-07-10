# ROAR Carla Simlulation

### QuickStart
1. Download the right distribution of Carla for your Operation System [here](https://drive.google.com/drive/folders/1xGz2r6fiXCHn57_ZOS710RX2IoCKzWWx)
    - You should download either `roar_easy_linux.zip` or `roar_easy_windows.zip`
2. Initiate a virtual environment.
    - `conda create -n ROAR python=3.7`
    - `conda activate ROAR`
3. Install required packages
    - `pip install -r requirements.txt`
4. You are all set up.
    - Start the server
        - on Windows, you may double click the `CarlaUE4.exe` or you may use a commandline prompt and type in `CarlaUE4.exe`
        - on Ubuntu, you can directly type `./CarlaUE4.sh`
    - Start Client connection
        - `python carla_runner.py`
        - You should be on manual control now. 
        - To switch to autonomous mode, open up `carla_runner.py` and scroll down the end, change the line `settings.enable_autopilot = False` to `settings.enable_autopilot = True`
    
    
