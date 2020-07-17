# ROAR Carla Simulation

## QuickStart
**Required Equipment:** Linux/Windows Computer
    
**Approximate Time:** 15 minutes

0. Download the this code [here](https://drive.google.com/file/d/12vOc6ukmZjkahjXZCWq-hy0iDBiEuEZq/view?usp=sharing) [as of writing this readme, we still need to create a public github repo]
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
        - To switch to autonomous mode, open up `carla_runner.py` and scroll down the end, change the line `settings.enable_aut
        

### Common Errors
1. Conda not found 
    - You should download miniconda3 Linux/Windows 64-bit [miniconda3](https://docs.conda.io/en/latest/miniconda.html)
    - Follow the below instructions to install miniconda successfully 
    ![](miniconda3.png)
    - If still cannot call conda, try (directory may vary):
        - `sudo chown -R /home/username/miniconda3/'`
        - `sudo chmod -R +x /home/username/miniconda3/`
