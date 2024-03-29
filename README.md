# Practical sessions on mobile robot behavior programming

This page describes how to set up and run the practical sessions. 

- [Download the practical session archive](https://drive.google.com/file/d/1gPVdtHJ2wpDviZiHR7Yx4efSs1kaRg3A/view?usp=sharing) and save it in `Documents` (if you prefer to use another folder, just replace `Documents` by the path to your folder in the following).
- Once the download is completed, open your file manager, go in `Documents` and extract the archive. This will create a folder called `robotics`. Inside this folder you should have two folders called `pyvrep-epuck` and `pypot`. You can then delete the archive file (`robotics.zip`).
- For MacOS and Windows: If Jupyter Notebook is not installed on your computer, [install Anaconda](https://www.anaconda.com/).
- Install `CoppeliaSim 4.1.0 Edu`
    - Download the `CoppeliaSim 4.1.0 Edu` version for your OS from https://www.coppeliarobotics.com/previousVersions
    - Install it
        - On Linux you just need to extract the archive and execute `./coppeliaSim.sh` in a terminal
        - On Windows and MacOS, do a normal install
    - Launch CoppeliaSim. The simulator will open and close just after. This is due to a previously [imposed limit on the runtime of Edu versions](https://forum.coppeliarobotics.com/viewtopic.php?f=9&t=9334). To allow this version to run you need to:
        - On Windows
            - Go to `C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\system`
        - On Mac
            - Right-click on `coppeliaSim` in the `Application` folder, then `Show Package content`. Then go to `Content/MacOS/system`.
        - On Linux
            - On Linux, go to the `system` folder in the extracted folder.
        - Then, open the file `usrset.txt`
        - Add a line at the end of this file with `allowOldEduRelease=7501`
    - Then relaunch CoppeliaSim. It should now open normally.
- If asked, indicate that you accept incoming network connections (this is necessary for the connection with Jupyter Notebook). 
- Once the simulator is open, launch `jupyter notebook` by opening another terminal (on Windows: open Anaconda Prompt instead) and executing:
```
cd Documents/robotics/pyvrep_epuck/notebooks/
jupyter notebook
```
- This will open a web page in the browser with a list of files. Click on the practical session you want to do (`practical_session_1.ipynb` if it is the first class). This will open a document in a new tab in your browser.

The rest of the session is described in this newly opened document, please continue from there. 
