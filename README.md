# Practical sessions on mobile robot behavior programming

This page describes how to set up and run the practical sessions. 

- [Download the practical session archive](https://drive.google.com/file/d/18JQClB_1gdKimCcvs_11YQKSJTf-J198/view?usp=sharing) and save it in `Documents` (if you prefer to use another folder, just replace `Documents` by the path to your folder in the following).
- Once the download is completed, open your file manager, go in `Documents` and extract the archive. This will create a folder called `robotics`. Inside this folder you should have two folders called `pyvrep-epuck` and `pypot` and a file `remoteApiConnections.txt`. You can then delete the archive file (`robotics.zip`).
- Download the [CoppeliaSim simulator](https://www.coppeliarobotics.com/downloads) and install it.
- In the directory where CoppeliaSim has been installed, replace the file `remoteApiConnections.txt` by the one present in your `robotics` folder. This is required to be able to control multiple robots from a Jupyter notebook.
    + If you are on MacOS, you will have to `ctrl-click` on the CoppeliaSim application and choose `Show package content`, then go to `Contents/MacOS` to replace the file. On MacOS you will also have to execute `sudo xattr -r -d com.apple.quarantine *` in a terminal from the `CoppeliaSim_Edu_V4_0_0_Mac` directory (the name of the directory can differ according to the version of CoppeliaSim). 
- Start the CoppeliaSim application. If asked, indicate that you accept incoming network connections (this is necessary for the connection with Jupyter Notebook). 
- Once the simulator is open, launch `jupyter notebook` by opening another terminal and executing:
```
cd Documents/robotics/pyvrep_epuck/notebooks/
jupyter notebook
```
- This will open a web page in the browser with a list of files. Click on the practical session you want to do (`practical_session_1.ipynb` if it is the first class). This will open a document in a new tab in your browser.

The rest of the session is described in this freshly opened document, please continue from there. 
