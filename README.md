# Practical sessions on mobile robot behavior programming

This page describes how to set up and run the practical sessions. 

- [Download the practical session archive](https://drive.google.com/file/d/1s8RuZSoa975u38zlsV3AyRjNcSnxxvbC/view?usp=share_link) and save it in `Documents` (if you prefer to use another folder, just replace `Documents` by the path to your folder in the following).
- Once the download is completed, open your file manager, go in `Documents` and extract the archive. This will create a folder called `robotics`. Inside this folder you should have two folders called `pyvrep-epuck` and `pypot`. You can then delete the archive file (`robotics.zip`).
- Download the [CoppeliaSim simulator](https://www.coppeliarobotics.com/downloads) and install it.
- Start the CoppeliaSim application. If asked, indicate that you accept incoming network connections (this is necessary for the connection with Jupyter Notebook). 
- Once the simulator is open, launch `jupyter notebook` by opening another terminal and executing:
```
cd Documents/robotics/pyvrep_epuck/notebooks/
jupyter notebook
```
- This will open a web page in the browser with a list of files. Click on the practical session you want to do (`practical_session_1.ipynb` if it is the first class). This will open a document in a new tab in your browser.

The rest of the session is described in this freshly opened document, please continue from there. 
