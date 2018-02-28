# Practical sessions on mobile robot behavior programming

This page describes how to set up and run a practical session. We assume that your computer has been booted on the Linux Ubuntu operating system (available in the classroom). So if your are currently on Windows, you have to restart your machine to boot on Ubuntu instead.

- [Download practical session archive](https://www.dropbox.com/s/qjpak7dy1bunjgo/sdic2018.tar.gz?dl=0) and save it in `Documents`.
- Once the download is done, open the file manager by clicking on the icon that looks like a yellow folder in the menu vertical bar on the left of the desktop. Go in `Documents`and extract the archive (`Right click` -> `Extract here`). This will create a folder called `sdic2018`. You can then delete the archive file (`sdic2018.tar.gz`).
- Open a terminal (the `>_` icon on the left pane of your desktop). Enter the two following commands to start the simulator (press `Enter` to execute):
```
cd Documents/sdic2018/V-REP_PRO_EDU_V3_5_0_Linux
./vrep.sh
```
- Once the simulator is open, launch `ipython notebook` by opening another terminal (middle-click on the `>_` icon) and executing:
```
cd Documents/sdic2018/pyvrep_epuck/notebooks/
ipython notebook
```
- This will open a web page in the browser with a list of files. Click on the practical session you want to do (`practical_session_1.ipynb` if it is the first class). This will open a document in a new tab in your browser.

The rest of the session is described in this freshly opened document, please continue from there. 
