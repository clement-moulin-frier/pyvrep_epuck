# Practical sessions on mobile robot behavior programming

[Download practical session material](https://www.dropbox.com/s/afbec7wc3wl04fw/sdic2018.tar.gz?dl=0)


This page describes how to set up and run a practical session. We assume that your computer has been booted on the Linux Ubuntu operating system (available in the classroom). So if your are currently on Windows, you have to restart your machine to boot on Ubuntu instead.

First, open the file manager by clicking on the icon that looks like a yellow folder in the menu vertical bar on the left of the desktop. Go in `Documents` and create a new folder called `rti2016` (right-click in the file manager window and choose `Carpeta nova`). This is in this folder that we will copy all the material necessary for the practical sessions. 

Then, download the V-REP robot simulator [here](http://coppeliarobotics.com/files/V-REP_PRO_EDU_V3_3_2_64_Linux.tar.gz). Save it in the directory we just created, which is `Documents/rti2016`.

Once the download is done, go to the `Documents/rti2016` directory in the file manager. Extract the archive by right-clicking on it and choosing `Extreu aquì`. You can delete the archive file (the one ending by `.tar.gz`), we'll not use it. 

We need to make a modification in this freshly extracted folder. Go in it and open the file called `remoteApiConnections.txt`. At the very end of it, copy and paste the following lines (but don't remove anything that was already present in the file):

    // Additions to deal with multi-robot setup
    portIndex2_port         = 19998
    portIndex2_debug        = false
    portIndex2_syncSimTrigger   = true
    
    portIndex3_port         = 19999
    portIndex3_debug        = false
    portIndex3_syncSimTrigger   = true
    
    portIndex4_port         = 20000
    portIndex4_debug        = false
    portIndex4_syncSimTrigger   = true
    
    portIndex5_port         = 20001
    portIndex5_debug        = false
    portIndex5_syncSimTrigger   = true
    
    portIndex6_port         = 20002
    portIndex6_debug        = false
    portIndex6_syncSimTrigger   = true
    
    portIndex7_port         = 20003
    portIndex7_debug        = false
    portIndex7_syncSimTrigger   = true

Now open a terminal (the `>_` icon on the left pane of your desktop). Go to the directory you've just extracted by entering the following command (press `Enter` to execute it):

    cd Documents/rti2016/V-REP_PRO_EDU_V3_3_2_64_Linux

And run the simulator by executing:
    
    ./vrep.sh

At this point you should see the V-REP simulator graphical interface. Contact the teacher if it is not the case.

Now let's download the material for the practical session. Open another terminal with a middle-click on the `>_` icon. Go to the `rti2016` directory as before:

    cd Documents/rti2016

Then copy the material of the session on your machine by running:

    git clone https://github.com/clement-moulin-frier/pyvrep_epuck.git

We also need to install a few extra libraries. To do so, once the previous command has terminated, execute the following:

    git clone https://github.com/clement-moulin-frier/pypot.git

as well as this one:

    git clone https://github.com/certik/enum34.git

Now we have everything we need. Go to the practical session directory by typing:

    cd pyvrep_epuck/notebooks

Start the interactive environment we'll use in the practical sessions by typing:

    ipython notebook

This will open a new page in your browser (usually the page appears in Firefox). Click on the practical session you want to open (start with the 1st one of course, which is called `practical_session_1.v3.ipynb`). This will open a document in a new tab in your browser.

The rest of the session is described in this freshly opened document, please continue from there. 
