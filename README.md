# Practical sessions on mobile robot behavior programming

This page describes how to set up and run a practical session. We assume that your computer has been booted on the Linux Ubuntu operating system (available in the classroom). So if your are currently on Windows, you have to restart your machine to boot on Ubuntu instead.

First, download the V-REP robot simulator [here](http://coppeliarobotics.com/V-REP_PRO_EDU_V3_2_2_64_Linux.tar.gz). Save it in `Documents` (by default normally).

Once the download is done, go to the `Documents` directory (to open the file browser, click on the "folder"-like icon on the left pane of you desktop). Extract the archive by right-clicking on it and choosing `Extreu aquì`.

Open a terminal (the `>_` icon on the left pane of your desktop). Go to the directory you've just extracted using:

    cd Documents/V-REP_PRO_EDU_V3_2_2_64_Linux

And run the simulator by typing:
    
    ./vrep.sh

At this point you should see the V-REP simulator graphical interface. Contact the teacher if it is not the case.

Now let's download the material for the practical session. Open another terminal with a middle-click on the `>_` icon. Go to the `Document` directory:

    cd Documents

Then copy the material of the session on your machine by running:

    git clone https://github.com/clement-moulin-frier/pyvrep-epuck.git

Go to the practical session directory by typing:

    cd pyvrep-epuck/notebooks

Start the interactive environment we'll use in the practical sessions by typing:

    ipython notebook

This will open a new page in your browser. Click on `practical_session_1`. This will open a document in a new tab in your browser entitled **First step with the simulator**.

The rest of the session is described in this freshly opened document, please continue from there. 