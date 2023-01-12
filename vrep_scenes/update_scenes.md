These instructions are only for the 2022-2023 UPF students having previously installed the library before January 12, 2023.

As we noticed during the first session on Monday, January 9, the connection between the Jupyter Notebook and the CoppeliaSim simulator was quite unstable. This is due to [important changes in the remote API provided by Coppelia](https://www.coppeliarobotics.com/helpFiles/index.html). This was leading to errors for many students, especially those using Windows.

I have worked on a fix for this issue. To solve it you need to follow the instructions below (whatever OS you are using). If you have time you can do it before the second class today (Jan 12). If you think you need help for doing it, either ask another student who did it successfully, or ask me at the beginning of the class today.

- Uninstall  the previously installed `CoppeliaSim 4.4.0 Edu` version.
- Install `CoppeliaSim 4.2.0 Edu`
    - Download the `CoppeliaSim 4.2.0 Edu` version for your OS from https://www.coppeliarobotics.com/previousVersions
    - Install it
    - Launch CoppeliaSim. The simulator will open and close just after. This is due to a previously [imposed limit on the runtime of Edu versions](https://forum.coppeliarobotics.com/viewtopic.php?f=9&t=9334). To allow this version to run you need to:
        - On Windows
            - Go to `C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\system`
        - On Mac
            - Right on `coppeliaSim` in the `Application` folder, then `Show Package content`. Then go to `Content/MacOS/system`.
        - Then, open the file `usrset.txt`
        - Add a line at the end of this file with `allowOldEduRelease=7775`
    - Then launch the CoppeliaSim application. It should now open normally.
- Update the updated scene files
    - [Download the new scenes](https://drive.google.com/file/d/1sONUjYGHPVegi0j04fxvM8XONio2v-kN/view?usp=sharing) and unzip the downloaded zip file
    - Make sure your work on Session 1 is safe by copy pasting the file `epuck-scene-1.ttt` (in `robotics/pyvrep_epuck/vrep_scenes`) to another location
    - Replace all the scene files in `robotics/pyvrep_epuck/vrep_scenes` by the files in the `updated_scenes` unzipped folder

You can then continue working from Session 2 on these updated scenes. If you followed the instructions above correctly, you should see the following messages in the CoppeliaSim console (at the bottom of the app window) when you open a scene file:

```
Init remote api connections
19997,1
19998,1
19999,1
20000,1
20001,1
20002,1
20003,1
Init done
```
