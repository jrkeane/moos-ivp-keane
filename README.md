# moos-ivp-keane
extend with focus on range only homing

# Range-Only (RO) homing for AUV with an Angle Parameterised Cubature Kalman Filter (APCKF). 
RO localisation and homing of a moving target is difficult task with low observability, and prior knowledge needed for target vehicle.  
This was necessary though for Autonomous Underwater Vehicle (AUV) localisation and homing where limited bandwidth in acoustic communications means that range measurements can travel further distance, and more reliably, than say an entire packet with target position. 
Further, most modems are readily configured to provide range, however Ultra Short Baseline (USBL) is required to get range and bearing. 
Need moos-ivp, python-moos and then some basic python dependencies. I’m running this on Windows 10 using Ubuntu WSL with VcXsrv server providing the UI. Ubuntu bash script starts vxcsrv xserver so it's running for MOOSDB. Running standalone ubuntu1804 would clearly be way easier. 

# Installs and setup: 
Moos-ivp (https://oceanai.mit.edu/ivpman/pmwiki/pmwiki.php?n=Lab.ClassSetup#obtaining_ivp)
Moos-ivp-extend (https://dspace.mit.edu/handle/1721.1/46361)

Pretty much rely on the MIT labs to get you up to speed with moos: https://ocw.mit.edu/courses/mechanical-engineering/2-s998-marine-autonomy-sensing-and-communications-spring-2012/
Then jump up to speed with Python-moos (https://github.com/msis/python-moos). Annoying install with tweaks needed that I can’t remember, and not much documentation but its possible and worth it to be able to develop in python instead of cpp. 
The apckf apps are done for Python3.6 with pymoos, time, re, csv, math, threading, re, numpy, matplotlib. 
Enjoying PyCharm with anaconda lately too (https://www.jetbrains.com/pycharm/promo/anaconda/)

# Multi-mission simulation
Essentially there is a moos-ivp mission called j88_hugo. Inside here we are hosting a multi vehicle mission simulator app with three communities:
•	Shoreside 9000
•	USV (archie) 9001
•	AUV (jackal) 9009

#  To launch: 
first terminal:
../j88_hugo/
./launch.sh 

second terminal: 
../j88_hugo/python3 ownship_apckf.py

and deploy from pMarineViewer.

if you want to monitor comms from shoreside to AUV then broadcasting waypoints, third terminal: 
uXMS APCKF_UPDATES RANGE_USV_AUV 
port 9009

# ownship_apckf.py
This is the moos app running onboard the auv which gets auv x,y and incoming ranges. Then runs the apckf for each new range and broadcasts a predicted position of the USV. This can be fed straight to the helm (current situation) but will be developed to include a sanity check and optimised homing trajectory. 

# shoreside.py
Simple app to collect extra usv info and range info to send to auv. 

# pUpdatesRelay
cpp app acts as a relay from pymoos to ivp as a dirty workaround for a utc/db_uptime issue with pymoos and ivp-helm. Apparently not an issue on proper ubuntu (just on my WSL setup). doesn't hurt anyway. 
