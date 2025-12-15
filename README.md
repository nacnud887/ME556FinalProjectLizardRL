# ME556 Final Project Lizard RL
My final project for ME 556 is to use reinforcement learning in Isaac Lab. I set rewards and encouraged forward movement. Once I got a good gait in Isaac, I translated it over to hardware. You can find some of the code I used here!

I ran into a lot of issues during the setup, so I created a simple to follow Onboarding Guide word doccument with step by step instructions for setting up Isaac Lab!

This project is based on:
https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/pip_installation.html, which has a git repository at: https://github.com/isaac-sim/IsaacLab.
I chose to do this project on a Windows Machine with a Conda Environment.

After following the directions for installation, there are specific files I changed in a cloned version of the repo. 
I did not copy all of the Isaac Files to this repo, but I will explain in order where each of the files is saved, should you want to try for yourself.

## Videos
Reinforcement Learning: https://youtu.be/2WuAtsqmyOE
Hardware: https://youtu.be/dmKyzmaHs6E

## Reinforcement Learning
### 1) Lizard Robot Code
Under source > isaaclab_assets_isaaclab_assets > robots, I saved [lizard.py](lizard.py)

### 2) Reinforcement Learning Code
In source>  isaaclab_tasks > isaaclab_tasks > manager_based > locomotion > velocity > config, I made a folder called lizard and put in: [__init__.py](__init__.py), [flat_env_cfg.py](flat_env_cfg.py), and [rough_env_cfg.py](rough_env_cfg.py).
In another folder under that, titled agents, I saved [rsl_rl_ppo_cfg.py](rsl_rl_ppo_cfg.py).

### 3) WandB Tracking
This is saved outside of the Isaac Repo, and in my system's AppData for the Conda Environment.
In AppData\Local\miniconda3\envs\env_isaaclab\Lib\site-packages\rsl_rl\utils\, I saved [wandb_utils.py](wandb_utils.py).

### 4) Export Joint Angles
In scripts > reinforcement_learning > rsl_rl, I saved [play.py](play.py).
This exported [LizardRL_Radians_12-11-2025.csv](LizardRL_Radians_12-11-2025.csv).

## MATLAB Code for Hardware

### 1) Code Setup
I adapted the code that came with the Dynamixel SDK to be usable for my case.
I named this file [LizardCodeRL.m](LizardCodeRL.m)
In the same directory as the MATLAB code, I saved [LizardRL_Radians_12-11-2025.csv](LizardRL_Radians_12-11-2025.csv), so that MATLAB could Read it.

### 2) Code Output
The output from the code is saved as [RLTrialLog.txt](RLTrialLog.txt). It lists the encoder values of each servo from 0 to 4095.
