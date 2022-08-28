# Autonomous saildrone simulation and motion control


This repo contains the final code of the thesis in Cybernetics and Robotics: 
- A motion control system for a sailboat drone is developed and tested
for the purpose of collision avoidance and autonomous path following under 
various environmental conditions
- A simulation framework is developed, consisting of a comprehensive model 
of the system dynamics and a simplified perception model

Get started:

- Change the file path in "main_v10_combined.m", in the executables folder
- Run main_v10_combined.m

Init-files:
- "init_saildrone_dynamics_v5.m" is the configuration and setup of the drone.
- "init_low_level_controllers_2.m" and "init_mid_level_controllers.m" is for the controllers and reference models
- "init_guidance.m" is the setup of the guidance system
- The collision avoidance parameters are contained in the Simulink file

Overview:
- A overview of the full system can be found in the thesis
- For a more detailed overview, see the Simulink file in the "Modeling/Models" folder




The MSS toolbox found at https://github.com/cybergalactic/MSS is also utilized.
