# IsaacSim-PX4_sitl_bridge (Beluga Bridge)
A custom omnigraph node to bridge px4_sitl and NVIDIA-isaacsim


<img width="640" height="640" alt="beluga" src="https://github.com/user-attachments/assets/bcfe4290-bf33-430e-bc53-ee367f020652" />



- Instructions, brief walkthroughs and demos coming soon!

source env variables
```bash
export PX4_BINARY_PATH=<path_to_your_px4_toolchain(PX4_Autopilot path)> (mandatory)
export PX4_SYS_AUTOSTART=<your airframe param script code val> (optional)
export PX4_SIM_MODEL=x500(or your custom frame) (optional)
```


## Note: Please note that as of now (2026 APRIL) its not in the plug and play kind of condition needs a bit of change in that department 

## Also Kindly Note: [Check this dude's repo](https://github.com/limshoonkit/uosm.isaac.px4_bridge/tree/main) for the main stuff, I simple got sidetracked and made mine to suit IsaacSim 4.5.0 and also later on add custom sensor plugins so I wanted to know how this repo worked. Plus its easily customizable and work with less complex abstractions compared to the Pegasus Simulator which sucks btw (no offence)
