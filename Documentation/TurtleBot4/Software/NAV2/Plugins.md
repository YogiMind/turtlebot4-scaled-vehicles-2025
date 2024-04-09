# Plugins for NAV2

## Overview 



## Enable plugins

In the TurtleBot 4 navigation folder you'll find your config file for NAV2, [nav2.yaml]() 

It will look something like this:   

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.03
      plugins: ["static_layer", "voxel_layer", "inflation_layer"] 
...
...
...
```   
where the last row "plugins" is where you enter what plugins you want to enable. 
