GulliView ROS Client
====================

A ROS node acting as a client for GulliView position data.

Start GulliView on the roof system using the following command:

    cd ~/repository/GulliView/build && ./startCamerasBroadcast.sh $PORT
    
where PORT points at the port where you want gv_client nodes to listen. 

## The node takes the following parameters:

| Parameter | Default | Description |
| --------- | ------- | ----------- |
| `~host` | "0.0.0.0" | Host IP that the server should bind to. Defaults to all interfaces on the host. |
| `~port` | 2121 | Port that the server should listen on, where GulliView will send its messages. |
| `~tag_id` | "all" | Which tag ID's to listen for. Defaults to all; allowed values are numeric ID's corresponding to the APRIL tag ID's. |

Example start:
    
    rosrun gv_client gv_socket_server.py _host:="0.0.0.0" _port:=2121 _tag_id:=4

Data is published on the `/gv_positions` topic with the GulliViewPosition message type, defined like so:

https://github.com/5535-ROStig/DAT295-av/blob/d238f886bc93f7a53b91cd7dafae62b24f86df0e/src/gv_client/msg/GulliViewPosition.msg
