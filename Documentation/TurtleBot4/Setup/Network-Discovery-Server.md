# Discovery Server
After flashing the SD-Card and booting up the Turtlebot 4, the robot will be in Access Point-mode. 

## Connecting to Turtlebot 
1. Connect to the TurtleBots WiFi, SSID is "TurtleBot4".

2. Once connected to the WiFi, open up a terminal and write:  
```ssh ubuntu@10.42.0.1```  

3. You're now connected to the on-board Raspberry Pi 4B. Run the TurtleBot setup by writing:  
```turtlebot4-setup```  
This will bring up a setup screen where you can add the credentials for the network you wish the TurtleBot to connect to.

## Configuring turtlebot4-setup
Now, you need to set up the TurtleBot to use Discovery Server. Once you have run the ```turtlebot4-setup``` command, you will be greeted by the following screen.  

<img src="../../Assets/Images/TurtleBot4/TurtleBot4-setup/TB4_Setup_1.png" width="400">

### ROS Setup
Navigate to Discovery Server  
<img src="../../Assets/Images/TurtleBot4/TurtleBot4-setup/TB4_Setup_2.png" width="400">  

Copy these settings   
<img src="../../Assets/Images/TurtleBot4/TurtleBot4-setup/TB4_Setup_3.png" width="400"> 

### WiFi Setup
Navigate back to the main setup menu (don't forget to save the settings) and go to WiFi-settings.  
`Wi-Fi Mode` should be set as `client`  
`SSID` is the name of the network you want to connect to  
`Password` is the WiFi password  
`Band` is recommended to be set to `5GHz`  
`IP Adress` can remain empty  
`DHCP` should be set to `True`

<img src="../../Assets/Images/TurtleBot4/TurtleBot4-setup/TB4_Setup_4.png" width="400">

#### **Don't forget to hit Save!**

You should now go back to the main menu again, hit `Apply Settings` and the TurtleBot will reboot and try to connect to the WiFi you've set it up for. 




