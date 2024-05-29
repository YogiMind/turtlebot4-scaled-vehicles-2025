# Project Overview

This project is focused on the development and documentation of the TurtleBot4 robot and the Hermes system, a software system designed to enable autonomous driving capabilities for the TurtleBot4. The repository is structured into three main directories: Documentation, Software, and Results. The Documentation section covers theory, tutorials, and setup instructions. The Software section includes the codebase for the Hermes system, and the Results section contains experimental results and analysis.

## Where to begin

To set up and quickly start using the TurtleBot4 with the Hermes system, we recommend the following approach:

1.  **[Base Setup](/Documentation/TurtleBot4/Setup/Base-Setup.md):** Follow the instructions in Base Setup to install ROS2 and the necessary TurtleBot4 packages on your PC.
2.  **[Network Discovery Server](/Documentation/TurtleBot4/Setup/Network-Discovery-Server.md):** Set up the network discovery server to enable communication between your PC and the TurtleBot4 using the guide in [Network Discovery Server](/Documentation/TurtleBot4/Setup/Network-Discovery-Server.md).
3.  **[Camera Configuration](/Documentation/Hermes/camera_configuration.md):** If you need to adjust the camera position or orientation, refer to the instructions in [Camera Configuration](/Documentation/Hermes/camera_configuration.md).
4.  **[Launching Hermes](/Documentation/Hermes/launch_files.md):** Launch the Hermes system using the provided launch files. Detailed instructions can be found in [Launch Files (Hermes)](/Documentation/Hermes/launch_files.md).
5.  **RViz:** Use RViz to visualize sensor data, maps, and other information from the TurtleBot 4 and Hermes system. Refer to the [RViz documentation](/Documentation/Software/RViz.md) for guidance.

## Documentation

### Hermes

The [Hermes directory](/Documentation/Hermes/) contains comprehensive documentation about the Hermes system. It includes information about the core functionality, image processing nodes, and launch files. Here are some key files:

-   **hermes\_core.md:** Overview of the core Hermes node responsible for mapping and localization.
-   **depth\_intensity\_image\_syncer.md:** Details about the node that synchronizes depth and intensity images from the OAK-D camera.
-   **launch\_files.md:** Explanation of the launch files used to start and configure the Hermes system.
-   **camera\_configuration.md:** Guide on how to modify the camera settings for the TurtleBot4.

### TurtleBot4

The [TurtleBot4 directory](/Documentation/TurtleBot4/) contains documentation about the TurtleBot 4. It includes information about the on-board computers, setup instructions, sensor details, and troubleshooting guides. Here are some key files:

-   **On-Board Computers:** Detailed information about the Raspberry Pi 4B used in the TurtleBot4, including how to flash the SD card.
-   **Setup:** Instructions on how to set up the TurtleBot4, including network setup and ROS configuration.
-   **Troubleshooting:** Guides to help you troubleshoot issues with the TurtleBot4, including sensor problems and software configurations.

## Software

### Hermes 

[Hermes](/Software/TurtleBot4/hermes/) - The system which is run on the TurtleBot 4 which we have used for mapping and navigation. Read the docs [here](/Documentation/Hermes/hermes_core.md)  

[Launch Files](/Software/TurtleBot4/hermes/launch/) - The Launch Files used, read the docs [here](/Documentation/Hermes/launch_files.md)

### Testing

[Image Compare](/Software/Testing/Image-Compare/image_comparison.py) - The [Testing](/Software/Testing/Image-Compare/) suite includes image comparison algorithms, namely NCC, MSE and SSIM. In [image_comparison.py](/Software/Testing/Image-Compare/image_comparison.py) you can set your own weights for each algorithm and you will get a csv containing the results. Read the docs [here](/Documentation/Hermes/image_comparisons.md)

[Graphs](/Software/Testing/make_graphs.py) - This script creates graphs to help visualize the results you get from your image comparisons. Read the docs [here](/Documentation/Hermes/image_comparisons.md)

