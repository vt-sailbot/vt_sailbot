# <p style="text-align: center;"> TLDR </p> 
For all the other nodes in the repository, don't worry about them for now. They are likely just tests or very highly under development. Either way you shouldn't worry about them on a conceptual level for now, just focus on understanding how the nodes that are documented work and developing on those unless explicitly told otherwise.


# <p style="text-align: center;"> Object Detection </p>
This node is supposed to take in a camera image and outputs both the relative "yaw" of all of the buoys and their positions as gps coordinates. In this case the "yaw" doesn't have to be in degrees, it just has to be proportional to the distance (number of pixels) from the center of the buoy to the center of the screen. This would mean that as this number goes to 0, we would be facing the buoy directly (which is what we want for the search event).


# <p style="text-align: center;"> NTRIP Client </p>
This node is designed to take RTCM messages from a local RTCM provider and publish them out so our gps can use them as correctional data. The node is currently fully functional; however, we haven't yet figured out how to get the gps to register the messages properly. That is what the official gps node is for.


# <p style="text-align: center;"> Official GPS </p>
This is a clone of the official ROS driver for our GPS series. The reason we did not use this from the start was a result of some weird choices Chris made for containerizing the entire project that didn't work with these drivers. This node works very similarly to our current GPS node; it simply listens for a GPS to plug into the device and publishes the results to a ROS topic. This ROS node, however has some other useful functionalities such as the ability to receive RTCM messages and use them as correctional data for our GPS.