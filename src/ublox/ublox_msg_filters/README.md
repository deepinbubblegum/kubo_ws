# ublox_msg_filters

Time synchronize multiple uBlox messages to get a single callback

Port of [message_filters::ExactTime](http://wiki.ros.org/message_filters#ExactTime_Policy) message synchronization policy to use integer time of week in milliseconds (iTOW) instead of `ros::Time` in a header.

The [message_filters::ApproximateTime](http://wiki.ros.org/message_filters#ApproximateTime_Policy) message synchronization policy is not relevent because messages generated by a ublox sensor for a single update contain identical iTOW time stamps.

See [src/example.cpp](src/example.cpp) for example usage.

Launch the example with `roslaunch ublox_msg_filters example.launch`. The example is not installed, so this must be in a source workspace.
