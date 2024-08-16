# Publisher-subscriber example

This is an example of a publisher and subscriber.

To run the example, generate Go bindings by running

    go generate

Then you can run the publisher and subscriber in separate terminals:

    source /opt/ros/${ROS_DISTRO}/setup.bash
    go run ./publisher
    go run ./subscriber
