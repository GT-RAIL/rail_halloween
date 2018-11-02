This is a service that detects bags based on it being the 3D point between the elbows of the person nearest to it.

The service is GetBagPose.srv. At the moment, the request is blank (see the main function in `bag_pose_client.py`), and the response is a message `bag_pose` of type `sensor_msgs/PointStamped`. The point is in the optical frame.
