# Local Strategy

This package coordinates the different methods of soliciting help from a local human (once such a person is identified).

The current method of soliciting help:

1. Looks around for a person (potentially forever. TODO: put a time limit.)
1. Enters compliant mode - all joints can be moved by a human.
1. When a person is found, starts a dialogue:
    1. Tells the person what failed and the likely cause of that failure.
    1. Can respond to questions of what else might have failed.
    1. Can explain that safety is the reason we're in compliant mode. (TODO: allow autonomous actions after asking person for safety checks)
1. When person indicates that the problem should be resolved, chirp happily and return.


## Notes

The different keys and values in the errors that are sent with the assistance request are:

- **task** - name of the task
- **action** - name of action
- **cause** - some string reason-for-failure
    - arm:"Unknown format"
    - beep:"Unrecognized"
    - gripper:"Unrecognized"
    - move:"Unknown format"
    - speak:"Invalid Args"
    - task:"Unexpected Params"
    - task:"Invalid Variables"
- **context** - argument to run
    - arm:poses - the name or the list of poses
    - beep:beep - name of the beep
    - gripper:command - open/close is expected
    - move:location - the name or the list of waypoints
    - speak:(text, affect,)
    - task:variables - the dictionary of K,V from the underlying action/task
- **status** - GoalStatus code of action client
- **goal** - transformed version of argument sent to run
    - arm:pose_waypoints - the series of waypoints to take the arm to
    - beep:beep - name of the beep key
    - find_closest_person:max_duration - the max amount of time, float, within which to find a person
    - find_grasps:segmented_obj - of type rail_manipulation_msgs/SegmentedObject
    - find_object:obj - the string name of the object in the DB
    - gripper:command - open/close command sent
    - listen:expected_cmd - the speech command that we were expecting
    - look:pose - dictionary of the pose to look at
    - move:coords - list of waypoints to visit
    - pick:cube_idx - the index of the cube in the segmented objects list
    - speak:text - the text to speak
    - torso:heigh - the height in float to raise the torso to
- **result** -  action client result
    - arm:PresetJointsMoveResult - bool success, int MoveItErrorCode
    - beep:SoundRequestResult - bool playing, time stamp
    - result:GripperCommandResult - position, effort, bool stalled, bool reached_goal
    - look:PointHeadResult - empty
    - move:MoveBaseResult - empty
    - pick:ExecuteGraspResult - bool success, int MoveItErrorCode, int failure_point in grasp execution
    - speak:SoundRequestResult - bool playing, time stamp
    - torso:FollowJointTrajectoryResult - int error_code, string error_string
- **srv** - name of the service that failed
    - find_grasps:suggester/suggest_grasps
    - find_grasps:suggester/pairwise_rank
    - find_object:rail_segmentation/segment_objects
    - find_object:grasp_executor/clear_objects
    - find_object:grasp_executor/add_object
    - place:grasp_executor/drop_object
- **orig_goal** - original argument to run
    - arm:poses - the original name of the poses, or perhaps the list of poses
    - move:location - the original name of the waypoints, or the list of them
- **attempt_num** - idx of the arm movement attempt when run stopped. Only arm
- **num_grasps** - number of grasps found be suggester/suggest_grasps. Only find_grasps and pick
- **segmented_objects** - of type rail_manipulation_msgs/SegmentedObjectList; segmented objects returned by RAIL segmentation. Only find_object
- **found_idx** - index of the object found in the SegmentedObjectList based on data in the object DB. Only find_object
- **found_obj** - of type rail_manipulation_msgs/SegmentedObject; object with idx found_idx in the SegmentedObjectLst. Only find_object
- **coord_num** - the index of the waypoint that we were moving to when we failed. Only move
- **grasp_num** - the index of the grasp that we were attempting when we failed. Only pick
- **grasps** - all the grasps that we were planning to/did attempt. Only pick
- **affect** - the affect key that we want the text to be said with. Only speak
- **expected_params** - the expected parameters to a task. Only task
- **received_params** - the params that were actually sent to the task. Only task
- **step_idx** - the index of the task step that was executing when we exited. Only task
- **step_name** - the name of the task step that was executing when we exited. Only task
- **expected_vars** - the expected vars that the task should have provided. Only task
- **received_vars** - the vars that the task actually provided. Only task
- **search_duration** - the duration (in seconds, float) spent searching for the nearest person. Only find_closest_person
- **received_cmd** - the command received from the speech listener. Only listen


## TODO

Some future ideas to add to this interaction coordinator:

1. Time limit within which a person must be found.
1. Dialogue:
    1. Responses to more complex queries.
    1. Check for safety of movement before moving.
    1. Watchdog for the word "STOP".
1. Enable some autonomous behaviour.
