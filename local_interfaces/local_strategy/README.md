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

```
TODO
```


## TODO

Some future ideas to add to this interaction coordinator:

1. Time limit within which a person must be found.
1. Dialogue:
    1. Responses to more complex queries.
    1. Check for safety of movement before moving.
    1. Watchdog for the word "STOP".
1. Enable some autonomous behaviour.
