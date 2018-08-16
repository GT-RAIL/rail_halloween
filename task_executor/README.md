# Task Executor

This package executes a pick-and-place task. When there is a failure, it calls sends a request to an arbitrator module to help it decide who to contact, and then it sends a request to a help interface. When the interface signals a completion, the task execution resumes
