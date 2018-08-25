# Assistance Arbitration

Feeling alone? Need help? Don't worry, we've got you covered. Our novel algorithms will help you choose what's the best help for you! Step right up.

Modules:

- [`assistance_msgs`](assistance_msgs/) - The primary interface between all the other modules in this folder.
- [`task_executor`](task_executor/) - Executes fetch and deliver tasks according to a program specification in YAML (now; perhaps CodeIt! in the future).
- [`assistance_arbitrator`](assistance_arbitrator/) - Research code that uses context passed on from the task_executor (or another node that might be requesting assistance), and decides where to forward the request - local or remote.
- [`local_interfaces`](local_interfaces/) - If we decide to solicit help from a local person, then the code in this folder (main entrypoint: [`local_interfaces/local_strategy`](local_interfaces/local_strategy)) handles the presentation of the request to a human.
- [`remote_interfaces`](remote_interfaces/) - If we decide to solicit help from a remote person, then the code in this folder (main entrypoint: [`remote_interfaces/remote_strategy`](remote_interfaces/remote_strategy)) handles the presentation of the request to a human.
