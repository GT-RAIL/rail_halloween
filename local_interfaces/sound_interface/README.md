# Sound Interface

This package generates sounds for the robot.

## Speech in Two Minutes

In order to perform TTS, the packages in this folder use [Mary TTS](http://mary.dfki.de/).

**Prerequisites**:

```bash
# Need Java 7+ and Maven 3+
sudo apt-get install maven oracle-java8-installer
sudo -H pip install requests
```

Installing MaryTTS (in a folder of your choice. For the rest of these instructions, I'm going to assume this folder is `$MARYTTS_ROOT`):

```bash
cd $MARYTTS_ROOT
git clone git@github.com:marytts/marytts.git
cd marytts
git checkout v5.2
mvn install
```

By default, I prefer to use the `dfki-prudence-hsmm` voice. However, you should play with the other voices at [this link](http://mary.dfki.de:59125/). To download the voice of your choice:

```bash
cd $MARYTTS_ROOT/target/marytts-5.2/bin/
./marytts-component-installer
```

Select the voice(s) you want to install and hit install. Now you're ready for TTS!

**Demo**:

The following commands should each be run in their own terminal window.

```bash
# Run the Mary TTS server. It's a good idea to have an alias for this.
$MARYTTS/target/marytts-5.2/bin/marytts-server

# Run a ROS Core
roscore

# Run a sound_play node
rosrun sound_play soundplay_node.py

# Run the script to test sounds. The following command prints out a help
python $(pwd)/local_strategy/src/local_strategy/speak.py --help
```

The demo script (the last command) has two modes of operation - `beep` and `speak`. The former plays pre-recorded R2D2 beeps based on the key. The latter performs TTS using Mary and outputs it via `sound_play`. There are also options to change the emotion in the output speech with the use of `EmotionML`.
