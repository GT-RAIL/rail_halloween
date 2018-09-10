#!/usr/bin/env python
# Interface to sounds for Ava

from __future__ import print_function, division

import os
import argparse
import requests
import tempfile

import rospy
import rospkg
import actionlib

from sound_play.msg import SoundRequestAction, SoundRequestGoal, SoundRequest


class SoundClient(object):
    """
    A reimplementation of ROS's SoundClient to help with speech and sound
    tasks. We force the use of an action client in this class. At this moment,
    ROS's sound_play node does not allow multiple sounds to be played at the
    same time, so the same constraint applies here. In the near future,
    that node should be reimplemented.

    To play a beep, you must refer to one of the constant keys in this
    package that begins with `SOUND_*`. By default, the sound files in `sounds`
    map to one of these keys.

    To speak, you can use EmotionML syntax to specify text. The TTS interface
    expects a MaryTTS server running in the background. The TTS could be a long
    running process and is currently blocking.
    """

    # Template EmotionML
    EMOTIONML_TEMPLATE = \
    """<emotionml version="1.0" xmlns="http://www.w3.org/2009/10/emotionml"
category-set="http://www.w3.org/TR/emotion-voc/xml#everyday-categories">
{speech}
</emotionml>
    """

    # Keys for the different beeps
    BEEP_CHEERFUL = "CHEERFUL"
    BEEP_CONCERNED = "CONCERNED"
    BEEP_EXCITED = "EXCITED"
    BEEP_HAPPY = "HAPPY"
    BEEP_PLAYFUL = "PLAYFUL"
    BEEP_PROUD = "PROUD"
    BEEP_SAD = "SAD"
    BEEP_SHOCKED = "SHOCKED"
    BEEP_SURPRISED = "SURPRISED"
    BEEP_UNSURE = "UNSURE"

    # Keys for the different affects
    AFFECT_SAD = "SAD"
    AFFECT_HAPPY = "HAPPY"
    AFFECT_ANGRY = "ANGRY"
    AFFECT_CALM = "CALM"
    AFFECT_NERVOUS = "NERVOUS"

    # Mary TTS server URL
    MARY_SERVER_URL = "http://localhost:59125/process"

    @staticmethod
    def make_happy(text):
        """Make the text happy!"""
        return ('<emotion><category name="happy" />{}</emotion>'.format(text))

    @staticmethod
    def make_sad(text):
        """Make the text sad :("""
        return ('<emotion><category name="sad" />{}</emotion>'.format(text))

    @staticmethod
    def make_angry(text):
        """Make the text angry"""
        return ('<emotion><category name="angry" />{}</emotion>'.format(text))

    @staticmethod
    def make_calm(text):
        """Make the text calm"""
        return (
            """
        <emotion dimension-set="http://www.w3.org/TR/emotion-voc/xml#pad-dimensions">
            {}
            <dimension name="arousal" value="0.3"/><!-- lower arousal -->
            <dimension name="pleasure" value="0.9"/><!-- high positive valence -->
            <dimension name="dominance" value="0.8"/><!-- high potency    -->
        </emotion>
            """.format(text)
        )

    @staticmethod
    def make_nervous(text):
        """Make the text nervous"""
        return (
            """
        <emotion dimension-set="http://www.w3.org/TR/emotion-voc/xml#pad-dimensions">
            {}
            <dimension name="arousal" value="0.9"/><!-- high arousal -->
            <dimension name="pleasure" value="0.2"/><!-- negative valence -->
            <dimension name="dominance" value="0.2"/><!-- low potency    -->
        </emotion>
            """.format(text)
        )

    def __init__(self, beeps=None):
        # Want to reimplement SoundClient so that we are always using the action
        # interface to the sound_play_node.
        self.sound_client = actionlib.SimpleActionClient("sound_play", SoundRequestAction)
        self._tmp_speech_file = None

        # Load the beeps
        self.beeps = beeps
        if self.beeps is None:
            default_sound_path = os.path.join(
                rospkg.RosPack().get_path('sound_interface'),
                'sounds'
            )
            self.beeps = {
                SoundClient.BEEP_CHEERFUL: os.path.join(default_sound_path, 'R2D2_cheerful.wav'),
                SoundClient.BEEP_CONCERNED: os.path.join(default_sound_path, 'R2D2_concerned.wav'),
                SoundClient.BEEP_EXCITED: os.path.join(default_sound_path, 'R2D2_excited.wav'),
                SoundClient.BEEP_HAPPY: os.path.join(default_sound_path, 'R2D2_happy.wav'),
                SoundClient.BEEP_PLAYFUL: os.path.join(default_sound_path, 'R2D2_playful.wav'),
                SoundClient.BEEP_PROUD: os.path.join(default_sound_path, 'R2D2_proud.wav'),
                SoundClient.BEEP_SAD: os.path.join(default_sound_path, 'R2D2_sad.wav'),
                SoundClient.BEEP_SHOCKED: os.path.join(default_sound_path, 'R2D2_shocked.wav'),
                SoundClient.BEEP_SURPRISED: os.path.join(default_sound_path, 'R2D2_surprised.wav'),
                SoundClient.BEEP_UNSURE: os.path.join(default_sound_path, 'R2D2_unsure.wav'),
            }

        # Load the affects
        self.affects = {
            SoundClient.AFFECT_SAD: SoundClient.make_sad,
            SoundClient.AFFECT_HAPPY: SoundClient.make_happy,
            SoundClient.AFFECT_NERVOUS: SoundClient.make_nervous,
            SoundClient.AFFECT_CALM: SoundClient.make_calm,
            SoundClient.AFFECT_ANGRY: SoundClient.make_angry,
        }

        # Need to connect to the server
        rospy.loginfo("Connecting to sound_play...")
        self.sound_client.wait_for_server()
        rospy.loginfo("...sound_play connected")

    def get_state(self):
        """Returns the state of the action client"""
        return self.sound_client.get_state()

    def get_result(self, blocking=False):
        """Returns the result of the last sound action. Blocks for a result"""
        if blocking:
            self.sound_client.wait_for_result()
        return self.sound_client.get_result()

    def get_beep_names(self):
        """Get the keys to the different beep types that are available"""
        return self.beeps.keys()

    def get_affect_names(self):
        """Get the keys to the different affects that are available"""
        return self.affects.keys()

    def say(self, text, affect="", blocking=False, **kwargs):
        """Perform TTS using EmotionML"""

        # Transform the text if the affect argument calls for it
        if affect and affect.upper() in self.affects.keys():
            text = self.affects[affect.upper()](text)

        # Create the vars for the EmotionML query
        text = SoundClient.EMOTIONML_TEMPLATE.format(speech=text)
        query_dict = {
            'INPUT_TEXT': text,
            'INPUT_TYPE': 'EMOTIONML',
            'LOCALE': 'en_GB',
            'VOICE': 'dfki-prudence-hsmm',
            'OUTPUT_TYPE': 'AUDIO',
            'AUDIO': 'WAVE',
            'effect_Robot_selected': 'on',
            'effect_Robot_parameters': 'amount:60.0',
        }

        # Close the old speech file, if it exists
        if self._tmp_speech_file is not None:
            self._tmp_speech_file.close()
            self._tmp_speech_file = None

        try:
            r = requests.post(SoundClient.MARY_SERVER_URL, params=query_dict)
            if r.headers['content-type'] != 'audio/x-wav':
                rospy.logerr("Response Error Code: {}. Content: {}".format(r.status_code, r.content))
                return

            self._tmp_speech_file = tempfile.NamedTemporaryFile(prefix='marytts', suffix='.wav')
            self._tmp_speech_file.write(r.content)
            self._tmp_speech_file.flush()

            # Now send the file's name over to sound play
            sound = SoundRequest()
            sound.sound = SoundRequest.PLAY_FILE
            sound.command = SoundRequest.PLAY_ONCE
            sound.arg = self._tmp_speech_file.name
            self._play(sound, blocking=blocking, **kwargs)

        finally:
            # We're done, so delete the file
            if self._tmp_speech_file is not None and blocking:
                self._tmp_speech_file.close()
                self._tmp_speech_file = None

    def beep(self, key, blocking=False, **kwargs):
        """Play one of the beeps and boops that we know of"""
        if not key or key.upper() not in self.beeps:
            return

        sound = SoundRequest()
        sound.sound = SoundRequest.PLAY_FILE
        sound.command = SoundRequest.PLAY_ONCE
        sound.arg = self.beeps[key.upper()]
        self._play(sound, blocking=blocking, **kwargs)

    def stop(self):
        """Stop all sounds"""
        self.sound_client.cancel_all_goals()

        # Send a stop request
        sound = SoundRequest()
        sound.sound = SoundRequest.ALL
        sound.command = SoundRequest.PLAY_STOP
        sound.arg = ""

        self._play(sound, blocking=True)

    def _play(self, sound, blocking, **kwargs):
        # Send the command
        rospy.logdebug("Sending sound action with (sound, command, arg): {}, {}, {}"
                       .format(sound.sound, sound.command, sound.arg))
        goal = SoundRequestGoal(sound_request=sound)
        self.sound_client.send_goal(goal)

        # If blocking, wait until the sound is done
        if blocking:
            self.sound_client.wait_for_result()
            rospy.logdebug('Response to sound action received')


if __name__ == '__main__':
    # For testing purposes
    rospy.init_node('speaker_test')
    client = SoundClient()

    def on_speak(args):
        text = args.text
        affect = args.affect
        client.say(text, affect, blocking=True)

    def on_beep(args):
        client.beep(args.key, blocking=True)

    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers()

    speak_parser = subparsers.add_parser("speak", help="get the robot to speak")
    speak_parser.add_argument("text", help="text to speak")
    speak_parser.add_argument("--affect", choices=client.get_affect_names(),
                              help="say things in an affected voice")
    speak_parser.set_defaults(func=on_speak)

    beep_parser = subparsers.add_parser("beep", help="get the robot to beep")
    beep_parser.add_argument("key", help="type of beep to produce",
                             choices=client.get_beep_names())
    beep_parser.set_defaults(func=on_beep)

    args = parser.parse_args()
    args.func(args)
