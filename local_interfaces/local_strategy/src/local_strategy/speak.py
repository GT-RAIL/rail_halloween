#!/usr/bin/env python
# Speech for Ava

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
    SOUND_SAD = "SAD"
    SOUND_HAPPY = "HAPPY"
    SOUND_SHOCKED = "SHOCKED"
    SOUND_UNSURE = "UNSURE"

    # Mary TTS server URL
    MARY_SERVER_URL = "http://localhost:59125/process"

    def __init__(self, beep_keys=None):
        # Want to reimplement SoundClient so that we are always using the action
        # interface to the sound_play_node.
        self.sound_client = actionlib.SimpleActionClient("sound_play", SoundRequestAction)

        self.beep_keys = beep_keys
        if self.beep_keys is None:
            default_sound_path = os.path.join(
                rospkg.RosPack().get_path('local_strategy'),
                'sounds'
            )
            self.beep_keys = {
                SoundClient.SOUND_SAD: os.path.join(default_sound_path, 'R2D2_sad.wav'),
                SoundClient.SOUND_HAPPY: os.path.join(default_sound_path, 'R2D2_excited.wav'),
                SoundClient.SOUND_UNSURE: os.path.join(default_sound_path, 'R2D2_unsure.wav'),
                SoundClient.SOUND_SHOCKED: os.path.join(default_sound_path, 'R2D2_shocked.wav'),
            }

    def make_happy(self, text):
        """Make the text happy!"""
        return ('<emotion><category name="happy" />{}</emotion>'.format(text))

    def make_sad(self, text):
        """Make the text sad :("""
        return ('<emotion><category name="sad" />{}</emotion>'.format(text))

    def make_angry(self, text):
        """Make the text angry"""
        return ('<emotion><category name="angry" />{}</emotion>'.format(text))

    def make_calm(self, text):
        """Make the text calm"""
        return (
            """
        <emotion dimension-set="http://www.w3.org/TR/emotion-voc/xml#pad-dimensions">{}
            <dimension name="arousal" value="0.3"/><!-- lower arousal -->
            <dimension name="pleasure" value="0.9"/><!-- high positive valence -->
            <dimension name="dominance" value="0.8"/><!-- high potency    -->
        </emotion>
            """.format(text)
        )

    def make_nervous(self, text):
        """Make the text nervous"""
        return (
            """
        <emotion dimension-set="http://www.w3.org/TR/emotion-voc/xml#pad-dimensions">{}
            <dimension name="arousal" value="0.9"/><!-- high arousal -->
            <dimension name="pleasure" value="0.2"/><!-- negative valence -->
            <dimension name="dominance" value="0.2"/><!-- low potency    -->
        </emotion>
            """.format(text)
        )

    def say(self, text, blocking=False, **kwargs):
        """Perform TTS using EmotionML"""
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
        wavfile = None

        try:
            r = requests.post(SoundClient.MARY_SERVER_URL, params=query_dict)
            if r.headers['content-type'] != 'audio/x-wav':
                rospy.logerr("Response Error Code: {}. Content: {}".format(r.status_code, r.content))
                return

            wavfile = tempfile.NamedTemporaryFile(prefix='marytts', suffix='.wav')
            wavfile.write(r.content)
            wavfile.flush()

            # Now send the file's name over to sound play
            sound = SoundRequest()
            sound.sound = SoundRequest.PLAY_FILE
            sound.command = SoundRequest.PLAY_ONCE
            sound.arg = wavfile.name
            self._play(sound, blocking=blocking, **kwargs)

        finally:
            # We're done, so delete the file
            if wavfile is not None:
                wavfile.close()

    def beep(self, key, blocking=False, **kwargs):
        """Play one of the beeps and boops that we know of"""
        if key not in self.beep_keys:
            return

        sound = SoundRequest()
        sound.sound = SoundRequest.PLAY_FILE
        sound.command = SoundRequest.PLAY_ONCE
        sound.arg = self.beep_keys[key]
        self._play(sound, blocking=blocking, **kwargs)

    def stop(self):
        """Stop all sounds"""
        self.sound_client.cancel_all_goals()

        # Send a stop request
        sound = SoundRequest()
        sound.sound = SoundRequest.ALL
        sound.command = SoundRequest.PLAY_STOP
        sound.arg = ""

        self._play(sound)

    def get_state(self):
        """Returns the state of the action client"""
        return self.sound_client.get_state()

    def get_result(self):
        """Returns the result of the last sound action"""
        return self.sound_client.get_result()

    def _play(self, sound, blocking, **kwargs):
        # Need to connect to the server
        rospy.logdebug("Connecting to sound_play...")
        self.sound_client.wait_for_server()
        rospy.logdebug("...sound_play connected")

        # Send the command
        rospy.logdebug("Sending sound action with (sound, command, arg): {}, {}, {}"
                       .format(sound.sound, sound.command, sound.arg))
        goal = SoundRequestGoal(sound_request=sound)
        self.sound_client.send_goal(goal)
        if blocking:
            self.sound_client.wait_for_result()
            rospy.logdebug('Response to sound action received')


if __name__ == '__main__':
    # For testing purposes
    rospy.init_node('speaker_test')
    client = SoundClient()

    def on_speak(args):
        text = args.text
        if args.happy:
            text = client.make_happy(text)
        if args.sad:
            text = client.make_sad(text)
        if args.angry:
            text = client.make_angry(text)
        if args.calm:
            text = client.make_calm(text)
        if args.nervous:
            text = client.make_nervous(text)

        client.say(text, blocking=True)

    def on_beep(args):
        client.beep(args.key, blocking=True)

    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers()

    speak_parser = subparsers.add_parser("speak", help="get the robot to speak")
    speak_parser.add_argument("text", help="text to speak")
    speak_parser.add_argument("--happy", action="store_true",
                              help="say things in a happy voice")
    speak_parser.add_argument("--sad", action="store_true",
                              help="Say things in a sad voice")
    speak_parser.add_argument("--angry", action="store_true",
                              help="Say things in an angry voice")
    speak_parser.add_argument("--calm", action="store_true",
                              help="Say things in a calm voice")
    speak_parser.add_argument("--nervous", action="store_true",
                              help="Say things in a nervous voice")
    speak_parser.set_defaults(func=on_speak)

    beep_parser = subparsers.add_parser("beep", help="get the robot to beep")
    beep_choices = [
        SoundClient.SOUND_SAD, SoundClient.SOUND_HAPPY, SoundClient.SOUND_SHOCKED, SoundClient.SOUND_UNSURE,
    ]
    beep_parser.add_argument("key", help="type of beep to produce",
                             choices=beep_choices)
    beep_parser.set_defaults(func=on_beep)

    args = parser.parse_args()
    args.func(args)
