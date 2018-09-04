#!/usr/bin/env python
# Speech for Ava

from __future__ import print_function, division

import os
import argparse
import requests
import tempfile

import rospy
import rospkg

from sound_play.libsoundplay import SoundClient as ROSSoundClient


class SpeakClient(object):
    """
    A wrapper around the ROSSoundClient to help with speech and sound
    tasks. I would highly recommend using an asynchronous thread worker when
    calling any of the methods in this class. They are long running processes
    and blocking by default.

    To play a beep, you must refer to one of the constant keys in this
    package that begins with `SOUND_*`. By default, the sound files in `sounds`
    map to one of these keys.

    To speak, you can use SABLE/EmotionML/Plain Text syntax to specify text.
    The TTS interface expects a MaryTTS server running in the background. Since
    TTS might be a long running process, and the current implementation is
    blocking, it is advisable to call the speak interface through an
    asynchronous thread.
    """

    # Template EmotionML
    SPEECH_TEMPLATE = \
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

    def __init__(self, audio_keys=None):
        self.sound_client = ROSSoundClient(blocking=True)

        self.audio_keys = audio_keys
        if self.audio_keys is None:
            default_sound_path = os.path.join(
                rospkg.RosPack().get_path('local_strategy'),
                'sounds'
            )
            self.audio_keys = {
                SpeakClient.SOUND_SAD: os.path.join(default_sound_path, 'R2D2_sad.wav'),
                SpeakClient.SOUND_HAPPY: os.path.join(default_sound_path, 'R2D2_excited.wav'),
                SpeakClient.SOUND_UNSURE: os.path.join(default_sound_path, 'R2D2_unsure.wav'),
                SpeakClient.SOUND_SHOCKED: os.path.join(default_sound_path, 'R2D2_shocked.wav'),
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
        <dimension name="arousal" value="0.3"/><!-- lower-than-average arousal -->
        <dimension name="pleasure" value="0.9"/><!-- very high positive valence -->
        <dimension name="dominance" value="0.8"/><!-- relatively high potency    -->
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

    def say(self, text):
        text = SpeakClient.SPEECH_TEMPLATE.format(speech=text)
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
            r = requests.post(SpeakClient.MARY_SERVER_URL, params=query_dict)
            if r.headers['content-type'] != 'audio/x-wav':
                rospy.logerr("Response Error Code: {}. Content: {}".format(r.status_code, r.content))
                return

            wavfile = tempfile.NamedTemporaryFile(prefix='marytts', suffix='.wav')
            wavfile.write(r.content)
            wavfile.flush()

            # Now send the file's name over to sound play
            self.sound_client.waveSound(wavfile.name).play()

        finally:
            # We're done, so delete the file
            if wavfile is not None:
                wavfile.close()

    def beep(self, key):
        if key not in self.audio_keys:
            return

        self.sound_client.waveSound(self.audio_keys[key]).play()


if __name__ == '__main__':
    # For testing purposes
    rospy.init_node('speaker_test')
    client = SpeakClient()

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

        client.say(text)

    def on_beep(args):
        client.beep(args.key)

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
        SpeakClient.SOUND_SAD, SpeakClient.SOUND_HAPPY, SpeakClient.SOUND_SHOCKED, SpeakClient.SOUND_UNSURE,
    ]
    beep_parser.add_argument("key", help="type of beep to produce",
                             choices=beep_choices)
    beep_parser.set_defaults(func=on_beep)

    args = parser.parse_args()
    args.func(args)
