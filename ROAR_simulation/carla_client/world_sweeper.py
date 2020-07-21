""""
WARNING: running this file will wipe ALL actors in the world!!!
"""
from pathlib import Path
import os, sys

import carla


def wipe_all_actors(host, port):
    client = carla.Client(host, port)
    all_actors = client.get_world().get_actors()
    for actor in all_actors:
        actor.destroy()

