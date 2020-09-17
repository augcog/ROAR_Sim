#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jun 25 10:44:24 2017

@author: wroscoe
"""

import time
from statistics import median
from threading import Thread
from prettytable import PrettyTable
from ROAR_Jetson.memory import Memory
from ROAR_Jetson.jetson_cmd_sender import JetsonCommandSender
import logging

class Vehicle:
    def __init__(self):
        self.parts = []
        self.on = True
        self.threads = []
        self.logger = logging.getLogger("Jetson Vehicle")

    def add(self, part, inputs=[], outputs=[],
            threaded=False, run_condition=None):
        """
        Method to add a part to the vehicle drive loop.

        Parameters
        ----------
            inputs : list
                Channel names to get from memory.
            ouputs : list
                Channel names to save to memory.
            threaded : boolean
                If a part should be run in a separate thread.
        """
        assert type(inputs) is list, "inputs is not a list: %r" % inputs
        assert type(outputs) is list, "outputs is not a list: %r" % outputs
        assert type(threaded) is bool, "threaded is not a boolean: %r" % threaded

        p = part
        self.logger.info('Adding part {}.'.format(p.__class__.__name__))
        entry = {}
        entry['part'] = p
        entry['inputs'] = inputs
        entry['outputs'] = outputs
        entry['run_condition'] = run_condition

        if threaded:
            t = Thread(target=part.update, args=())
            t.daemon = True
            entry['thread'] = t

        self.parts.append(entry)

    def remove(self, part):
        """
        remove part form list
        """
        self.parts.remove(part)

    def start(self, rate_hz=10, max_loop_count=None, verbose=False):
        """
        Start vehicle's main drive loop.

        This is the main thread of the vehicle. It starts all the new
        threads for the threaded parts then starts an infinite loop
        that runs each part and updates the memory.

        Args:
            verbose:

            rate_hz : int, The max frequency that the drive loop should run. The actual
            frequency may be less than this if there are many blocking parts.

            max_loop_count : int, Maximum number of loops the drive loop should execute. This is
            used for testing that all the parts of the vehicle work.
        """

        try:
            self.on = True
            for entry in self.parts:
                if entry.get('thread'):
                    # start the update thread
                    entry.get('thread').start()

            # wait until the parts warm up.
            self.logger.info('Starting vehicle...')

            loop_count = 0
            while self.on:
                start_time = time.time()
                loop_count += 1

                self.update_parts()
                # stop drive loop if loop_count exceeds max_loopcount
                if max_loop_count and loop_count > max_loop_count:
                    self.on = False

                sleep_time = 1.0 / rate_hz - (time.time() - start_time)
                if sleep_time > 0.0:
                    time.sleep(sleep_time)
                else:
                    # print a message when could not maintain loop rate.
                    if verbose:
                        print('WARN::Vehicle: jitter violation in vehicle loop with value:', abs(sleep_time))
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

    def update_parts(self, new_throttle: float = 0, new_steering: float = 0):
        """
        loop over all parts
        """
        for entry in self.parts:
            p = entry["part"]
            if entry.get('thread') and isinstance(p, JetsonCommandSender):
                # send the throttle and steering to Arduino
                p.run_threaded(throttle=new_throttle, steering=new_steering)
            else:
                self.logger.error(f"Unknown part [{p}]")

    def stop(self):
        self.logger.debug('\n\nShutting down vehicle and its parts...')
        for entry in self.parts:
            try:
                entry['part'].shutdown()
            except AttributeError:
                # usually from missing shutdown method, which should be optional
                pass
            except Exception as e:
                print(e)
