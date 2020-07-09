from abc import ABC, abstractmethod
import logging


class Detector(ABC):
    def __init__(self):
        self.logger = logging

    @abstractmethod
    def run_step(self):
        """
        This method needs to be called on every step

        Returns:

        """
        self.sync()
        return None

    @abstractmethod
    def sync(self):
        return None
