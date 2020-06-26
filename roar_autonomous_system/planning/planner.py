from abc import ABC, abstractmethod
import logging


class Planer(ABC):
    def __init__(self):
        self.logger = logging.getLogger(__name__)

    @abstractmethod
    def sync(self):
        """
        Sync the current class with other modules it is involved with
        :return:
        """
        pass
