# build an abstract class for defining a learning problem
import numpy as np
import numpy.typing as npt
import abc
from dataclasses import dataclass
from typing import List, Set, Tuple

# define input and output types
Input = npt.NDArray[np.float64]
Output = npt.NDArray[np.float64]

class Problem:
    @abc.abstractmethod
    def get_training_data(self):
        """Labeled data for training."""

    @abc.abstractmethod
    def get_evaluation_data(self):
        """Held-out labeled data for evaluation."""

    @abc.abstractmethod
    def get_name(self):
        """Name of the problem."""

    @abc.abstractmethod
    def get_test_data(self):
        """Test data for testing."""

    @abc.abstractmethod
    def get_balance_ratio(self):
        """Balance ratio for the data."""