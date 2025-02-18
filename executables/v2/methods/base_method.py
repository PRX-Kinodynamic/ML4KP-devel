import abc
from typing import List, Tuple, Set

class Method(abc.ABC):
  """A method for image classification."""

  @abc.abstractmethod
  def train(self, training_data, evaluation_data, balance_ratio, verbose=False):
    """Train the method given training data."""

  @abc.abstractmethod
  def predict(self, input_data):
    """Predict the class label for an image."""

  @abc.abstractmethod
  def get_name(self):
    """Name of the method."""