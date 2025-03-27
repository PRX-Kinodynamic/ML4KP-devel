# build an abstract class for defining a learning problem
import abc

class Problem:
    @abc.abstractmethod
    def get_training_data(self):
        """Labeled data for training."""

    @abc.abstractmethod
    def get_evaluation_data(self):
        """Held-out labeled data for evaluation."""
        
    # @abc.abstractmethod
    # def get_test_data(self):
    #     """Test data for testing."""

    @abc.abstractmethod
    def get_name(self):
        """Name of the problem."""

    
