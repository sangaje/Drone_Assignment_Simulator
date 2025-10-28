from abc import ABC, abstractmethod


class Simulator(ABC):

    def __init__(self):
        pass

    def run(self):
        pass

    def results(self):
        pass


    @abstractmethod
    def init_data(self):
        """Initialize simulation data and state.

        This method should be implemented by subclasses to define how the
        simulation data and initial state are set up before running the simulation.
        """
        pass

    @abstractmethod
    def init_vehicles(self):
        """Initialize vehicles in the simulation.

        This method should be implemented by subclasses to define how the
        vehicles are created and configured for the simulation.
        """
        pass

    @abstractmethod
    def sim_update(self, dt):
        """Update the simulation state by a time step.

        This method should be implemented by subclasses to define how the
        simulation state is updated for each time step.

        Args:
            dt: Time step duration for this update cycle.
        """
        pass

    @abstractmethod
    def sim_post_update(self, dt):
        """Perform any necessary operations after the simulation update.

        This method should be implemented by subclasses to define any
        post-update operations that need to be performed after the main
        simulation update.

        Args:
            dt: Time step duration for this update cycle.
        """
        pass
