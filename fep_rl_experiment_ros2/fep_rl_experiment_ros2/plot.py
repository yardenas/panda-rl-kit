"""Real-time reward visualization for RL experiments.

This module provides a RewardPlotter class that creates interactive
matplotlib plots for monitoring training progress.
"""

from typing import Dict, List, Any
import matplotlib.pyplot as plt


class RewardPlotter:
    """Interactive real-time reward plotter using matplotlib.

    Creates an interactive plot window that updates in real-time as rewards
    are received. Uses matplotlib's interactive mode (ion).

    Attributes:
        rewards: List of reward values to plot.
        steps: List of step counts (x-axis values).
        fig: Matplotlib figure object.
        ax: Matplotlib axes object.
        line: Matplotlib line object for the reward curve.
    """

    def __init__(self) -> None:
        """Initialize the interactive reward plot.

        Sets up matplotlib in interactive mode and creates the plot window.
        """
        self.rewards: List[float] = []
        self.steps: List[int] = []
        # Set up the interactive plot
        plt.ion()
        self.fig, self.ax = plt.subplots()
        (self.line,) = self.ax.plot([], [], label="Reward")
        self.ax.set_xlabel("Steps")
        self.ax.set_ylabel("Reward")
        self.ax.set_title("Reward Over Time")
        self.ax.legend()
        self.fig.show()

    def update(self, data: Dict[str, Any]) -> None:
        """Update the plot with a new reward data point.

        Args:
            data: Dictionary containing "reward" (float) and "ongoing_steps" (int).

        Raises:
            ValueError: If "reward" key is missing or not a float.
        """
        if "reward" not in data:
            raise ValueError("Data must contain a 'reward' key.")
        if not isinstance(data["reward"], float):
            raise ValueError("'reward' must be a float.")
        self.steps.append(data["ongoing_steps"])
        self.rewards.append(data["reward"])
        self.line.set_data(self.steps, self.rewards)
        self.ax.relim()
        self.ax.autoscale_view()

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
