"""Session management for experiment tracking and logging.

This module provides the Session class which coordinates CSV logging and
real-time reward plotting for RL experiments.
"""

from typing import Dict, List, Any, Optional
from fep_rl_experiment.logger import Logger
from fep_rl_experiment.plot import RewardPlotter


class Session:
    """Manages experiment logging and real-time reward visualization.

    This class combines CSV logging via Logger and real-time plotting via
    RewardPlotter. It maintains cumulative step counts across trajectories
    and handles resumption from existing log files.

    Attributes:
        logger: Logger instance for CSV output.
        ongoing_steps: Cumulative step counter across all trajectories.
        plotter: RewardPlotter instance for visualization.
    """

    def __init__(
        self,
        directory: str = ".",
        filename_prefix: str = "log",
        filename: Optional[str] = None
    ) -> None:
        """Initialize the session with logging and plotting.

        If an existing log file is found, loads previous data and updates
        the plot to show historical rewards.

        Args:
            directory: Directory to store log files (default: current directory).
            filename_prefix: Prefix for generated log filenames (default: "log").
            filename: Explicit filename (without .csv extension). If None, generates
                timestamped filename using prefix.
        """
        self.logger = Logger(
            directory=directory, filename_prefix=filename_prefix, filename=filename
        )
        self.ongoing_steps = 0
        self.plotter = RewardPlotter()
        # Load existing data if any
        existing_data = self.logger.load_existing_data()
        for row in existing_data:
            try:
                reward = float(row["reward"])
                self.plotter.steps.append(int(row["ongoing_steps"]))
                self.plotter.rewards.append(reward)
            except (ValueError, KeyError):
                continue  # Skip malformed rows
        if len(existing_data) > 0:
            self.ongoing_steps = int(max(data["steps"] for data in existing_data))
        else:
            self.ongoing_steps = 0
        # Update plot with loaded data
        self.plotter.line.set_data(self.plotter.steps, self.plotter.rewards)
        self.plotter.ax.relim()
        self.plotter.ax.autoscale_view()
        self.plotter.fig.canvas.draw()
        self.plotter.fig.canvas.flush_events()

    def update(self, data: Dict[str, Any]) -> None:
        """Log a new trajectory's data and update cumulative step counter.

        Args:
            data: Dictionary containing at least "reward" (float) and "steps" (int).

        Raises:
            ValueError: If "reward" key is missing or not a float.
        """
        if "reward" not in data or not isinstance(data["reward"], float):
            raise ValueError("Data must include a float 'reward'.")
        self.ongoing_steps += data["steps"]
        data["ongoing_steps"] = self.ongoing_steps
        self.logger.append_row(data)
        # FIXME (yarden)
        # self.plotter.update(data)

    @property
    def steps(self) -> List[int]:
        """Get the list of step counts for all logged trajectories.

        Returns:
            List of cumulative step counts.
        """
        return self.plotter.steps
