"""CSV logging utility for experiment data.

This module provides a simple CSV logger that writes experiment metrics
to timestamped files with automatic header management.
"""

import csv
import os
from typing import Dict, List, Any, Optional
from datetime import datetime


class Logger:
    """CSV logger for experiment metrics.

    Writes experiment data to CSV files with automatic timestamping and
    header management. The first call to append_row() establishes the
    schema, and subsequent calls must match those headers.

    Attributes:
        directory: Directory path for log files.
        filename: Name of the CSV file (with .csv extension).
        full_path: Absolute path to the log file.
        headers_written: Whether the CSV header has been written.
        headers: List of column names.
        logger: Optional ROS2 logger for log messages.
    """

    def __init__(
        self,
        directory: str = ".",
        filename_prefix: str = "log",
        filename: Optional[str] = None,
        logger: Optional[Any] = None
    ) -> None:
        """Initialize the logger with file path and create directory.

        Args:
            directory: Directory to store log files (default: current directory).
            filename_prefix: Prefix for timestamped filename (default: "log").
            filename: Explicit filename (without .csv extension). If None,
                generates timestamped filename: {prefix}_{timestamp}.csv.
            logger: Optional ROS2 logger instance for log messages.
        """
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.directory = directory
        if filename is None:
            self.filename = f"{filename_prefix}_{timestamp}.csv"
        else:
            self.filename = f"{filename}.csv"
        self.full_path = os.path.join(directory, self.filename)
        os.makedirs(os.path.dirname(self.full_path), exist_ok=True)
        self.headers_written = False
        self.headers: List[str] = []
        self.logger = logger
        if self.logger:
            self.logger.info(f"Logging to {os.path.abspath(self.full_path)}")
        else:
            print(f"Logging to {os.path.abspath(self.full_path)}")

    def append_row(self, data: Dict[str, Any]) -> None:
        """Append a row to the CSV log file.

        Automatically adds a timestamp column. The first call establishes the
        schema (headers), and subsequent calls must have matching keys.

        Args:
            data: Dictionary of metrics to log. Keys become column headers.

        Raises:
            ValueError: If keys don't match previously established headers.
        """
        data["timestamp"] = datetime.now().isoformat()
        if not self.headers_written:
            self.headers = list(data.keys())
            with open(self.full_path, mode="w", newline="") as file:
                writer = csv.DictWriter(file, fieldnames=self.headers)
                writer.writeheader()
                writer.writerow(data)
            self.headers_written = True
        else:
            if set(data.keys()) != set(self.headers):
                raise ValueError(
                    "Keys of the new data must match the original headers. The following keys are missing: "
                    + str(set(self.headers) - set(data.keys()))
                )
            with open(self.full_path, mode="a", newline="") as file:
                writer = csv.DictWriter(file, fieldnames=self.headers)
                writer.writerow(data)

    def load_existing_data(self) -> List[Dict[str, str]]:
        """Load all rows from an existing CSV file.

        If the file doesn't exist, returns an empty list. Sets headers_written
        flag if file exists.

        Returns:
            List of dictionaries, one per CSV row.
        """
        if not os.path.exists(self.full_path):
            return []
        with open(self.full_path, newline="") as file:
            reader = csv.DictReader(file)
            self.headers_written = True
            self.headers = list(reader.fieldnames)
            return [row for row in reader]
