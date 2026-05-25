#!/usr/bin/env python3
"""Extensible log collection for CanSat2026.

Logger is intentionally independent from each device manager.  Any manager can
be logged as long as it exposes a function that returns a dict, for example
SensorManager.read_all() or a future MotorManager.read_all().
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Iterable


LogReader = Callable[[], dict[str, Any]]


@dataclass(frozen=True)
class LogSource:
    """One named log source such as sensors, motors, or communication."""

    name: str
    reader: LogReader
    required: bool = True


class Logger:
    """Collect timestamped logs from registered sources.

    Example:
        sensors = SensorManager()
        logger = Logger()
        logger.register_source("sensors", sensors.read_all)
        log = logger.get_log()
    """

    def __init__(
        self,
        log_dir: str | Path = "logs",
        node_id: str | None = None,
    ) -> None:
        self.log_dir = Path(log_dir)
        self.node_id = node_id
        self._sources: dict[str, LogSource] = {}

    def register_source(
        self,
        name: str,
        reader: LogReader,
        required: bool = True,
    ) -> None:
        """Register a log source.

        Args:
            name: Source name used as the key in get_log(), e.g. "sensors".
            reader: Function that returns a dict with current values.
            required: If False, errors are stored in the log instead of raised.
        """
        if not name:
            raise ValueError("log source name must not be empty")
        if not callable(reader):
            raise TypeError("log source reader must be callable")
        self._sources[name] = LogSource(name=name, reader=reader, required=required)

    def unregister_source(self, name: str) -> None:
        """Remove a registered source."""
        self._sources.pop(name, None)

    def get_log(self, sources: Iterable[str] | None = None) -> dict[str, Any]:
        """Return one timestamped log snapshot.

        The return value is JSON-serializable and keeps each subsystem under its
        own key so new sources can be added without changing existing readers.
        """
        now = datetime.now(timezone.utc)
        selected = self._select_sources(sources)
        payload: dict[str, Any] = {
            "timestamp": now.isoformat(timespec="milliseconds"),
            "timestamp_unix": now.timestamp(),
            "node_id": self.node_id,
            "data": {},
            "errors": {},
        }

        for source in selected:
            try:
                value = source.reader()
                if not isinstance(value, dict):
                    raise TypeError(f"{source.name} reader must return dict")
                payload["data"][source.name] = _json_safe(value)
            except Exception as exc:
                if source.required:
                    raise
                payload["errors"][source.name] = {
                    "type": type(exc).__name__,
                    "message": str(exc),
                }

        if not payload["errors"]:
            payload.pop("errors")
        if payload["node_id"] is None:
            payload.pop("node_id")
        return payload

    def get_source_log(self, name: str) -> dict[str, Any]:
        """Return a timestamped log containing only one source."""
        return self.get_log(sources=[name])

    def append_jsonl(
        self,
        log: dict[str, Any] | None = None,
        filename: str | None = None,
    ) -> Path:
        """Append a log snapshot as one JSON Lines record and return the path."""
        record = self.get_log() if log is None else log
        path = self._log_path(filename)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("a", encoding="utf-8") as file:
            file.write(json.dumps(record, ensure_ascii=True, separators=(",", ":")))
            file.write("\n")
        return path

    def _select_sources(self, names: Iterable[str] | None) -> list[LogSource]:
        if names is None:
            return list(self._sources.values())

        selected: list[LogSource] = []
        for name in names:
            try:
                selected.append(self._sources[name])
            except KeyError as exc:
                raise KeyError(f"unknown log source: {name}") from exc
        return selected

    def _log_path(self, filename: str | None) -> Path:
        if filename:
            return self.log_dir / filename
        return self.log_dir / f"cansat_{datetime.now():%Y%m%d}.jsonl"


def _json_safe(value: Any) -> Any:
    """Convert common Python values to JSON-safe structures."""
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, Path):
        return str(value)
    return value
