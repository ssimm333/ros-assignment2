"""Lightweight behavior tree primitives used by the mission controller."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
from typing import Callable, Iterable, List, Optional


class Status(Enum):
    SUCCESS = auto()
    FAILURE = auto()
    RUNNING = auto()


class Behaviour:
    """Base behavior with optional lifecycle hooks."""

    def __init__(self, name: str) -> None:
        self.name = name
        self._started = False

    def initialise(self) -> None:
        """Called once before first update when not already running."""

    def update(self) -> Status:
        raise NotImplementedError

    def terminate(self, new_status: Status) -> None:
        """Called when behavior leaves RUNNING state."""

    def tick(self) -> Status:
        if not self._started:
            self.initialise()
            self._started = True

        status = self.update()

        if status != Status.RUNNING:
            self.terminate(status)
            self._started = False

        return status


class Sequence(Behaviour):
    """Ticks children in order until one fails or runs."""

    def __init__(self, name: str, children: Iterable[Behaviour]) -> None:
        super().__init__(name)
        self.children: List[Behaviour] = list(children)
        self.index = 0

    def initialise(self) -> None:
        self.index = 0

    def update(self) -> Status:
        while self.index < len(self.children):
            child_status = self.children[self.index].tick()
            if child_status == Status.SUCCESS:
                self.index += 1
                continue
            return child_status
        return Status.SUCCESS


class Fallback(Behaviour):
    """Ticks children in order until one succeeds or runs."""

    def __init__(self, name: str, children: Iterable[Behaviour]) -> None:
        super().__init__(name)
        self.children: List[Behaviour] = list(children)
        self.index = 0

    def initialise(self) -> None:
        self.index = 0

    def update(self) -> Status:
        while self.index < len(self.children):
            child_status = self.children[self.index].tick()
            if child_status == Status.FAILURE:
                self.index += 1
                continue
            return child_status
        return Status.FAILURE


class Condition(Behaviour):
    """Boolean condition node."""

    def __init__(self, name: str, predicate: Callable[[], bool]) -> None:
        super().__init__(name)
        self.predicate = predicate

    def update(self) -> Status:
        return Status.SUCCESS if self.predicate() else Status.FAILURE


class Action(Behaviour):
    """Single-step action node."""

    def __init__(self, name: str, fn: Callable[[], Status]) -> None:
        super().__init__(name)
        self.fn = fn

    def update(self) -> Status:
        return self.fn()


class TimeoutDecorator(Behaviour):
    """Fails child if it exceeds a timeout in seconds."""

    def __init__(self, name: str, child: Behaviour, now_fn: Callable[[], float], timeout_s: float) -> None:
        super().__init__(name)
        self.child = child
        self.now_fn = now_fn
        self.timeout_s = timeout_s
        self.start_time: Optional[float] = None

    def initialise(self) -> None:
        self.start_time = self.now_fn()

    def update(self) -> Status:
        assert self.start_time is not None
        if self.now_fn() - self.start_time > self.timeout_s:
            return Status.FAILURE
        return self.child.tick()


@dataclass
class Blackboard:
    mission_complete: bool = False
    last_event: str = ""
