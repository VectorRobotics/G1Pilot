"""Replan / goal-acceptance gating policies.

Each policy independently evaluates one skip condition and (optionally)
returns a log message. Compose multiple policies via ``ReplanPolicyChain``.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any, Iterable, Optional


@dataclass(frozen=True)
class ReplanContext:
    """Read-only snapshot passed to each policy.

    ``now`` is captured once by the caller so cooldown policies share a
    consistent timestamp.
    """
    node: Any
    now: float


@dataclass(frozen=True)
class Skip:
    """Decision returned by a policy that wants to skip.

    ``message`` is the log line (None or empty for a silent skip).
    ``level`` selects the logger method ('debug', 'info', 'warn').
    """
    message: Optional[str] = None
    level: str = 'info'


class ReplanPolicy(ABC):
    """Base class for a single skip-condition check."""

    @abstractmethod
    def evaluate(self, ctx: ReplanContext) -> Optional[Skip]:
        """Return ``Skip(...)`` to skip; ``None`` to pass through."""


class NoActiveGoal(ReplanPolicy):
    def evaluate(self, ctx):
        if ctx.node._active_goal_handle is None:
            return Skip()
        return None


class EEPoseUnavailable(ReplanPolicy):
    def evaluate(self, ctx):
        node = ctx.node
        ee = node.left_ee_pose if node._left_arm else node.right_ee_pose
        if ee is None:
            return Skip(message='Replan skipped — ee poses not available')
        return None


class TargetPoseUnavailable(ReplanPolicy):
    def evaluate(self, ctx):
        if ctx.node.last_goal is None:
            return Skip(message='Replan skipped — target pose not available')
        return None


class EECloseToGoal(ReplanPolicy):
    def evaluate(self, ctx):
        node = ctx.node
        dist, _ = node._compute_errors(node._left_arm)
        threshold = node.get_parameter('replan_skip_distance').value
        if dist < threshold:
            return Skip(
                message=(f'Replan skipped — EE within {dist:.4f}m '
                         f'(< {threshold}m)'),
            )
        return None


class InPostGoalCooldown(ReplanPolicy):
    def evaluate(self, ctx):
        since = ctx.now - ctx.node._goal_completed_time
        cooldown = ctx.node.get_parameter('post_goal_cooldown').value
        if since < cooldown:
            return Skip(
                message=(f'Replan skipped — post-goal cooldown '
                         f'{since:.2f}/{cooldown}s'),
            )
        return None


class InReplanCooldown(ReplanPolicy):
    def evaluate(self, ctx):
        since = ctx.now - ctx.node._last_replan_time
        cooldown = ctx.node.get_parameter('replan_cooldown').value
        if since < cooldown:
            return Skip(
                message=(f'Replan skipped — replan cooldown '
                         f'{since:.2f}/{cooldown}s'),
            )
        return None


class ReplanPolicyChain:
    """Run policies in order; first one to fire wins."""

    def __init__(self, policies: Iterable[ReplanPolicy]):
        self._policies = list(policies)

    def should_skip(self, ctx: ReplanContext, logger) -> bool:
        for policy in self._policies:
            decision = policy.evaluate(ctx)
            if decision is None:
                continue
            if decision.message:
                log_fn = getattr(logger, decision.level, logger.info)
                log_fn(decision.message)
            return True
        return False
