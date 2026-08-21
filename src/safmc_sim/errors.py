"""Exception types.

The project rule is fail fast, fail early: every one of these represents a condition we
refuse to paper over. There is no exception type here for "recoverable" anything.
"""


class SafmcSimError(Exception):
    """Base for everything this package raises."""


class ConfigError(SafmcSimError):
    """A configuration value is impossible, contradictory, or silently lossy.

    Raised at construction time, never at step time.
    """


class ArenaError(SafmcSimError):
    """A generated arena violates a published constraint or is unusable.

    Raised by arena validation. An arena that fails validation would invalidate every run
    performed on it, so this is never downgraded to a warning.
    """


class PolicyError(SafmcSimError):
    """A policy misbehaved: raised, returned the wrong type, or reached for state it must not have."""


class RuleViolation(SafmcSimError):
    """A competition rule was broken in a way that invalidates the run (e.g. a third take-off wave)."""


class LogFormatError(SafmcSimError):
    """A recorded log is malformed or of an unsupported schema version."""
