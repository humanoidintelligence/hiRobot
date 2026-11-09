"""Map joint vectors between different joint orderings, by name.

Different robots and policies order their joints differently, and a policy may
act on a subset of a robot's joints. :class:`DofMap` builds an index mapping
from a source joint ordering to a target ordering using joint *names*, so a
value vector can be reordered without the caller tracking indices. This is the
mechanism that lets one policy drive robots with different joint layouts.
"""

from __future__ import annotations

from typing import List, Optional, Sequence

import numpy as np


class DofMap:
    """A by-name reordering between two joint orderings.

    Args:
        source_names: Joint names in the order the source vector uses.
        target_names: Joint names in the order the target vector should use.

    Raises:
        ValueError: If a target name is missing from the source ordering.
    """

    def __init__(self, source_names: Sequence[str], target_names: Sequence[str]) -> None:
        self.source_names: List[str] = list(source_names)
        self.target_names: List[str] = list(target_names)

        src_index = {name: i for i, name in enumerate(self.source_names)}
        missing = [n for n in self.target_names if n not in src_index]
        if missing:
            raise ValueError(f"Target joints missing from source ordering: {missing}")
        self._index = np.array([src_index[n] for n in self.target_names], dtype=int)

    def remap(self, values: np.ndarray) -> np.ndarray:
        """Reorder a source-ordered vector into target order.

        Args:
            values: Array whose first axis is indexed by ``source_names``.

        Returns:
            Array reordered so its first axis follows ``target_names``.
        """
        values = np.asarray(values)
        if values.shape[0] != len(self.source_names):
            raise ValueError(
                f"Expected first axis of length {len(self.source_names)}, "
                f"got {values.shape[0]}."
            )
        return values[self._index]

    def scatter(
        self,
        values: np.ndarray,
        default: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """Place target-ordered values back into the source ordering.

        Joints in the source ordering that are not in the target ordering take
        their value from ``default`` (or 0 if not given). This is the inverse of
        :meth:`remap` for the shared joints — useful when a policy commands only
        a subset of a robot's joints.

        Args:
            values: Array whose first axis follows ``target_names``.
            default: Optional source-ordered array of fallback values.

        Returns:
            Source-ordered array.
        """
        values = np.asarray(values)
        out_shape = (len(self.source_names),) + values.shape[1:]
        out = np.zeros(out_shape, dtype=values.dtype) if default is None else np.array(default)
        out[self._index] = values
        return out
