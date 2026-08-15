#!/usr/bin/env python3
"""Backward-compatible alias for :mod:`vptele.main_scripted`."""

import sys
from pathlib import Path


if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from vptele.main_scripted import (  # noqa: F401
    DEFAULT_CONFIG,
    build_cli_parser,
    build_standalone_config,
    main,
    run_scripted_collection,
)


if __name__ == "__main__":
    raise SystemExit(main())
