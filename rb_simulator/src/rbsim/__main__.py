"""Command-line entry point for `python -m rbsim`."""

from .server import main

if __name__ == "__main__":
    raise SystemExit(main())
