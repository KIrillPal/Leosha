"""Robot entry point. The only required argument is the server address."""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path


def _bootstrap() -> None:
    shared = Path(__file__).resolve().parents[2] / "shared"
    if str(shared) not in sys.path:
        sys.path.insert(0, str(shared))
    client = Path(__file__).resolve().parents[1]
    if str(client) not in sys.path:
        sys.path.insert(0, str(client))


def main() -> None:
    _bootstrap()
    parser = argparse.ArgumentParser(description="LEO2 robot client")
    parser.add_argument("--server", required=True, help="server hostname or IP")
    parser.add_argument("--sim", default="", help="run the mock robot in this world")
    parser.add_argument("--show", action="store_true", help="show the mock top-down window")
    parser.add_argument("--route", action="store_true", help="play the scripted route from the server config")
    parser.add_argument("--duration", type=float, default=0.0, help="stop after this many seconds")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s %(message)s")
    from leo_client.session import open_session

    open_session(args.server, args.sim, args.show, args.route, args.duration)


if __name__ == "__main__":
    main()
