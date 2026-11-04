"""Robot entry point.

Pass ``--server`` as an IP to dial that host directly. Pass a DuckDNS name,
or omit it and set ``domain`` in ``data/server_ips.yaml``. The client tries
each server's private address, then its global address, and remembers both.
"""

from __future__ import annotations

import argparse
import logging
import signal
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
    parser.add_argument("--server", default="", help="server IP, or DuckDNS name. Default: domain in the address book")
    parser.add_argument("--addresses", default="", help="yaml address book. Default: data/server_ips.yaml")
    parser.add_argument("--sim", default="", help="run the mock robot in this world")
    parser.add_argument("--show", action="store_true", help="show the mock top-down window")
    parser.add_argument("--route", action="store_true", help="play the scripted route from the server config")
    parser.add_argument("--duration", type=float, default=0.0, help="stop after this many seconds")
    args = parser.parse_args()
    _configure_logging()
    signal.signal(signal.SIGINT, _end_session)
    signal.signal(signal.SIGTERM, _end_session)
    from leo_client.session import open_session

    try:
        open_session(args.server, args.sim, args.show, args.route, args.duration, args.addresses or None)
    except KeyboardInterrupt:
        from leo_protocol.console import shutdown_line, write

        write(shutdown_line())


def _configure_logging() -> None:
    if sys.stderr.isatty():
        logger = logging.getLogger("leo")
        logger.addHandler(logging.NullHandler())
        logger.propagate = False
        return
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s %(message)s")


def _end_session(signum: int, _frame: object) -> None:
    logging.getLogger("leo.client").info("signal %s, ending session", signum)
    raise KeyboardInterrupt


if __name__ == "__main__":
    main()
