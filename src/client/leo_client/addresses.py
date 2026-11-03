"""YAML address book of server IPs, newest session first.

The robot resolves its DuckDNS name, handshakes that IP, and rewrites this
file. If DNS is down it tries the saved addresses from top to bottom.
A literal ``--server`` IP skips the book, so the local simulator stays offline.
"""

from __future__ import annotations

import logging
import os
import socket
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

from leo_protocol.duckdns import hostname, is_ipv4

LOGGER = logging.getLogger("leo.client")

_HEADER = """\
# Server addresses learned from DuckDNS. Newest session is first.
# `domain` is the name this robot resolves. The update token stays on the server.
# `session` is the local time of the last successful handshake.
# List order is the dial order. A successful session moves that IP to the top.
# Delete a row to stop trying it. This file is rewritten after each session.
"""


@dataclass(frozen=True)
class ServerAddress:
    ip: str
    session: str


class AddressBook:
    def __init__(self, path: Path, domain: str, servers: list[ServerAddress]) -> None:
        self.path = path
        self.domain = domain
        self.servers = list(servers)

    @classmethod
    def load(cls, path: Path) -> AddressBook:
        if not path.is_file():
            return cls(path, "", [])
        text = path.read_text(encoding="utf-8")
        domain, servers = _parse(text, path)
        return cls(path, domain, servers)

    def remember(self, ip: str, domain: str | None = None, session: str | None = None) -> None:
        if not is_ipv4(ip):
            raise ValueError(f"not an IPv4 address: {ip}")
        if domain and not self.domain:
            self.domain = domain
        stamp = session or datetime.now().astimezone().isoformat(timespec="seconds")
        kept = [item for item in self.servers if item.ip != ip]
        self.servers = [ServerAddress(ip, stamp), *kept]
        self.save()

    def save(self) -> None:
        lines = [_HEADER.rstrip("\n"), f"domain: {_quote(self.domain)}", "servers:"]
        if not self.servers:
            lines[-1] = "servers: []"
        for item in self.servers:
            lines.append(f"  - ip: {_quote(item.ip)}")
            lines.append(f"    session: {_quote(item.session)}")
        lines.append("")
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(self.path.suffix + ".tmp")
        temporary.write_text("\n".join(lines), encoding="utf-8")
        temporary.replace(self.path)

    def dial_order(self, resolved: str | None) -> list[str]:
        ordered: list[str] = []
        if resolved:
            ordered.append(resolved)
        for item in self.servers:
            if item.ip not in ordered:
                ordered.append(item.ip)
        return ordered


def default_book_path() -> Path:
    env = os.environ.get("LEO_SERVER_IPS", "").strip()
    if env:
        return Path(env)
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "config" / "robot.yaml").is_file():
            return parent / "data" / "server_ips.yaml"
    return Path.home() / ".leo" / "server_ips.yaml"


def resolve_ipv4(host: str) -> str:
    try:
        infos = socket.getaddrinfo(host, None, socket.AF_INET, socket.SOCK_STREAM)
    except socket.gaierror as exc:
        raise OSError(f"could not resolve {host}") from exc
    if not infos:
        raise OSError(f"could not resolve {host}")
    return str(infos[0][4][0])


def plan_targets(server: str, book: AddressBook, resolve=resolve_ipv4) -> tuple[list[str], str | None]:
    """Return IPs to try, and the DuckDNS hostname when the book is in use.

    A literal IPv4 ``server`` is used alone. The second value is then ``None``.
    """
    chosen = server.strip()
    if chosen and is_ipv4(chosen):
        return [chosen], None
    raw_domain = chosen or book.domain
    if not raw_domain.strip():
        raise RuntimeError("pass --server or set domain in data/server_ips.yaml")
    host = hostname(raw_domain)
    resolved = None
    try:
        resolved = resolve(host)
    except OSError as exc:
        LOGGER.warning("duckdns resolve failed for %s: %s", host, exc)
    if resolved is not None and not is_ipv4(resolved):
        raise RuntimeError(f"{host} resolved to {resolved}, which is not IPv4")
    targets = book.dial_order(resolved)
    if not targets:
        raise RuntimeError(f"no server IP for {host}, and {book.path} has no saved addresses")
    return targets, host


def _parse(text: str, path: Path) -> tuple[str, list[ServerAddress]]:
    domain = ""
    servers: list[ServerAddress] = []
    current: dict[str, str] | None = None
    in_servers = False
    seen = False

    def finish() -> None:
        nonlocal current
        if current is None:
            return
        ip = current.get("ip", "")
        session = current.get("session", "")
        if not is_ipv4(ip):
            raise ValueError(f"{path}: not an IPv4 address: {ip!r}")
        if not session:
            raise ValueError(f"{path}: {ip} is missing session")
        servers.append(ServerAddress(ip, session))
        current = None

    for line_number, raw in enumerate(text.splitlines(), start=1):
        stripped = _strip_comment(raw).strip()
        if not stripped:
            continue
        if stripped.startswith("domain:"):
            finish()
            in_servers = False
            domain = _unquote(stripped.split(":", 1)[1].strip())
            seen = True
            continue
        if stripped.startswith("servers:"):
            finish()
            rest = stripped.split(":", 1)[1].strip()
            if rest not in ("", "[]"):
                raise ValueError(f"{path}:{line_number}: write servers as a list")
            in_servers = True
            seen = True
            continue
        if not in_servers:
            raise ValueError(f"{path}:{line_number}: unexpected line")
        if stripped.startswith("- "):
            finish()
            current = {"ip": "", "session": ""}
            body = stripped[2:].strip()
            if body:
                key, value = _pair(body, path, line_number)
                current[key] = value
            continue
        if current is None:
            raise ValueError(f"{path}:{line_number}: list item must start with '- ip:'")
        key, value = _pair(stripped, path, line_number)
        current[key] = value
    finish()
    if not seen and text.strip():
        raise ValueError(f"{path}: expected domain and servers")
    return domain, servers


def _strip_comment(line: str) -> str:
    in_quote = ""
    for index, char in enumerate(line):
        if char in "\"'" and (index == 0 or line[index - 1] != "\\"):
            if in_quote == char:
                in_quote = ""
            elif not in_quote:
                in_quote = char
            continue
        if char == "#" and not in_quote:
            return line[:index]
    return line


def _pair(body: str, path: Path, line_number: int) -> tuple[str, str]:
    if ":" not in body:
        raise ValueError(f"{path}:{line_number}: expected key: value")
    key, value = body.split(":", 1)
    key = key.strip()
    if key not in ("ip", "session"):
        raise ValueError(f"{path}:{line_number}: unknown field {key!r}")
    return key, _unquote(value.strip())


def _unquote(value: str) -> str:
    if len(value) >= 2 and value[0] == value[-1] and value[0] in "\"'":
        return value[1:-1]
    return value


def _quote(value: str) -> str:
    escaped = value.replace("\\", "\\\\").replace('"', '\\"')
    return f'"{escaped}"'
