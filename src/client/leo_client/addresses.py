"""YAML address book of server endpoints, newest session first.

The robot resolves its DuckDNS name. The TXT record is the server's private
address and the A record is the global one. Each saved row is tried the same
way: local, then global. A literal ``--server`` IP skips the book.
"""

from __future__ import annotations

import logging
import os
import socket
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

from leo_protocol.duckdns import hostname, is_ipv4, local_ipv4_from_txt, lookup_txt

LOGGER = logging.getLogger("leo.client")

_HEADER = """\
# Server addresses learned from DuckDNS. Newest session is first.
# `domain` is the name this robot resolves. The update token stays on the server.
# `local` is tried before `global` for every row.
# `session` is the local time of the last successful handshake.
# List order is the dial order. A successful session moves that row to the top.
# Delete a row to stop trying it. This file is rewritten after each session.
"""
_FIELDS = ("ip", "local", "global", "session")


@dataclass(frozen=True)
class ServerAddress:
    local_ip: str
    global_ip: str
    session: str


@dataclass(frozen=True)
class Dial:
    ip: str
    local_ip: str
    global_ip: str


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

    def remember(self, local_ip: str, global_ip: str, domain: str | None = None, session: str | None = None) -> None:
        local_ip, global_ip = _pair_ips(local_ip, global_ip)
        if domain and not self.domain:
            self.domain = domain
        stamp = session or datetime.now().astimezone().isoformat(timespec="seconds")
        kept = [item for item in self.servers if (item.local_ip, item.global_ip) != (local_ip, global_ip)]
        self.servers = [ServerAddress(local_ip, global_ip, stamp), *kept]
        self.save()

    def save(self) -> None:
        lines = [_HEADER.rstrip("\n"), f"domain: {_quote(self.domain)}", "servers:"]
        if not self.servers:
            lines[-1] = "servers: []"
        for item in self.servers:
            lines.append(f"  - local: {_quote(item.local_ip)}")
            lines.append(f"    global: {_quote(item.global_ip)}")
            lines.append(f"    session: {_quote(item.session)}")
        lines.append("")
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(self.path.suffix + ".tmp")
        temporary.write_text("\n".join(lines), encoding="utf-8")
        temporary.replace(self.path)

    def dial_order(self, resolved: ServerAddress | None) -> list[Dial]:
        rows: list[ServerAddress] = []
        if resolved is not None and (resolved.local_ip or resolved.global_ip):
            rows.append(resolved)
        rows.extend(self.servers)
        attempts: list[Dial] = []
        seen: set[str] = set()
        for row in rows:
            for ip in _row_ips(row):
                if ip in seen:
                    continue
                seen.add(ip)
                attempts.append(Dial(ip, row.local_ip, row.global_ip))
        return attempts


def known_local_first(attempts: list[Dial], book: AddressBook) -> list[Dial]:
    """Try the first saved local address before the other targets.

    At startup that address is the last successful connection.
    """
    if not book.servers or not book.servers[0].local_ip:
        return list(attempts)
    row = book.servers[0]
    known = Dial(row.local_ip, row.local_ip, row.global_ip)
    rest = [item for item in attempts if item.ip != row.local_ip]
    return [known, *rest]


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


def resolve_server(host: str) -> ServerAddress:
    """Return the TXT address as local and the A record as global."""
    local_ip = ""
    global_ip = ""
    errors: list[str] = []
    try:
        global_ip = resolve_ipv4(host)
    except OSError as exc:
        errors.append(str(exc))
    try:
        local_ip = local_ipv4_from_txt(lookup_txt(host))
    except OSError as exc:
        errors.append(str(exc))
    if not local_ip and not global_ip:
        detail = "; ".join(errors) if errors else host
        raise OSError(f"could not resolve {host}: {detail}")
    return ServerAddress(local_ip, global_ip, "")


def plan_targets(server: str, book: AddressBook, resolve=resolve_server) -> tuple[list[Dial], str | None]:
    """Return dial attempts, and the DuckDNS hostname when the book is in use.

    A literal IPv4 ``server`` is used alone. The second value is then ``None``.
    """
    chosen = server.strip()
    if chosen and is_ipv4(chosen):
        return [Dial(chosen, "", "")], None
    raw_domain = chosen or book.domain
    if not raw_domain.strip():
        raise RuntimeError("pass --server or set domain in data/server_ips.yaml")
    host = hostname(raw_domain)
    resolved = _resolve_row(host, resolve)
    attempts = book.dial_order(resolved)
    if not attempts:
        raise RuntimeError(f"no server IP for {host}, and {book.path} has no saved addresses")
    return attempts, host


def _resolve_row(host: str, resolve) -> ServerAddress | None:
    try:
        resolved = resolve(host)
    except OSError as exc:
        LOGGER.warning("duckdns resolve failed for %s: %s", host, exc)
        return None
    _require_ipv4(host, "local", resolved.local_ip)
    _require_ipv4(host, "global", resolved.global_ip)
    return resolved


def _require_ipv4(host: str, label: str, value: str) -> None:
    if value and not is_ipv4(value):
        raise RuntimeError(f"{host} {label} address {value} is not IPv4")


def _pair_ips(local_ip: str, global_ip: str) -> tuple[str, str]:
    local_ip = local_ip.strip()
    global_ip = global_ip.strip()
    if local_ip and not is_ipv4(local_ip):
        raise ValueError(f"not an IPv4 address: {local_ip}")
    if global_ip and not is_ipv4(global_ip):
        raise ValueError(f"not an IPv4 address: {global_ip}")
    if not local_ip and not global_ip:
        raise ValueError("server address is empty")
    return local_ip, global_ip


def _row_ips(row: ServerAddress) -> list[str]:
    ordered: list[str] = []
    if row.local_ip:
        ordered.append(row.local_ip)
    if row.global_ip and row.global_ip not in ordered:
        ordered.append(row.global_ip)
    return ordered


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
        servers.append(_row_from(current, path))
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
            current = {"local": "", "global": "", "ip": "", "session": ""}
            body = stripped[2:].strip()
            if body:
                key, value = _pair(body, path, line_number)
                current[key] = value
            continue
        if current is None:
            raise ValueError(f"{path}:{line_number}: list item must start with '- local:'")
        key, value = _pair(stripped, path, line_number)
        current[key] = value
    finish()
    if not seen and text.strip():
        raise ValueError(f"{path}: expected domain and servers")
    return domain, servers


def _row_from(current: dict[str, str], path: Path) -> ServerAddress:
    local_ip = current.get("local", "")
    global_ip = current.get("global", "") or current.get("ip", "")
    session = current.get("session", "")
    if local_ip and not is_ipv4(local_ip):
        raise ValueError(f"{path}: not an IPv4 address: {local_ip!r}")
    if global_ip and not is_ipv4(global_ip):
        raise ValueError(f"{path}: not an IPv4 address: {global_ip!r}")
    if not local_ip and not global_ip:
        raise ValueError(f"{path}: a server row needs local or global")
    if not session:
        address = local_ip or global_ip
        raise ValueError(f"{path}: {address} is missing session")
    return ServerAddress(local_ip, global_ip, session)


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
    if key not in _FIELDS:
        raise ValueError(f"{path}:{line_number}: unknown field {key!r}")
    return key, _unquote(value.strip())


def _unquote(value: str) -> str:
    if len(value) >= 2 and value[0] == value[-1] and value[0] in "\"'":
        return value[1:-1]
    return value


def _quote(value: str) -> str:
    escaped = value.replace("\\", "\\\\").replace('"', '\\"')
    return f'"{escaped}"'
