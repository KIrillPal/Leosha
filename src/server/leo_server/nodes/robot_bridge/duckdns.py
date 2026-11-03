"""Publish this machine's global and local IPv4 addresses to DuckDNS.

The A record is the public address DuckDNS sees, unless ``global_ip`` is set.
That override is where a NetBird address goes later. The TXT record is the
private address on the default route. The robot tries the TXT address first.
"""

from __future__ import annotations

import http.client
import os
import socket
import ssl
import threading
import urllib.parse
import urllib.request
from dataclasses import dataclass

from leo_protocol.duckdns import hostname, is_ipv4, is_private_ipv4, subdomain


class _IPv4HTTPSConnection(http.client.HTTPSConnection):
    def connect(self) -> None:
        infos = socket.getaddrinfo(self.host, self.port, socket.AF_INET, socket.SOCK_STREAM)
        if not infos:
            raise OSError(f"no IPv4 address for {self.host}")
        self.sock = socket.create_connection(infos[0][4], self.timeout)
        try:
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError:
            pass
        context = self._context or ssl.create_default_context()
        self.sock = context.wrap_socket(self.sock, server_hostname=self.host)


class _IPv4Handler(urllib.request.HTTPSHandler):
    def https_open(self, req):
        return self.do_open(_IPv4HTTPSConnection, req)


def _urlopen(request, timeout: float):
    return urllib.request.build_opener(_IPv4Handler()).open(request, timeout=timeout)


@dataclass(frozen=True)
class Published:
    global_ip: str
    local_ip: str


def lan_ipv4() -> str:
    """Return the private IPv4 of the default route, or an empty string."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.connect(("192.0.2.1", 9))
            address = sock.getsockname()[0]
    except OSError:
        return ""
    if not is_private_ipv4(address):
        return ""
    return address


def parse_reply(body: str) -> str:
    lines = [line.strip() for line in body.splitlines() if line.strip()]
    if not lines or lines[0] != "OK":
        status = lines[0] if lines else "empty reply"
        raise RuntimeError(f"DuckDNS rejected the update ({status})")
    if len(lines) < 2 or not is_ipv4(lines[1]):
        raise RuntimeError("DuckDNS reply did not include an IPv4 address")
    return lines[1]


class DuckDns:
    def __init__(
        self,
        domain: str,
        token: str,
        update_sec: float,
        log,
        opener=None,
        global_ip: str = "",
        local_ip=None,
    ) -> None:
        self.hostname = hostname(domain)
        self._subdomain = subdomain(domain)
        self._token = token.strip()
        if not self._token:
            raise ValueError("DuckDNS token is empty")
        if update_sec <= 0:
            raise ValueError("duckdns.update_sec must be positive")
        self._global_ip = global_ip.strip()
        if self._global_ip and not is_ipv4(self._global_ip):
            raise ValueError("duckdns.global_ip must be an IPv4 address or empty")
        self._update_sec = float(update_sec)
        self._log = log
        self._opener = opener or _urlopen
        self._local_ip = local_ip or lan_ipv4
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    @classmethod
    def start_from(cls, section: dict, log, local_ip: str = "") -> DuckDns | None:
        if not section["enabled"]:
            log.info("duckdns is disabled")
            return None
        token = os.environ.get("DUCKDNS_TOKEN", "").strip() or str(section["token"]).strip()
        domain = str(section["domain"]).strip()
        if not domain or not token:
            log.error(
                "duckdns is enabled but domain or token is empty; "
                "set duckdns.domain and duckdns.token, or DUCKDNS_TOKEN"
            )
            return None
        pinned = local_ip.strip()
        if pinned and not is_private_ipv4(pinned):
            log.error(f"duckdns local address {pinned} is not a private IPv4 address")
            return None

        def provider():
            return pinned

        if not pinned:
            provider = None
        try:
            service = cls(
                domain,
                token,
                float(section["update_sec"]),
                log,
                global_ip=str(section.get("global_ip", "")),
                local_ip=provider,
            )
        except ValueError as exc:
            log.error(str(exc))
            return None
        service.start()
        log.info(f"duckdns publishing {service.hostname} every {section['update_sec']}s")
        return service

    def start(self) -> None:
        self._thread = threading.Thread(target=self._loop, name="duckdns", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def publish_once(self) -> Published:
        local_ip = self._local_ip() or ""
        if local_ip and not is_private_ipv4(local_ip):
            raise ValueError(f"local IPv4 is not a private address: {local_ip}")
        fields = {"domains": self._subdomain, "token": self._token, "verbose": "true"}
        if self._global_ip:
            fields["ip"] = self._global_ip
        if local_ip:
            fields["txt"] = local_ip
        query = urllib.parse.urlencode(fields)
        request = urllib.request.Request(f"https://www.duckdns.org/update?{query}", method="GET")
        with self._opener(request, timeout=15) as response:
            body = response.read().decode("utf-8", errors="replace")
        return Published(parse_reply(body), local_ip)

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                published = self.publish_once()
            except Exception as exc:
                self._log.warning(f"duckdns update failed: {self._safe(exc)}")
            else:
                self._log.info(self._published(published))
            if self._stop.wait(self._update_sec):
                return

    def _published(self, published: Published) -> str:
        text = f"duckdns {self.hostname} global {published.global_ip}"
        if published.local_ip:
            text += f" local {published.local_ip}"
        return text

    def _safe(self, exc: Exception) -> str:
        text = f"{exc.__class__.__name__}: {exc}"
        if self._token:
            text = text.replace(self._token, "***")
        return text
