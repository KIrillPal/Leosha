"""Publish this machine's public IPv4 address to DuckDNS.

The request goes out over IPv4 and omits an explicit address, so DuckDNS
records the address it sees. The robot resolves the same name.
"""

from __future__ import annotations

import http.client
import os
import socket
import ssl
import threading
import urllib.parse
import urllib.request

from leo_protocol.duckdns import hostname, is_ipv4, subdomain


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


def parse_reply(body: str) -> str:
    lines = [line.strip() for line in body.splitlines() if line.strip()]
    if not lines or lines[0] != "OK":
        status = lines[0] if lines else "empty reply"
        raise RuntimeError(f"DuckDNS rejected the update ({status})")
    if len(lines) < 2 or not is_ipv4(lines[1]):
        raise RuntimeError("DuckDNS reply did not include an IPv4 address")
    return lines[1]


class DuckDns:
    def __init__(self, domain: str, token: str, update_sec: float, log, opener=None) -> None:
        self.hostname = hostname(domain)
        self._subdomain = subdomain(domain)
        self._token = token.strip()
        if not self._token:
            raise ValueError("DuckDNS token is empty")
        if update_sec <= 0:
            raise ValueError("duckdns.update_sec must be positive")
        self._update_sec = float(update_sec)
        self._log = log
        self._opener = opener or _urlopen
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    @classmethod
    def start_from(cls, section: dict, log) -> DuckDns | None:
        if not section["enabled"]:
            log.info("duckdns is disabled")
            return None
        token = os.environ.get("DUCKDNS_TOKEN", "").strip() or str(section["token"]).strip()
        domain = str(section["domain"]).strip()
        if not domain or not token:
            log.error("duckdns is enabled but domain or token is empty; set duckdns.domain and duckdns.token, or DUCKDNS_TOKEN")
            return None
        try:
            service = cls(domain, token, float(section["update_sec"]), log)
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

    def publish_once(self) -> str:
        query = urllib.parse.urlencode(
            {"domains": self._subdomain, "token": self._token, "verbose": "true"}
        )
        request = urllib.request.Request(f"https://www.duckdns.org/update?{query}", method="GET")
        with self._opener(request, timeout=15) as response:
            body = response.read().decode("utf-8", errors="replace")
        return parse_reply(body)

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                address = self.publish_once()
            except Exception as exc:
                self._log.warning(f"duckdns update failed: {self._safe(exc)}")
            else:
                self._log.info(f"duckdns {self.hostname} -> {address}")
            if self._stop.wait(self._update_sec):
                return

    def _safe(self, exc: Exception) -> str:
        text = f"{exc.__class__.__name__}: {exc}"
        if self._token:
            text = text.replace(self._token, "***")
        return text
