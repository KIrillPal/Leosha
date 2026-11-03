"""DuckDNS name and IPv4 checks shared by the server and the robot."""

from __future__ import annotations

import json
import random
import socket
import struct
import urllib.parse
import urllib.request

_SUFFIX = ".duckdns.org"
_TXT = 16
_DOH = (
    "https://dns.google/resolve",
    "https://cloudflare-dns.com/dns-query",
)


def is_ipv4(value: str) -> bool:
    parts = value.split(".")
    if len(parts) != 4:
        return False
    for part in parts:
        if not part.isdigit() or (len(part) > 1 and part.startswith("0")):
            return False
        if int(part) > 255:
            return False
    return True


def hostname(domain: str) -> str:
    """Return ``name.duckdns.org`` from either a bare name or a full hostname."""
    raw = domain.strip().lower().rstrip(".")
    if not raw:
        raise ValueError("DuckDNS domain is empty")
    if raw.endswith(_SUFFIX):
        label = raw[: -len(_SUFFIX)]
    else:
        label = raw
    if "." in label or not _label(label):
        raise ValueError(f"not a DuckDNS name: {domain}")
    return label + _SUFFIX


def subdomain(domain: str) -> str:
    """Return the label DuckDNS expects in the update request."""
    host = hostname(domain)
    return host[: -len(_SUFFIX)]


def is_private_ipv4(value: str) -> bool:
    """Return whether ``value`` is an RFC1918 address the robot can try on the LAN."""
    if not is_ipv4(value):
        return False
    first, second, _third, _fourth = (int(part) for part in value.split("."))
    if first == 10 or (first == 192 and second == 168):
        return True
    return first == 172 and 16 <= second <= 31


def local_ipv4_from_txt(text: str) -> str:
    """Return the first IPv4 address in a DuckDNS TXT record, or an empty string."""
    cleaned = text.replace(",", " ").replace('"', " ")
    for token in cleaned.split():
        if is_ipv4(token):
            return token
    return ""


def lookup_txt(host: str, timeout: float = 3.0) -> str:
    """Return the TXT data for ``host``, or raise ``OSError`` when DNS fails.

    The system resolver is asked first. Some resolvers answer TXT with an empty
    set, so a public DNS-over-HTTPS server is used when that answer has no IPv4.
    """
    try:
        text = _lookup_txt_udp(host, timeout)
    except OSError:
        text = ""
    if local_ipv4_from_txt(text):
        return text
    try:
        https = _lookup_txt_https(host, timeout)
    except OSError as exc:
        if text:
            return text
        raise OSError(f"txt lookup {host} failed") from exc
    return https or text


def _lookup_txt_udp(host: str, timeout: float) -> str:
    query_id = random.randrange(65536)
    message = _dns_query(host, query_id)
    server = _nameserver()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.settimeout(timeout)
        try:
            sock.sendto(message, (server, 53))
            payload, _source = sock.recvfrom(2048)
        except OSError as exc:
            raise OSError(f"txt lookup {host} via {server} failed") from exc
    try:
        return parse_txt_response(payload, query_id)
    except ValueError as exc:
        raise OSError(f"txt lookup {host} via {server} failed: {exc}") from exc


def _lookup_txt_https(host: str, timeout: float) -> str:
    errors: list[str] = []
    for endpoint in _DOH:
        try:
            return _doh_txt(endpoint, host, timeout)
        except OSError as exc:
            errors.append(str(exc))
    detail = "; ".join(errors) if errors else host
    raise OSError(f"txt lookup {host} via HTTPS failed: {detail}")


def _doh_txt(endpoint: str, host: str, timeout: float) -> str:
    query = urllib.parse.urlencode({"name": host, "type": "TXT"})
    request = urllib.request.Request(
        f"{endpoint}?{query}",
        headers={"accept": "application/dns-json"},
    )
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            payload = json.loads(response.read().decode("utf-8"))
    except (OSError, json.JSONDecodeError, ValueError) as exc:
        raise OSError(f"txt lookup {host} via {endpoint} failed") from exc
    return txt_from_doh(payload)


def txt_from_doh(payload: dict) -> str:
    """Return TXT strings from a DNS-over-HTTPS JSON answer."""
    if payload.get("Status") not in (0, None):
        raise OSError(f"dns status {payload.get('Status')}")
    parts: list[str] = []
    for answer in payload.get("Answer") or []:
        if answer.get("type") == _TXT:
            parts.append(str(answer.get("data", "")))
    return "".join(parts)


def parse_txt_response(payload: bytes, query_id: int) -> str:
    """Return TXT character-strings from one DNS response."""
    if len(payload) < 12:
        raise ValueError("truncated dns reply")
    ident, flags, questions, answers, _authority, _extra = struct.unpack_from("!HHHHHH", payload, 0)
    if ident != query_id:
        raise ValueError("dns id mismatch")
    rcode = flags & 0x000F
    if rcode:
        raise ValueError(f"dns rcode {rcode}")
    if flags & 0x0200:
        raise ValueError("dns reply truncated")
    offset = _skip_questions(payload, questions)
    return _txt_answers(payload, offset, answers)


def _label(label: str) -> bool:
    if not label or len(label) > 63:
        return False
    alphabet = set("abcdefghijklmnopqrstuvwxyz0123456789-")
    if any(char not in alphabet for char in label):
        return False
    return not label.startswith("-") and not label.endswith("-")


def _nameserver() -> str:
    try:
        with open("/etc/resolv.conf", encoding="utf-8") as handle:
            text = handle.read()
    except OSError:
        return "1.1.1.1"
    for line in text.splitlines():
        parts = line.split()
        if len(parts) >= 2 and parts[0] == "nameserver" and is_ipv4(parts[1]):
            return parts[1]
    return "1.1.1.1"


def _dns_query(host: str, query_id: int) -> bytes:
    header = struct.pack("!HHHHHH", query_id, 0x0100, 1, 0, 0, 0)
    return header + _encode_name(host) + struct.pack("!HH", _TXT, 1)


def _encode_name(host: str) -> bytes:
    labels = host.strip(".").split(".")
    if not labels or any(not label or len(label) > 63 for label in labels):
        raise ValueError(f"not a DNS name: {host}")
    body = b"".join(bytes([len(label)]) + label.encode("ascii") for label in labels)
    return body + b"\x00"


def _skip_questions(payload: bytes, count: int) -> int:
    offset = 12
    for _question in range(count):
        offset = _skip_name(payload, offset) + 4
    return offset


def _skip_name(payload: bytes, offset: int) -> int:
    while True:
        if offset >= len(payload):
            raise ValueError("truncated dns name")
        length = payload[offset]
        if length == 0:
            return offset + 1
        if length & 0xC0 == 0xC0:
            if offset + 1 >= len(payload):
                raise ValueError("truncated dns pointer")
            return offset + 2
        offset += 1 + length


def _txt_answers(payload: bytes, offset: int, count: int) -> str:
    texts: list[str] = []
    for _answer in range(count):
        offset = _skip_name(payload, offset)
        if offset + 10 > len(payload):
            raise ValueError("truncated dns answer")
        rtype, _rclass, _ttl, length = struct.unpack_from("!HHIH", payload, offset)
        offset += 10
        if offset + length > len(payload):
            raise ValueError("truncated dns rdata")
        rdata = payload[offset : offset + length]
        offset += length
        if rtype == _TXT:
            texts.append(_txt_rdata(rdata))
    return "".join(texts)


def _txt_rdata(rdata: bytes) -> str:
    parts: list[str] = []
    index = 0
    while index < len(rdata):
        length = rdata[index]
        index += 1
        if index + length > len(rdata):
            raise ValueError("truncated txt string")
        parts.append(rdata[index : index + length].decode("utf-8", errors="replace"))
        index += length
    return "".join(parts)
