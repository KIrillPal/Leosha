"""DuckDNS name and IPv4 checks shared by the server and the robot."""

from __future__ import annotations

_SUFFIX = ".duckdns.org"


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


def _label(label: str) -> bool:
    if not label or len(label) > 63:
        return False
    alphabet = set("abcdefghijklmnopqrstuvwxyz0123456789-")
    if any(char not in alphabet for char in label):
        return False
    return not label.startswith("-") and not label.endswith("-")
