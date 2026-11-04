import struct

from leo_protocol.duckdns import (
    hostname,
    is_private_ipv4,
    local_ipv4_from_txt,
    parse_txt_response,
    subdomain,
    txt_from_doh,
)
from leo_server.nodes.robot_bridge.duckdns import DuckDns, parse_reply


class _Log:
    def __init__(self) -> None:
        self.infos: list[str] = []
        self.warnings: list[str] = []
        self.errors: list[str] = []

    def info(self, message: str) -> None:
        self.infos.append(message)

    def warning(self, message: str) -> None:
        self.warnings.append(message)

    def error(self, message: str) -> None:
        self.errors.append(message)


class _Body:
    def __init__(self, payload: bytes) -> None:
        self._payload = payload

    def read(self) -> bytes:
        return self._payload

    def __enter__(self):
        return self

    def __exit__(self, *args) -> bool:
        return False


def test_name_accepts_a_label_or_the_full_hostname():
    assert hostname("Leosha") == "leosha.duckdns.org"
    assert hostname("leosha.duckdns.org") == "leosha.duckdns.org"
    assert subdomain("leosha.duckdns.org") == "leosha"
    try:
        hostname("leosha.example.com")
    except ValueError as exc:
        assert "not a DuckDNS name" in str(exc)
    else:
        raise AssertionError("expected a rejected domain")


def test_publish_once_lets_duckdns_detect_the_public_ipv4():
    captured = {}

    def opener(request, timeout):
        captured["url"] = request.full_url
        captured["timeout"] = timeout
        return _Body(b"OK\n203.0.113.10\nNOCHANGE\n")

    service = DuckDns("Leosha.DuckDNS.org", "secret-token", 300, _Log(), opener, local_ip=lambda: "")
    published = service.publish_once()
    assert published.global_ip == "203.0.113.10"
    assert published.local_ip == ""
    url = captured["url"]
    assert "domains=leosha" in url
    assert "token=secret-token" in url
    assert "verbose=true" in url
    query = url.split("?", 1)[1]
    assert "ip=" not in query
    assert "txt=" not in query


def test_publish_sends_a_global_override_and_the_lan_address():
    captured = {}

    def opener(request, timeout):
        captured["url"] = request.full_url
        captured["timeout"] = timeout
        return _Body(b"OK\n100.64.0.8\nUPDATED\n")

    service = DuckDns(
        "ipleo.duckdns.org",
        "secret-token",
        300,
        _Log(),
        opener,
        global_ip="100.64.0.8",
        local_ip=lambda: "192.168.1.40",
    )
    published = service.publish_once()
    assert published.global_ip == "100.64.0.8"
    assert published.local_ip == "192.168.1.40"
    query = captured["url"].split("?", 1)[1]
    assert "ip=100.64.0.8" in query
    assert "txt=192.168.1.40" in query


def test_publish_sends_any_ipv4_from_the_ip_argument():
    captured = {}

    def opener(request, timeout):
        captured["url"] = request.full_url
        captured["timeout"] = timeout
        return _Body(b"OK\n203.0.113.10\nUPDATED\n")

    service = DuckDns(
        "ipleo.duckdns.org",
        "secret-token",
        300,
        _Log(),
        opener,
        local_ip=lambda: "100.98.148.137",
    )
    published = service.publish_once()
    assert published.global_ip == "203.0.113.10"
    assert published.local_ip == "100.98.148.137"
    assert "txt=100.98.148.137" in captured["url"]


def test_private_ranges_and_txt_parsing():
    assert is_private_ipv4("10.1.2.3")
    assert is_private_ipv4("172.16.0.1")
    assert is_private_ipv4("172.31.255.255")
    assert is_private_ipv4("192.168.0.20")
    assert not is_private_ipv4("172.15.0.1")
    assert not is_private_ipv4("100.64.0.8")
    assert not is_private_ipv4("31.59.217.158")
    assert local_ipv4_from_txt("192.168.0.20") == "192.168.0.20"
    assert local_ipv4_from_txt('"192.168.31.71"') == "192.168.31.71"
    assert local_ipv4_from_txt("nope") == ""
    answer = {"Status": 0, "Answer": [{"type": 16, "data": '"192.168.31.71"'}]}
    assert local_ipv4_from_txt(txt_from_doh(answer)) == "192.168.31.71"
    assert txt_from_doh({"Status": 0, "Answer": []}) == ""
    packet = struct.pack("!HHHHHH", 7, 0x8180, 1, 1, 0, 0)
    for label in (b"ipleo", b"duckdns", b"org"):
        packet += bytes([len(label)]) + label
    packet += b"\x00" + struct.pack("!HH", 16, 1)
    packet += b"\xc0\x0c"
    text = b"192.168.0.20"
    rdata = bytes([len(text)]) + text
    packet += struct.pack("!HHIH", 16, 1, 60, len(rdata)) + rdata
    assert parse_txt_response(packet, 7) == "192.168.0.20"
    try:
        parse_txt_response(struct.pack("!HHHHHH", 7, 0x8183, 0, 0, 0, 0), 7)
    except ValueError as exc:
        assert "rcode" in str(exc)
    else:
        raise AssertionError("expected a dns error")


def test_rejected_update_and_a_redacted_error():
    try:
        parse_reply("KO\n")
    except RuntimeError as exc:
        assert "KO" in str(exc)
    else:
        raise AssertionError("expected a rejected update")
    service = DuckDns("leosha", "secret-token", 300, _Log(), lambda *_args, **_kwargs: None)
    assert "secret-token" not in service._safe(RuntimeError("secret-token in the url"))
    assert "***" in service._safe(RuntimeError("secret-token in the url"))


def test_publish_and_a_rejected_ip_are_printed(monkeypatch):
    written = []
    monkeypatch.setattr("leo_server.nodes.robot_bridge.duckdns.console.write", written.append)
    holder = {}

    def opener(request, timeout):
        holder["service"]._stop.set()
        return _Body(b"OK\n203.0.113.10\nUPDATED\n")

    service = DuckDns(
        "ipleo.duckdns.org",
        "secret-token",
        300,
        _Log(),
        opener,
        local_ip=lambda: "100.98.148.137",
    )
    holder["service"] = service
    service._loop()
    assert any("203.0.113.10" in line and "100.98.148.137" in line and "were published to DuckDNS" in line for line in written)

    written.clear()
    monkeypatch.delenv("DUCKDNS_TOKEN", raising=False)
    rejected = {"enabled": True, "domain": "leosha.duckdns.org", "token": "secret", "update_sec": 300}
    assert DuckDns.start_from(rejected, _Log(), "not-an-ip") is None
    assert any("--ip not-an-ip is not an IPv4 address" in line for line in written)


def test_disabled_or_incomplete_settings_do_not_publish(monkeypatch):
    monkeypatch.delenv("DUCKDNS_TOKEN", raising=False)
    log = _Log()
    assert DuckDns.start_from({"enabled": False, "domain": "", "token": "", "update_sec": 300}, log) is None
    assert any("disabled" in line for line in log.infos)
    failed = _Log()
    section = {"enabled": True, "domain": "leosha.duckdns.org", "token": "", "update_sec": 300}
    assert DuckDns.start_from(section, failed) is None
    assert failed.errors
    bad = _Log()
    rejected = {"enabled": True, "domain": "nope.example.com", "token": "secret", "update_sec": 300}
    assert DuckDns.start_from(rejected, bad) is None
    assert bad.errors
    override = _Log()
    bad_ip = {
        "enabled": True,
        "domain": "leosha.duckdns.org",
        "token": "secret",
        "update_sec": 300,
        "global_ip": "netbird",
    }
    assert DuckDns.start_from(bad_ip, override) is None
    assert any("global_ip" in line for line in override.errors)
    local = _Log()
    public = {"enabled": True, "domain": "leosha.duckdns.org", "token": "secret", "update_sec": 300}
    assert DuckDns.start_from(public, local, "not-an-ip") is None
    assert any("local address" in line for line in local.errors)
