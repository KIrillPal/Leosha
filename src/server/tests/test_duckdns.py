from leo_protocol.duckdns import hostname, subdomain
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

    service = DuckDns("Leosha.DuckDNS.org", "secret-token", 300, _Log(), opener)
    assert service.publish_once() == "203.0.113.10"
    url = captured["url"]
    assert "domains=leosha" in url
    assert "token=secret-token" in url
    assert "verbose=true" in url
    assert "ip=" not in url.split("?", 1)[1]


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
