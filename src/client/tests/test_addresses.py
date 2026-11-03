import time
from pathlib import Path

from leo_client.addresses import AddressBook, Dial, ServerAddress, plan_targets
from leo_client.session import open_link
from leo_protocol.protocol import Hello
from leo_server.common.config import load_config
from leo_server.nodes.robot_bridge.link import ServerLink, reply_bytes

ROOT = Path(__file__).resolve().parents[3]
CFG = load_config(ROOT / "config" / "robot.yaml")


def test_template_starts_empty():
    book = AddressBook.load(ROOT / "data" / "server_ips.yaml")
    assert book.domain == ""
    assert book.servers == []


def test_remember_moves_the_latest_session_to_the_top(tmp_path):
    path = tmp_path / "server_ips.yaml"
    book = AddressBook(path, "leosha.duckdns.org", [])
    book.remember("192.168.1.1", "203.0.113.1", session="2026-11-01T10:00:00+03:00")
    book.remember("192.168.1.2", "203.0.113.2", session="2026-11-02T10:00:00+03:00")
    book.remember("192.168.1.1", "203.0.113.1", session="2026-11-03T12:30:00+03:00")
    loaded = AddressBook.load(path)
    assert loaded.domain == "leosha.duckdns.org"
    assert [(item.local_ip, item.global_ip, item.session) for item in loaded.servers] == [
        ("192.168.1.1", "203.0.113.1", "2026-11-03T12:30:00+03:00"),
        ("192.168.1.2", "203.0.113.2", "2026-11-02T10:00:00+03:00"),
    ]
    text = path.read_text(encoding="utf-8")
    assert text.index("192.168.1.1") < text.index("192.168.1.2")
    assert "local` is tried before `global`" in text


def test_hand_edited_book_keeps_its_order_and_comments(tmp_path):
    path = tmp_path / "server_ips.yaml"
    path.write_text(
        "\n".join(
            [
                "# keep me",
                'domain: "leosha"',
                "servers:",
                '  - local: "192.168.1.8"',
                '    global: "198.51.100.8"',
                '    session: "2026-11-03T09:00:00+03:00"  # latest',
                "  - local: 192.168.1.2",
                "    global: 198.51.100.2",
                "    session: 2026-11-01T09:00:00+03:00",
                "",
            ]
        ),
        encoding="utf-8",
    )
    book = AddressBook.load(path)
    assert book.domain == "leosha"
    assert [(item.local_ip, item.global_ip) for item in book.servers] == [
        ("192.168.1.8", "198.51.100.8"),
        ("192.168.1.2", "198.51.100.2"),
    ]
    assert book.servers[1].session == "2026-11-01T09:00:00+03:00"


def test_legacy_ip_field_is_the_global_address(tmp_path):
    path = tmp_path / "server_ips.yaml"
    path.write_text(
        "\n".join(
            [
                'domain: "leosha"',
                "servers:",
                '  - ip: "198.51.100.8"',
                '    session: "2026-11-03T09:00:00+03:00"',
                "",
            ]
        ),
        encoding="utf-8",
    )
    book = AddressBook.load(path)
    assert book.servers == [ServerAddress("", "198.51.100.8", "2026-11-03T09:00:00+03:00")]
    assert [item.ip for item in book.dial_order(None)] == ["198.51.100.8"]


def test_plan_tries_local_before_global_for_dns_and_every_saved_row(tmp_path):
    book = AddressBook(
        tmp_path / "server_ips.yaml",
        "leosha",
        [
            ServerAddress("192.168.1.9", "203.0.113.9", "2026-11-02T00:00:00+03:00"),
            ServerAddress("192.168.1.4", "203.0.113.4", "2026-11-01T00:00:00+03:00"),
        ],
    )

    def resolve(host):
        assert host == "leosha.duckdns.org"
        return ServerAddress("192.168.1.50", "203.0.113.4", "")

    attempts, host = plan_targets("", book, resolve)
    assert host == "leosha.duckdns.org"
    assert [item.ip for item in attempts] == [
        "192.168.1.50",
        "203.0.113.4",
        "192.168.1.9",
        "203.0.113.9",
        "192.168.1.4",
    ]
    assert attempts[1] == Dial("203.0.113.4", "192.168.1.50", "203.0.113.4")

    def offline(host):
        raise OSError(f"dns down for {host}")

    attempts, host = plan_targets("leosha.duckdns.org", book, offline)
    assert [item.ip for item in attempts] == ["192.168.1.9", "203.0.113.9", "192.168.1.4", "203.0.113.4"]

    def explode(host):
        raise AssertionError(host)

    assert plan_targets("127.0.0.1", book, explode) == ([Dial("127.0.0.1", "", "")], None)


def test_plan_requires_a_name_or_a_saved_address(tmp_path):
    empty = AddressBook(tmp_path / "empty.yaml", "", [])
    try:
        plan_targets("", empty)
    except RuntimeError as exc:
        assert "domain" in str(exc)
    else:
        raise AssertionError("expected a missing domain")

    def offline(host):
        raise OSError("dns down")

    try:
        plan_targets("leosha", empty, offline)
    except RuntimeError as exc:
        assert "no server IP" in str(exc)
    else:
        raise AssertionError("expected an empty book")


def test_handshake_saves_the_resolved_ip_and_falls_back(tmp_path):
    network = CFG.network
    server = ServerLink("127.0.0.1", network["telemetry_port"], network["handshake_port"], network["command_port"], "127.0.0.1")

    def handler(hello, server_time_ns):
        assert isinstance(hello, Hello)
        return reply_bytes(server_time_ns, hello, CFG.client_payload(), None, network)

    server.start(handler)
    time.sleep(0.15)
    path = tmp_path / "server_ips.yaml"
    try:
        direct = tmp_path / "unused.yaml"
        link, reply = open_link("127.0.0.1", "", str(direct), timeout_ms=1000)
        assert reply["client"]["command_hz"] == CFG.client["command_hz"]
        link.close()
        assert not direct.exists()

        book = AddressBook(path, "leosha.duckdns.org", [ServerAddress("", "203.0.113.7", "2026-11-01T00:00:00+03:00")])
        book.save()
        live = ServerAddress("192.0.2.9", "127.0.0.1", "")
        link, _reply = open_link("", "", str(path), timeout_ms=1000, resolve=lambda _host: live)
        link.close()
        saved = AddressBook.load(path)
        assert [(item.local_ip, item.global_ip) for item in saved.servers] == [
            ("192.0.2.9", "127.0.0.1"),
            ("", "203.0.113.7"),
        ]
        assert saved.servers[0].session.startswith("20")
        assert saved.domain == "leosha.duckdns.org"

        missed = ServerAddress("", "192.0.2.1", "")
        link, _reply = open_link("", "", str(path), timeout_ms=800, resolve=lambda _host: missed)
        link.close()
        saved = AddressBook.load(path)
        assert (saved.servers[0].local_ip, saved.servers[0].global_ip) == ("192.0.2.9", "127.0.0.1")
        assert "192.0.2.1" not in [item.local_ip for item in saved.servers]
        assert "192.0.2.1" not in [item.global_ip for item in saved.servers]
    finally:
        server.close()
