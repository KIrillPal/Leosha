from leo_server.launch_screen import QuietLaunchScreen, is_leo_launch


def test_only_a_leo_launch_is_selected():
    assert is_leo_launch("leo_server", "leo.launch.py")
    assert is_leo_launch("/home/kir/Leosha/LEO/src/server/launch/leo.launch.py", "")
    assert is_leo_launch("demo_nodes_cpp", "talker.launch.py") is False
    assert is_leo_launch("", "") is False


def test_prestart_ignores_other_packages(monkeypatch):
    called = []
    monkeypatch.setattr("leo_server.launch_screen.quiet_screen", lambda: called.append(True))
    extension = QuietLaunchScreen()

    class _Args:
        package_name = "demo_nodes_cpp"
        launch_file_name = "talker.launch.py"

    extension.prestart(_Args())
    assert called == []
    extension.prestart(type("Args", (), {"package_name": "leo_server", "launch_file_name": "leo.launch.py"})())
    assert called == [True]
