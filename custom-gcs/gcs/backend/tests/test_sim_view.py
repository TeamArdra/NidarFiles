"""The live Gazebo 3D panel (Main Mission (Sim)) is advertised only when the
GCS talks to the simulator, never with the real MicroLR900 radio."""
from fastapi.testclient import TestClient

from app.config import Settings
from app.main import create_app

from .fakes import FakeRadioLink, FakeRosBridgeClient


def test_no_sim_view_with_the_real_radio():
    assert Settings(radio_port="COM5", sim_view_url=None).sim_view() is None
    assert Settings(radio_port="/dev/ttyUSB0", sim_view_url=None).sim_view() is None


def test_sim_view_when_the_radio_is_the_simulators_tcp_link():
    assert Settings(radio_port="socket://127.0.0.1:5800", sim_view_url=None).sim_view() == "ws://127.0.0.1:9010"


def test_explicit_sim_view_url_wins():
    assert Settings(radio_port="COM5", sim_view_url="ws://10.0.0.5:9010").sim_view() == "ws://10.0.0.5:9010"


def test_health_reports_sim_view_url():
    app = create_app(
        client=FakeRosBridgeClient(),
        settings=Settings(radio_port="socket://127.0.0.1:5800", sim_view_url=None, radio_enabled=False),
        radio=FakeRadioLink(),
    )
    with TestClient(app) as client:
        assert client.get("/health").json()["sim_view_url"] == "ws://127.0.0.1:9010"
