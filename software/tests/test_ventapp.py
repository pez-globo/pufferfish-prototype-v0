"""Tests basic loading of ventapp package."""

try:
    import ventapp
except ImportError:
    ventapp = None


def test_ventapp_loads():
    assert ventapp is not None
