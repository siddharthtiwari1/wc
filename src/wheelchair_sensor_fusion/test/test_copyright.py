"""Copyright test for wheelchair_sensor_fusion package."""

from ament_copyright.main import main
import pytest


@pytest.mark.copyright
@pytest.mark.linter
def test_copyright():
    """Test copyright compliance."""
    rc = main(argv=['.', 'test'])
    assert rc == 0, 'Found errors'
