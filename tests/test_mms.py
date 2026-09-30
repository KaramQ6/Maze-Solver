from unittest.mock import patch

import pytest

from mms_main import MMS_API


def test_mms_move_failure_is_not_accepted_as_progress() -> None:
    with patch.object(MMS_API, "_command", return_value="crash"):
        with pytest.raises(RuntimeError, match="crash"):
            MMS_API.move_forward()
