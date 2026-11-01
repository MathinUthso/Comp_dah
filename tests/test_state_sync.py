from __future__ import annotations

from backend.state_sync.service import StateSyncService


def test_update_merges_state_and_notifies_listener() -> None:
    service = StateSyncService()
    snapshots: list[dict] = []
    service.set_notifier(lambda state: snapshots.append(state))

    result = service.update({"progress": 42})

    assert result["progress"] == 42
    assert snapshots[-1]["progress"] == 42
    assert service.snapshot()["autonomous"] is False
