from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any

from .action_sources.tcp_delta import CARTESIAN_ACTION_REQUIREMENTS, clamp_tcp_twist, tcp_twist_local_intent
from .recording import _hash_canonical_json
from .robot_state_client import StateSnapshot
from .servo_command_client import CommandIntent


STATE_DIM = 40
ACTION_DIM = 12


class BehaviorCloningActionSource:
    requirements = CARTESIAN_ACTION_REQUIREMENTS

    def __init__(
        self,
        checkpoint_path: str | Path,
        timeout_sec: float = 0.2,
        ignore_config_drift: bool = False,
    ):
        import torch

        self.torch = torch
        self.timeout_sec = timeout_sec
        self.ignore_config_drift = bool(ignore_config_drift)
        self._config_drift_checked = False
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        checkpoint = torch.load(checkpoint_path, map_location=self.device, weights_only=False)
        self.model = _PolicyNet(int(checkpoint["input_dim"]), int(checkpoint["output_dim"]))
        self.model.load_state_dict(checkpoint["model_state"])
        self.model.to(self.device)
        self.model.eval()
        self.obs_mean = torch.tensor(checkpoint["obs_mean"], dtype=torch.float32, device=self.device)
        self.obs_std = torch.tensor(checkpoint["obs_std"], dtype=torch.float32, device=self.device)
        self.training_cartesian_control_hash = str(checkpoint.get("cartesian_control_hash", "") or "")
        self.training_kinematics_hash = str(checkpoint.get("kinematics_hash", "") or "")

    def next_intent(self, snapshot: StateSnapshot, now_monotonic: float) -> CommandIntent | None:
        _ = now_monotonic
        self._warn_once_on_config_drift(snapshot.payload)
        obs = self.torch.tensor(state_vector(snapshot.payload), dtype=self.torch.float32, device=self.device)
        obs = (obs - self.obs_mean) / self.obs_std.clamp_min(1e-6)
        with self.torch.no_grad():
            action = self.model(obs.unsqueeze(0)).squeeze(0).detach().cpu().tolist()
        left = clamp_tcp_twist(action[:6], 0.03, 0.2)
        right = clamp_tcp_twist(action[6:12], 0.03, 0.2)
        if all(value == 0.0 for value in left) and all(value == 0.0 for value in right):
            return None
        return tcp_twist_local_intent(left=left, right=right, timeout_sec=self.timeout_sec)

    def _warn_once_on_config_drift(self, payload: dict[str, Any]) -> None:
        if self.ignore_config_drift or self._config_drift_checked:
            return
        self._config_drift_checked = True
        checks = (
            (
                "cartesian_control",
                self.training_cartesian_control_hash,
                _hash_canonical_json(_dict_or_empty(payload.get("cartesian_control_snapshot"))),
            ),
            (
                "kinematics",
                self.training_kinematics_hash,
                _hash_canonical_json(_dict_or_empty(payload.get("kinematics_snapshot"))),
            ),
        )
        mismatches = [
            f"{name}: checkpoint={expected or '<empty>'} runtime={actual or '<empty>'}"
            for name, expected, actual in checks
            if expected and expected != actual
        ]
        if mismatches:
            print(
                "WARNING: policy_runner config drift detected; "
                + "; ".join(mismatches)
                + ". Use --ignore-config-drift only for deliberate replay/debug.",
                file=sys.stderr,
            )


def train_behavior_cloning(
    *,
    episodes_dir: str | Path,
    checkpoint_path: str | Path,
    epochs: int = 50,
    batch_size: int = 64,
    lr: float = 1e-3,
    strict_config_check: bool = False,
) -> None:
    import torch
    from torch import nn
    from torch.utils.data import DataLoader, TensorDataset

    obs, act = load_dataset(episodes_dir)
    if not obs:
        raise ValueError(f"no TcpTwistLocal action samples found under {episodes_dir}")
    cartesian_control_hash, kinematics_hash = _read_training_config_hashes(episodes_dir)
    mismatches = _config_hash_mismatches(
        episodes_dir,
        cartesian_control_hash=cartesian_control_hash,
        kinematics_hash=kinematics_hash,
    )
    if mismatches:
        message = (
            "config hash mismatch across HDF5 episodes: "
            + "; ".join(mismatches)
        )
        if strict_config_check:
            raise RuntimeError(message)
        print(f"WARNING: {message}", file=sys.stderr)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    x = torch.tensor(obs, dtype=torch.float32)
    y = torch.tensor(act, dtype=torch.float32)
    obs_mean = x.mean(dim=0)
    obs_std = x.std(dim=0).clamp_min(1e-6)
    x = (x - obs_mean) / obs_std

    dataset = TensorDataset(x, y)
    loader = DataLoader(dataset, batch_size=max(1, int(batch_size)), shuffle=True)
    model = _PolicyNet(x.shape[1], y.shape[1]).to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=float(lr))
    loss_fn = nn.MSELoss()

    model.train()
    for epoch in range(max(1, int(epochs))):
        total = 0.0
        count = 0
        for batch_x, batch_y in loader:
            batch_x = batch_x.to(device)
            batch_y = batch_y.to(device)
            pred = model(batch_x)
            loss = loss_fn(pred, batch_y)
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            total += float(loss.item()) * batch_x.shape[0]
            count += batch_x.shape[0]
        print(f"epoch={epoch + 1} loss={total / max(count, 1):.8f}", flush=True)

    checkpoint = {
        "schema": "robotics_lab.policy_runner.bc_checkpoint.v2",
        "input_dim": x.shape[1],
        "output_dim": y.shape[1],
        "obs_mean": obs_mean.tolist(),
        "obs_std": obs_std.tolist(),
        "model_state": model.cpu().state_dict(),
        "cartesian_control_hash": cartesian_control_hash,
        "kinematics_hash": kinematics_hash,
    }
    path = Path(checkpoint_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    torch.save(checkpoint, path)
    print(f"saved checkpoint: {path}", flush=True)


def load_dataset(episodes_dir: str | Path) -> tuple[list[list[float]], list[list[float]]]:
    root = Path(episodes_dir)
    observations: list[list[float]] = []
    actions: list[list[float]] = []
    for episode in sorted(path for path in root.glob("episode_*") if path.is_dir()):
        states = _load_states_by_tick(episode / "robot_state.jsonl")
        actions_path = episode / "actions.jsonl"
        if not actions_path.exists():
            continue
        for raw in actions_path.read_text(encoding="utf-8").splitlines():
            if not raw.strip():
                continue
            row = json.loads(raw)
            action = action_vector(row.get("packet", {}))
            if action is None:
                continue
            state = states.get(row.get("nearest_state_tick"))
            if state is None:
                continue
            observations.append(state_vector(state))
            actions.append(action)
    return observations, actions


def state_vector(payload: dict[str, Any]) -> list[float]:
    values: list[float] = []
    for side in ("left", "right"):
        arm = payload.get(side, {})
        if not isinstance(arm, dict):
            arm = {}
        values.extend(_float_list(arm.get("q_actual_deg"), 6))
        values.extend(_float_list(arm.get("q_sent_deg"), 6))
        values.extend(_pose_values(arm.get("tcp_stand")))
    values.extend([
        float(payload.get("period_ms", 0.0) or 0.0),
        float(payload.get("jitter_ms", 0.0) or 0.0),
        1.0 if payload.get("fault_latched") else 0.0,
        1.0 if payload.get("motion_state") == "Running" else 0.0,
    ])
    return (values + [0.0] * STATE_DIM)[:STATE_DIM]


def action_vector(packet: dict[str, Any]) -> list[float] | None:
    values: list[float] = []
    saw_action = False
    for side in ("left", "right"):
        arm = packet.get(side, {})
        if isinstance(arm, dict) and arm.get("mode") == "TcpTwistLocal":
            twist = _float_list(arm.get("tcp_twist_local"), 6)
            saw_action = True
        else:
            twist = [0.0] * 6
        values.extend(twist)
    return values if saw_action else None


def _load_states_by_tick(path: Path) -> dict[Any, dict[str, Any]]:
    if not path.exists():
        return {}
    states: dict[Any, dict[str, Any]] = {}
    for raw in path.read_text(encoding="utf-8").splitlines():
        if not raw.strip():
            continue
        row = json.loads(raw)
        payload = row.get("payload", {})
        if isinstance(payload, dict):
            states[payload.get("tick")] = payload
    return states


def _read_training_config_hashes(episodes_dir: str | Path) -> tuple[str, str]:
    first = _first_hdf5_episode(episodes_dir)
    if first is None:
        return "", ""
    try:
        import h5py
    except ModuleNotFoundError:
        return "", ""
    with h5py.File(first, "r") as handle:
        return (
            str(handle.attrs.get("cartesian_control_hash", "") or ""),
            str(handle.attrs.get("kinematics_hash", "") or ""),
        )


def _config_hash_mismatches(
    episodes_dir: str | Path,
    *,
    cartesian_control_hash: str,
    kinematics_hash: str,
) -> list[str]:
    root = Path(episodes_dir)
    try:
        import h5py
    except ModuleNotFoundError:
        return []
    mismatches: list[str] = []
    expected = {
        "cartesian_control_hash": cartesian_control_hash,
        "kinematics_hash": kinematics_hash,
    }
    for path in sorted(root.rglob("*.hdf5")):
        with h5py.File(path, "r") as handle:
            for key, expected_hash in expected.items():
                actual = str(handle.attrs.get(key, "") or "")
                if actual != expected_hash:
                    mismatches.append(f"{path.name}:{key}={actual}")
    return mismatches


def _first_hdf5_episode(episodes_dir: str | Path) -> Path | None:
    root = Path(episodes_dir)
    for path in sorted(root.rglob("*.hdf5")):
        return path
    return None


def _dict_or_empty(value: Any) -> dict[str, Any]:
    return dict(value) if isinstance(value, dict) else {}


def _float_list(value: Any, length: int) -> list[float]:
    if not isinstance(value, list):
        return [0.0] * length
    out = []
    for item in value[:length]:
        try:
            out.append(float(item))
        except (TypeError, ValueError):
            out.append(0.0)
    return out + [0.0] * (length - len(out))


def _pose_values(value: Any) -> list[float]:
    if not isinstance(value, dict):
        return [0.0] * 8
    return [
        float(value.get("x", 0.0) or 0.0),
        float(value.get("y", 0.0) or 0.0),
        float(value.get("z", 0.0) or 0.0),
        float(value.get("rx", 0.0) or 0.0),
        float(value.get("ry", 0.0) or 0.0),
        float(value.get("rz", 0.0) or 0.0),
        float(value.get("qw", 0.0) or 0.0),
        1.0,
    ]


class _PolicyNet:
    def __new__(cls, input_dim: int, output_dim: int):
        import torch
        from torch import nn

        _ = cls
        return nn.Sequential(
            nn.Linear(input_dim, 128),
            nn.ReLU(),
            nn.Linear(128, 128),
            nn.ReLU(),
            nn.Linear(128, output_dim),
        )
