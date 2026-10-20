from __future__ import annotations

import json
from pathlib import Path

import yaml

from trajectory.trajectory_utils import normalize_trajectory


def _mat_to_python(value):
    import numpy as np
    if isinstance(value, dict):
        return {k: _mat_to_python(v) for k, v in value.items()}
    if hasattr(value, "_fieldnames"):
        return {k: _mat_to_python(getattr(value, k)) for k in value._fieldnames}
    if isinstance(value, np.void) and value.dtype.names:
        return {k: _mat_to_python(value[k]) for k in value.dtype.names}
    if isinstance(value, np.ndarray) and value.dtype.names:
        if value.size == 1:
            return _mat_to_python(value.reshape(-1)[0])
        return [_mat_to_python(v) for v in value.reshape(-1)]
    if isinstance(value, np.ndarray) and value.dtype == object:
        if value.size == 1:
            return _mat_to_python(value.flat[0])
        return [_mat_to_python(v) for v in value.flat]
    return value


def load_trajectory(path: str):
    file_path = Path(path)
    suffix = file_path.suffix.lower()
    if suffix in (".yaml", ".yml"):
        data = yaml.safe_load(file_path.read_text())
    elif suffix == ".json":
        data = json.loads(file_path.read_text())
    elif suffix == ".mat":
        from scipy.io import loadmat
        raw = {
            k: v
            for k, v in loadmat(file_path, squeeze_me=True, struct_as_record=False).items()
            if not k.startswith("__")
        }
        data = _mat_to_python(raw)
    else:
        raise ValueError("仅支持 .yaml/.yml/.json/.mat")
    return normalize_trajectory(data, source_path=str(file_path))
