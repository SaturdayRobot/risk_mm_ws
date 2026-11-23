"""Content identity for a robot URDF and its ordered execution/planning configs."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET


def robot_signature(urdf, config_paths=(), *, model_data=None):
    urdf = Path(urdf)
    model_id = ET.fromstring(urdf.read_bytes()).get('name', '').strip()
    if not model_id:
        raise ValueError('URDF robot name is required')
    digest = hashlib.sha256()
    # Content and order matter; absolute paths do not.
    for path in (urdf, *map(Path, config_paths)):
        data = path.read_bytes()
        digest.update(len(data).to_bytes(8, 'big'))
        digest.update(data)
    if model_data is not None:
        # URDF-derived collision points include actual sampled mesh geometry.
        # Resource paths are presentation/source metadata; exclude their paths.
        geometry={k:v for k,v in model_data.items() if k!='mm.urdf_source' and
                  not k.endswith(('_resource','_resources'))}
        digest.update(json.dumps(geometry,sort_keys=True,separators=(',',':'),allow_nan=False).encode())
    return model_id + ':' + digest.hexdigest()
