"""Load baked hole_poses.yaml for ROS needle_insert_node."""
import os


def _parse_float_list(text):
    vals = text.strip().strip('[]')
    if not vals:
        return []
    return [float(x) for x in vals.split(',')]


def load_hole_poses(path):
    """
    Returns dict:
      home: [6]
      holes: {id: {'xyz', 'approach', 'insert', 'z_path'?}}
    """
    out = {
        'home': [0.0, -1.57, 1.57, -1.57, -1.57, 0.0],
        'holes': {},
    }
    if not path or not os.path.isfile(path):
        return out
    cur = None
    in_z_path = False
    with open(path, encoding='utf-8') as f:
        for line in f:
            raw = line.rstrip('\n')
            s = raw.strip()
            if not s or s.startswith('#'):
                continue
            if s.startswith('home:'):
                in_z_path = False
                parsed = _parse_float_list(s.split(':', 1)[1])
                if len(parsed) == 6:
                    out['home'] = parsed
            elif s.startswith('holes:') and s.endswith('[]'):
                continue
            elif s.startswith('- id:'):
                if cur and 'approach' in cur and 'insert' in cur:
                    out['holes'][cur['id']] = cur
                cur = {'id': int(s.split(':', 1)[1].strip()), 'z_path': []}
                in_z_path = False
            elif cur is not None and s.startswith('xyz:'):
                in_z_path = False
                cur['xyz'] = _parse_float_list(s.split(':', 1)[1])
            elif cur is not None and s.startswith('approach:'):
                in_z_path = False
                cur['approach'] = _parse_float_list(s.split(':', 1)[1])
            elif cur is not None and s.startswith('insert:'):
                in_z_path = False
                cur['insert'] = _parse_float_list(s.split(':', 1)[1])
            elif cur is not None and s.startswith('z_path:'):
                in_z_path = True
                cur['z_path'] = []
            elif cur is not None and in_z_path and s.startswith('- ['):
                cur['z_path'].append(_parse_float_list(s[1:].strip()))
            elif cur is not None and in_z_path and s.startswith('-'):
                # "- 1.0, 2.0, ..." without brackets
                cur['z_path'].append(_parse_float_list(s[1:].strip()))
        if cur and 'approach' in cur and 'insert' in cur:
            out['holes'][cur['id']] = cur
    return out


def default_path():
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('ur3_needle_sim'), 'config', 'hole_poses.yaml')
