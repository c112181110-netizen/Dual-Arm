"""Strict configuration and journal checks, independent of ROS."""
import math


def validate_arm2(c):
    for key, count in [('b_pick', 7), ('c_poses', 56), ('home_joints', 4)]:
        if len(c[key]) != count or not all(math.isfinite(x) for x in c[key]):
            raise ValueError(f'{key} requires {count} measured finite values')
    poses = [c['b_pick']] + [c['c_poses'][i:i + 7] for i in range(0, 56, 7)]
    for p in poses:
        if abs(sum(v * v for v in p[3:]) - 1) > 1e-4:
            raise ValueError('Pose quaternion must have unit norm (x,y,z,qx,qy,qz,qw)')
    if len({tuple(p[:3]) for p in poses[1:]}) != 8:
        raise ValueError('C requires eight distinct measured slot positions')
    if c['rows'] != 2 or c['columns'] != 4:
        raise ValueError('C layout must be 2 rows by 4 columns')
    if c.get('grid_verification', True) and c.get('base_frame', 'arm2_base') != 'arm2_base':
        raise ValueError('Existing grid service coordinates are explicitly arm2_base')
    tolerance = c.get('grid_position_tolerance', 0.01)
    if not math.isfinite(tolerance) or tolerance <= 0:
        raise ValueError('grid_position_tolerance must be positive and finite')
    if c.get('auto_confirm_demo', False) and not c.get('grid_verification', True):
        raise ValueError('Manual-only grid mode cannot automatically confirm layout/placement')
    if not math.isfinite(c['safety_max_age']) or c['safety_max_age'] <= 0:
        raise ValueError('safety_max_age must be positive')
    if any(v <= 0 for v in c.get('table_size', [1, 1, 1])):
        raise ValueError('table_size dimensions must be positive')
    if not c['simulation'] and (not c['calibrated'] or not c['standalone_arm1_secured'] or not c['calibration_id']):
        raise ValueError('Hardware requires calibration_id, calibrated and standalone_arm1_secured')
    boxes = c['obstacle_boxes']
    if len(boxes) % 6 or not all(math.isfinite(x) for x in boxes):
        raise ValueError('obstacle_boxes: flattened [cx,cy,cz,sx,sy,sz]')
    if any(min(boxes[i + 3:i + 6]) <= 0 for i in range(0, len(boxes), 6)):
        raise ValueError('Obstacle sizes must be positive')
    return poses[1:]


def validate_journal(old):
    occupied = old.get('occupied')
    if not isinstance(occupied, list) or len(occupied) != 8 or any(type(x) is not bool for x in occupied):
        raise ValueError('Invalid occupancy journal; inspect physical layout before repairing')
    if old.get('holding') not in ('empty', 'held', 'unknown') or type(old.get('b_locked')) is not bool:
        raise ValueError('Invalid holding/lock journal')
    return occupied
