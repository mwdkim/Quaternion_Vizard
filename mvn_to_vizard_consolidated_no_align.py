# mvn_to_vizard_consolidated_no_align.py
# One-file MVN (UDP MXTP) → Vizard retarget using direct quaternion component remap
# Requested mapping: MVN (w,x,y,z)  →  Vizard (x,y,z,w) = (-z, x, y, w)
# - No world-frame ALIGN rotation, no optional Z-flip
# - Global-space calibration: offset = S1.inverse() * A1 ; Apply = S * offset
# - Positions keep the same Z-up (MVN) → Y-up (Vizard) axis swap used previously

import viz, viztask, vizcam, vizshape, vizact
import socket, struct, threading

# ------------------ Vizard setup ------------------
viz.setMultiSample(4)
viz.go()
viz.clearcolor(0.9, 0.9, 0.9)

# Scene + camera
_ = vizshape.addPlane(size=(12,12), axis=vizshape.AXIS_Y)
vizcam.PivotNavigate(center=[0,1.6,0], distance=3.0)

# Avatar
man = viz.addAvatar('Avatar/vcc_male.cfg')
man.setPosition(0,0,2.0)
man.setEuler(180,0,0)  # keep if you want the avatar to face the camera

# ------------------ Segment → bone map ------------------
# 23-segment MVN model → common Bip01 bones in vcc_male
SEG_TO_BONE = {
    'Pelvis'        : 'Bip01 Pelvis',
    'L5'            : 'Bip01 Spine',
    'L3'            : 'Bip01 Spine1',
    'T12'           : 'Bip01 Spine1',   # adjust to your rig
    'T8'            : 'Bip01 Spine2',
    'Neck'          : 'Bip01 Neck',
    'Head'          : 'Bip01 Head',
    'RightShoulder' : 'Bip01 R Clavicle',
    'RightUpperArm' : 'Bip01 R UpperArm',
    'RightForeArm'  : 'Bip01 R Forearm',
    'RightHand'     : 'Bip01 R Hand',
    'LeftShoulder'  : 'Bip01 L Clavicle',
    'LeftUpperArm'  : 'Bip01 L UpperArm',
    'LeftForeArm'   : 'Bip01 L Forearm',
    'LeftHand'      : 'Bip01 L Hand',
    'RightUpLeg'    : 'Bip01 R Thigh',
    'RightLeg'      : 'Bip01 R Calf',
    'RightFoot'     : 'Bip01 R Foot',
    # 'RightToe'    : 'Bip01 R Toe0',   # uncomment if your rig has it
    'LeftUpLeg'     : 'Bip01 L Thigh',
    'LeftLeg'       : 'Bip01 L Calf',
    'LeftFoot'      : 'Bip01 L Foot',
    # 'LeftToe'     : 'Bip01 L Toe0',
}

# Cache existing bones
bones = {}
for seg, bone_name in SEG_TO_BONE.items():
    try:
        b = man.getBone(bone_name)
        if b: bones[seg] = b
    except: pass

for b in bones.values():
    b.lock()

# ------------------ MVN UDP (MXTP) receiver ------------------
UDP_PORT = 9763  # MVN default

HEADER_FMT = '>6s I B B I B B B B H H'  # 24 bytes, big-endian
ITEM_FMT   = '>i f f f f f f f'         # seg_id, px,py,pz, q1..q4 (w,x,y,z)

SEG_NAMES = [
    'Pelvis','L5','L3','T12','T8','Neck','Head',
    'RightShoulder','RightUpperArm','RightForeArm','RightHand',
    'LeftShoulder','LeftUpperArm','LeftForeArm','LeftHand',
    'RightUpLeg','RightLeg','RightFoot','RightToe',
    'LeftUpLeg','LeftLeg','LeftFoot','LeftToe'
]

def seg_name_from_id(seg_id):
    idx = seg_id - 1
    return SEG_NAMES[idx] if 0 <= idx < len(SEG_NAMES) else None

# Shared state
latest_quat_viz = {}   # seg -> viz.Quat (global) after direct component remap (no ALIGN)
offset_quats    = {}   # seg -> viz.Quat
initialized     = set()
latest_root_pos = None # [x,y,z] in MVN frame (Z-up)

# ---- Quaternion component remap (no world-frame rotation) ----
# Directly map MVN quaternion (w,x,y,z) → Vizard quaternion (x,y,z,w) as:
#   (w,x,y,z) ↦ (-z, x, y, w)
# Use this when your rig and data conventions are already aligned and you want no extra transform.


def fix_axes_pos_mvn_to_vizard(p):
    # MVN pos: (x,y,z) in Z-up RH → Vizard Y-up: (x,z,-y)
    x,y,z = p
    return [-z, x, y]


def parse_mxtp02(data):
    """Parse one MXTP02 datagram. Returns (character_id, root_pos, dict(seg_name -> viz.Quat))."""
    if len(data) < 24:
        return None, None, {}
    try:
        (idstr, sc, dg, num_items, tc, char_id, nbody, nprops, nfingers, _res, payload) = struct.unpack(HEADER_FMT, data[:24])
    except struct.error:
        return None, None, {}
    if not idstr.startswith(b'MXTP02'):
        return char_id, None, {}
    offset = 24
    item_sz = 32
    max_items = (len(data) - offset) // item_sz
    N = num_items if num_items > 0 else max_items
    N = min(N, max_items)

    out_quats = {}
    root = None

    for i in range(N):
        start = offset + i*item_sz
        seg_id, px, py, pz, qw, qx, qy, qz = struct.unpack(ITEM_FMT, data[start:start+item_sz])
        name = seg_name_from_id(seg_id)
        if not name:
            continue
        # Direct component remap: (w,x,y,z) -> (-z, x, y, w)
        S_viz = viz.Quat(-qz, qx, qy, qw)
        out_quats[name] = S_viz
        if seg_id == 1 and root is None:    # Pelvis position as root
            root = [px, py, pz]
    return char_id, root, out_quats


def udp_loop():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(('0.0.0.0', UDP_PORT))
    viz.logNotice('Listening for MVN (MXTP UDP) on {}.'.format(UDP_PORT))
    while True:
        data, _ = s.recvfrom(16384)
        char_id, root, qmap = parse_mxtp02(data)
        if root is not None:
            globals()['latest_root_pos'] = root
        for seg, q in qmap.items():
            if seg in bones:
                latest_quat_viz[seg] = q

threading.Thread(target=udp_loop, daemon=True).start()

# ------------------ Calibration + Update (global-global scheme) ------------------

def calibrate():
    """Compute per-bone offsets: offset = S1.inverse() * A1 (both ABS_GLOBAL)."""
    for seg, bone in bones.items():
        S1 = latest_quat_viz.get(seg)
        if not S1:
            continue
        A1 = bone.getQuat(viz.ABS_GLOBAL)
        offset_quats[seg] = S1.inverse() * A1
        initialized.add(seg)
    viz.logNotice('Calibration complete (press C in neutral pose).')

vizact.onkeydown('c', calibrate)


def update_avatar():
    while True:
        # root translation (Pelvis)
        if latest_root_pos is not None:
            man.setPosition(fix_axes_pos_mvn_to_vizard(latest_root_pos))

        # per-segment orientation (ABS_GLOBAL)
        for seg, bone in bones.items():
            S = latest_quat_viz.get(seg)
            if not S:
                continue
            if seg not in offset_quats:
                # Lazy-init offset on first valid sensor/bone pair if user forgot to press C
                A1 = bone.getQuat(viz.ABS_GLOBAL)
                offset_quats[seg] = S.inverse() * A1
                initialized.add(seg)
            A2 = S * offset_quats[seg]
            bone.setQuat(A2, viz.ABS_GLOBAL)
        yield viztask.waitTime(0.016)

viztask.schedule(update_avatar)

viz.logNotice("Keys: C = calibrate. MVN UDP on {}. Direct quat remap (-z,x,y,w) with global S^-1*A offset & S*offset apply.".format(UDP_PORT))
