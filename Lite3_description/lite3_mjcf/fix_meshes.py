# 处理 mjcf 中的 mesh 文件，从 ASCII 转为 binary STL，并且对超过 20w faces 的进行简化。

import trimesh as tm, re, os, pathlib, xml.etree.ElementTree as ET

MJCF = "./mjcf/Lite3.xml"      # 你的 mjcf 路径
MESH_DIR = "./meshes"          # mesh 文件夹
MAX_FACES = 200_000
TARGET_FACES = 100_000

tree = ET.parse(MJCF)
root = tree.getroot()

meshes = {g.attrib['file'] for g in root.iter('mesh')}
for rel_path in meshes:
    fpath = os.path.normpath(os.path.join(os.path.dirname(MJCF), rel_path))
    if not os.path.isfile(fpath):
        print("[MISS]", fpath); continue
    try:
        m = tm.load(fpath, force='mesh')
    except Exception as e:
        print("[ERR ]", fpath, e); continue

    if len(m.faces) == 0:
        print("[BAD ] 0 faces:", fpath); continue

    # ASCII 检测
    with open(fpath, 'rb') as fh:
        ascii_file = fh.read(5).lower() == b'solid'

    changed = False
    if ascii_file:
        changed = True
    if len(m.faces) > MAX_FACES:
        original = len(m.faces)

        # 新 fast-simplification 支持 target_count; 旧版只认 target_reduction
        if hasattr(m, "simplify_quadric_decimation"):
            try:
                m = m.simplify_quadric_decimation(TARGET_FACES)       # 尝试新版
            except ValueError:
                ratio = 1.0 - TARGET_FACES / original                 # 计算比例
                m = m.simplify_quadric_decimation(ratio)              # 旧版走这里
        else:
            # 纯 python 慢版
            m = m.simplify_quadratic_decimation(TARGET_FACES, preserve_volume=False)

        changed = True
        
    if changed:
        m.export(fpath, file_type='stl')   # binary
        print(f"[FIX ] {os.path.basename(fpath)} -> faces {len(m.faces)}")
    else:
        print(f"[OK  ] {os.path.basename(fpath)}")

print("Done.")
