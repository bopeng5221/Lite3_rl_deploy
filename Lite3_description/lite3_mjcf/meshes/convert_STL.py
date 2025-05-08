import trimesh as tm
import os, sys, pathlib

def convert(file_path: str):
    mesh = tm.load(file_path, file_type='stl')
    out_path = pathlib.Path(file_path)
    # 覆盖原文件；如想保留原文件可改名加 _bin
    mesh.export(out_path, file_type='stl')  # trimesh 默认导出为 binary
    print(f"[OK] {file_path} -> binary STL")

if __name__ == "__main__":
    # 目录参数可自定义；默认当前文件夹
    folder = sys.argv[1] if len(sys.argv) > 1 else "."
    for f in os.listdir(folder):
        if f.lower().endswith(".stl"):
            fp = os.path.join(folder, f)
            # 跳过已是 binary 的（大文件或 header 非 'solid'）
            with open(fp, 'rb') as fh:
                if fh.read(5).lower() == b"solid":
                    convert(fp)
                else:
                    print(f"[SKIP] already binary: {f}")
