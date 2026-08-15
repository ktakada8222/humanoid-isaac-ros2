import sys
import os

def convert_with_intensity(input_ply, output_pcd):
    tmp_pcd = "tmp_map.pcd"

    # PLY → PCD (ascii) に変換
    # os.system(f"pcl_ply2pcd {input_ply} {tmp_pcd} -ascii")
    os.system(f"pcl_ply2pcd {input_ply} {tmp_pcd} -format 0")


    # 読み込み
    with open(tmp_pcd, "r") as f:
        lines = f.readlines()

    # ヘッダ編集
    header = []
    data_start = 0
    for i, line in enumerate(lines):
        header.append(line)
        if line.startswith("DATA"):
            data_start = i + 1
            break

    # ヘッダ修正
    for j, l in enumerate(header):
        if l.startswith("FIELDS"):
            header[j] = "FIELDS x y z intensity\n"
        if l.startswith("SIZE"):
            header[j] = "SIZE 4 4 4 4\n"
        if l.startswith("TYPE"):
            header[j] = "TYPE F F F F\n"
        if l.startswith("COUNT"):
            header[j] = "COUNT 1 1 1 1\n"

    # データ修正
    data_lines = []
    for l in lines[data_start:]:
        l = l.strip()
        if l:
            data_lines.append(l + " 1.0\n")  # intensity=1.0 を追加

    # 書き出し
    with open(output_pcd, "w") as f:
        f.writelines(header + data_lines)

    os.remove(tmp_pcd)
    print(f"[OK] {output_pcd} を生成しました")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("使い方: docker run --rm -v $(pwd):/data intensity-converter map.ply map_with_intensity.pcd")
        sys.exit(1)

    input_ply = sys.argv[1]
    output_pcd = sys.argv[2]
    convert_with_intensity(input_ply, output_pcd)
