#!/bin/bash
# 워크스페이스 손상 점검/복구 (방전·강제종료 후 노드가 이유 없이 죽을 때 실행)
#
#   bash tools/doctor.sh        # 점검만
#   bash tools/doctor.sh --fix  # 손상 발견 시 touch + 해당 패키지 리빌드까지
#
# 배경: 전원이 갑자기 나가면 install/ 쪽 파일이 0바이트로 깨질 수 있는데,
# timestamp 는 소스와 같아서 colcon build 가 복사를 건너뛴다 (2026-07-07 방전 사고).
# -> 소스와 내용을 직접 비교해서 다르면 touch 후 리빌드해야 한다.

WS=/home/topst/ai_moon_ros2
FIX=false
[ "$1" = "--fix" ] && FIX=true

issues=0
declare -A rebuild_pkgs

echo "=== 1. 소스(src/) 자체 손상 점검 (0바이트 파일) ==="
src_broken=$(find "$WS/src" "$WS/tools" -type f \( -name "*.py" -o -name "*.yaml" -o -name "*.xml" \) ! -name "__init__.py" -size 0 2>/dev/null)
if [ -n "$src_broken" ]; then
    echo "!! 소스 파일이 비어 있음 — git 으로 복구 필요 (git checkout -- <파일>):"
    echo "$src_broken"
    issues=$((issues + 1))
else
    echo "OK"
fi

echo ""
echo "=== 2. src vs install 데이터 파일 내용 비교 (config/launch) ==="
for pkg_dir in "$WS"/src/*/; do
    pkg=$(basename "$pkg_dir")
    for sub in config launch; do
        for src_f in "$pkg_dir$sub"/*; do
            [ -f "$src_f" ] || continue
            inst_f="$WS/install/$pkg/share/$pkg/$sub/$(basename "$src_f")"
            [ -f "$inst_f" ] || continue   # 미설치 파일은 패스
            if ! cmp -s "$src_f" "$inst_f"; then
                echo "!! 불일치: $inst_f ($(wc -c < "$inst_f")B, 소스는 $(wc -c < "$src_f")B)"
                rebuild_pkgs[$pkg]=1
                issues=$((issues + 1))
                # touch 만으로는 truncate 와 같은 초에 일어나면 또 건너뛴다
                # -> 깨진 install 파일을 지워서 무조건 재복사시킨다
                $FIX && rm -f "$inst_f"
            fi
        done
    done
done
[ ${#rebuild_pkgs[@]} -eq 0 ] && echo "OK"

echo ""
echo "=== 3. 하드웨어 디바이스 점검 ==="
cam_dev=$(grep -oE "camera_device: *[^ #]+" "$WS/src/car_planner/config/params.yaml" | awk '{print $2}')
i2c_bus=$(grep -oE "i2c_bus: *[0-9]+" "$WS/src/car_planner/config/params.yaml" | awk '{print $2}')
for dev in "$cam_dev" "/dev/i2c-$i2c_bus"; do
    if [ -e "$dev" ]; then
        echo "OK: $dev"
    else
        echo "!! 없음: $dev (케이블/전원 확인, params.yaml 설정값 기준)"
        issues=$((issues + 1))
    fi
done

if [ ${#rebuild_pkgs[@]} -gt 0 ]; then
    echo ""
    if $FIX; then
        echo "=== 4. 손상 패키지 리빌드: ${!rebuild_pkgs[*]} ==="
        cd "$WS" && colcon build --packages-select "${!rebuild_pkgs[@]}"
    else
        echo "-> 복구하려면: bash tools/doctor.sh --fix"
    fi
fi

echo ""
if [ "$issues" -eq 0 ]; then
    echo "모두 정상."
else
    echo "$issues 건 발견."
fi
exit 0
