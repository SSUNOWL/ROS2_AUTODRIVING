import os
import re
import math
import argparse
from collections import defaultdict

import pandas as pd


# -----------------------
# 1) 파일명 파싱
# -----------------------

FNAME_PATTERN = re.compile(
    r'(?P<map>.+)_SPD(?P<spd>[\d.]+)_TGT(?P<tgt>[\d.]+)_ACC(?P<acc>[\d.]+)_CRV(?P<crv>[\d.]+)_FRENET_(?P<stamp>\d+)_(?P<result>[A-Z]+)_dur_(?P<dur>[\d.]+)s\.csv'
)


def parse_filename(fname: str):
    m = FNAME_PATTERN.match(fname)
    if not m:
        return None
    d = m.groupdict()
    for k in ["spd", "tgt", "acc", "crv", "dur"]:
        d[k] = float(d[k])
    return d


# -----------------------
# 2) CSV에서 수치 구간만 추출
# -----------------------

def _is_float_str(s):
    try:
        float(s)
        return True
    except Exception:
        return False


def extract_numeric(df: pd.DataFrame) -> pd.DataFrame:
    """
    t 컬럼이 숫자로 변환 가능한 row만 골라서
    t, x, y를 float로 변환해서 반환.
    (실제 주행 로그 구간)
    """
    mask = df["t"].apply(
        lambda v: (
            isinstance(v, (int, float))
            or (isinstance(v, str) and _is_float_str(v))
        )
    )
    num = df[mask].copy()
    if num.empty:
        return num

    for col in ["t", "x", "y"]:
        num[col] = num[col].astype(float)
    return num


# -----------------------
# 3) 첫 랩 완주 시점 추정
# -----------------------

def calc_first_lap_time(num_df: pd.DataFrame,
                        min_lap_time: float = 10.0,
                        goal_radius: float = 1.0):
    """
    - num_df: extract_numeric() 결과
    - min_lap_time: 이 시간 이후부터만 '골인지점 재통과'로 인정
    - goal_radius: 시작점 근처로 돌아왔다고 볼 거리 임계값 [m]

    return: 첫 랩 완주로 간주되는 시간 t (없으면 None)
    """
    if num_df.empty:
        return None

    x0, y0 = num_df["x"].iloc[0], num_df["y"].iloc[0]

    for _, row in num_df.iterrows():
        t = row["t"]
        if t < min_lap_time:
            continue
        d = math.hypot(row["x"] - x0, row["y"] - y0)
        if d < goal_radius:
            return t

    return None


# -----------------------
# 4) 개별 파일 분석
# -----------------------

def analyze_run(path: str,
                min_lap_time: float = 20.0,
                goal_radius: float = 1.0) -> dict:
    """
    단일 CSV 파일에 대해:
      - 파일명에서 파라미터/결과 파싱
      - CSV 내용 기반으로 abort 이유 / 유효 랩타임 계산
    """
    fname = os.path.basename(path)
    meta = parse_filename(fname)
    if meta is None:
        raise ValueError(f"파일명 형식이 예상과 다름: {fname}")

    df = pd.read_csv(path)

    # summary row들
    finish_row = df.loc[df["t"] == "Finish Reason", "x"]
    finish_reason = finish_row.iloc[0] if not finish_row.empty else meta["result"]

    duration_row = df.loc[df["t"] == "Duration (s)", "x"]
    duration_summary = float(duration_row.iloc[0]) if not duration_row.empty else None

    # numeric 구간
    num = extract_numeric(df)
    duration_numeric = num["t"].iloc[-1] if not num.empty else 0.0

    info = {
        **meta,  # map, spd, tgt, acc, crv, stamp, result, dur
        "finish_reason": finish_reason,
        "n_rows": len(df),
        "n_numeric": len(num),
        "duration_numeric": duration_numeric,
        "duration_summary": duration_summary,
        "abort_reason": None,
        "effective_time": None,  # 이걸 랩타임/목표시간으로 사용
        "needs_rerun": False,
    }

    result = meta["result"]

    # ----------- 결과별 분기 -----------

    if result == "GOAL":
        # 정식 GOAL이면 summary Duration 우선
        if duration_summary is not None:
            info["effective_time"] = duration_summary
        else:
            info["effective_time"] = duration_numeric

    elif result == "CRASH":
        info["abort_reason"] = "CRASH"
        info["needs_rerun"] = False
    elif result == "STUCK":
        info["abort_reason"] = "STUCK"
        info["needs_rerun"] = False
    elif result == "ABORT":
        if num.empty:
            # 요약 row 16줄만 있는 케이스 = 시작도 못 함
            info["abort_reason"] = "NO_MOTION"
            info["needs_rerun"] = True
        else:
            # 어느 정도 달리긴 했음 → 첫 랩 완주 시점 찾기
            lap_time = calc_first_lap_time(
                num_df=num,
                min_lap_time=min_lap_time,
                goal_radius=goal_radius,
            )
            if lap_time is None:
                # 한 바퀴를 못 채우고 중간에 멈춘 abort
                info["abort_reason"] = "EARLY_ABORT_NO_LAP"
                info["needs_rerun"] = True
            else:
                # 여러 바퀴/한 바퀴 이상 돌고 수동 abort한 케이스
                info["abort_reason"] = "MULTI_LAP_OR_MANUAL_ABORT_AFTER_LAP"
                info["effective_time"] = lap_time
                info["needs_rerun"] = False

    else:
        # 혹시 모르는 다른 상태 값
        info["abort_reason"] = f"UNKNOWN_RESULT_{result}"
        info["needs_rerun"] = True

    return info


# -----------------------
# 5) 두 맵 합산 타임으로 베스트 파라미터 찾기
# -----------------------

def find_best_weights(analyses,
                      map_names=("Spielberg", "hairpin_combo")):
    """
    analyses: analyze_run() 결과 dict 리스트
    map_names: 합산에 포함할 맵 이름 튜플

    return: total_time 기준으로 정렬된 베스트 파라미터 리스트
    """
    groups = defaultdict(list)
    for a in analyses:
        key = (a["spd"], a["tgt"], a["acc"], a["crv"])
        groups[key].append(a)

    results = []

    for key, runs in groups.items():
        spd, tgt, acc, crv = key
        times_by_map = {}
        # 각 맵별 최소 유효 타임 찾기
        for m in map_names:
            best_t = None
            for a in runs:
                if a["map"] != m:
                    continue
                t = a.get("effective_time")
                if t is None:
                    continue
                if best_t is None or t < best_t:
                    best_t = t
            times_by_map[m] = best_t

        # 모든 맵에서 유효 타임이 있어야 합산 가능
        if all(times_by_map[m] is not None for m in map_names):
            total_time = sum(times_by_map[m] for m in map_names)
            entry = {
                "spd": spd,
                "tgt": tgt,
                "acc": acc,
                "crv": crv,
                "total_time": total_time,
            }
            for m in map_names:
                entry[f"time_{m}"] = times_by_map[m]
            results.append(entry)

    # total_time 기준 오름차순 정렬
    results.sort(key=lambda r: r["total_time"])
    return results


def find_rerun_candidates(analyses,
                          map_names=("Spielberg", "hairpin_combo")):
    """
    어떤 (spd, tgt, acc, crv)에 대해
    특정 맵에서 '유효 타임이 없는' 경우를 재실험 후보로 뽑는다.
    """
    groups = defaultdict(list)
    for a in analyses:
        key = (a["spd"], a["tgt"], a["acc"], a["crv"])
        groups[key].append(a)

    reruns = []

    for key, runs in groups.items():
        spd, tgt, acc, crv = key
        for m in map_names:
            # 이 파라미터 + 맵 조합에서 유효 타임이 있는지 확인
            has_valid_time = any(
                (r["map"] == m and r.get("effective_time") is not None)
                for r in runs
            )
            if not has_valid_time:
                reruns.append({
                    "spd": spd,
                    "tgt": tgt,
                    "acc": acc,
                    "crv": crv,
                    "map": m,
                })

    return reruns


# -----------------------
# 6) main
# -----------------------

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--log_dir",
        type=str,
        required=True,
        help="Frenet 실험 CSV들이 들어있는 디렉터리",
    )
    parser.add_argument(
        "--min_lap_time",
        type=float,
        default=20.0,
        help="첫 랩 완주로 인정할 최소 시간 (s)",
    )
    parser.add_argument(
        "--goal_radius",
        type=float,
        default=1.0,
        help="시작점으로 돌아왔다고 볼 거리 임계값 (m)",
    )
    parser.add_argument(
        "--maps",
        type=str,
        default="Spielberg,hairpin_combo",
        help="합산할 맵 이름들 (콤마로 구분)",
    )

    args = parser.parse_args()
    map_names = tuple(args.maps.split(","))

    # 1) 디렉터리 내 CSV 전부 스캔
    analyses = []
    for fname in os.listdir(args.log_dir):
        if not fname.endswith(".csv"):
            continue
        if "FRENET" not in fname:
            continue

        path = os.path.join(args.log_dir, fname)
        try:
            info = analyze_run(
                path,
                min_lap_time=args.min_lap_time,
                goal_radius=args.goal_radius,
            )
            analyses.append(info)
        except Exception as e:
            print(f"[WARN] {fname} 분석 실패: {e}")

    if not analyses:
        print("분석할 CSV가 없습니다.")
        return

    # 2) 각 파일 분석 결과 요약 출력
    print("==== 개별 실험 분석 결과 (요약) ====")
    for a in analyses:
        print(
            f"{a['map']:12s} | SPD={a['spd']:.2f}, TGT={a['tgt']:.2f}, "
            f"ACC={a['acc']:.2f}, CRV={a['crv']:.2f} | "
            f"res={a['result']:5s} | "
            f"eff_time={a['effective_time']} | "
            f"abort_reason={a['abort_reason']} | "
            f"needs_rerun={a['needs_rerun']}"
        )

    # 3) 베스트 파라미터 찾기
    best = find_best_weights(analyses, map_names=map_names)
    print("\n==== 두 맵 합산 타임 기준 베스트 프레넷 가중치 ====")
    if not best:
        print("두 맵 모두에서 유효한 랩타임이 있는 파라미터 세트가 없습니다.")
    else:
        for i, b in enumerate(best[:10], start=1):
            times_str = " | ".join(
                f"{m}={b[f'time_{m}']:.2f}s" for m in map_names
            )
            print(
                f"[{i:02d}] SPD={b['spd']:.2f}, TGT={b['tgt']:.2f}, "
                f"ACC={b['acc']:.2f}, CRV={b['crv']:.2f} "
                f"|| total={b['total_time']:.2f}s || {times_str}"
            )

    # 4) 재실험이 필요한 파라미터 세트 출력
    reruns = find_rerun_candidates(analyses, map_names=map_names)
    print("\n==== 재실험이 필요한 (파라미터, 맵) 목록 ====")
    if not reruns:
        print("재실험이 필요한 조합이 없습니다.")
    else:
        for r in reruns:
            print(
                f"map={r['map']:12s} | SPD={r['spd']:.2f}, "
                f"TGT={r['tgt']:.2f}, ACC={r['acc']:.2f}, CRV={r['crv']:.2f}"
            )


if __name__ == "__main__":
    main()
