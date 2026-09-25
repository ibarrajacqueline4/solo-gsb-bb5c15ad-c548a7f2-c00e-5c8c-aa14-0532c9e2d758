"""计划切换绿波发车窗口核心算法测试。

确定性用例覆盖：switch_at 在范围之外（纯旧/纯新计划）、切换点两侧绿灯相接
合并为连续分量、抵达波动区间跨切换点时两侧分别检查（不能只按最早或最晚
抵达选定计划）、到达时刻恰为 switch_at 时用新计划、新计划不同周期、跨零点
绿灯、常绿、多路口联动与无解。另含 600 组随机参数下与逐秒 × 切换点分段
判定参考实现的逐项对照。
"""

import random

from app.greenwave import compute_plan_switch_windows


def it(next_plan=None, **kw):
    # 默认旧计划整周期常绿、next_plan 与旧计划相同，所有取值均满足 schema 约束。
    node = {
        "cycle": 60, "offset": 0, "green_start": 0, "green_end": 0,
        "travel": 0, **kw,
    }
    if next_plan is None:
        next_plan = {
            "cycle": node["cycle"],
            "offset": node["offset"],
            "green_start": node["green_start"],
            "green_end": node["green_end"],
        }
    node["next_plan"] = next_plan
    return node


# ---------- switch_at 在范围之外：退化为单一计划 ----------

def test_pure_old_plan_when_switch_far_future():
    # switch_at 远大于任何到达时刻：全部按旧计划 [0,30) 判定
    nodes = [it(
        green_start=0, green_end=30,
        next_plan={"cycle": 60, "offset": 0, "green_start": 10, "green_end": 40},
    )]
    assert compute_plan_switch_windows(nodes, 0, 120, 100000) == [
        [0, 30], [60, 90],
    ]


def test_pure_new_plan_when_switch_at_zero():
    # switch_at = 0：所有到达时刻都 >= switch_at，全部按新计划 [10,40) 判定
    nodes = [it(
        green_start=0, green_end=30,
        next_plan={"cycle": 60, "offset": 0, "green_start": 10, "green_end": 40},
    )]
    assert compute_plan_switch_windows(nodes, 0, 120, 0) == [
        [10, 40], [70, 100],
    ]


# ---------- 切换点两侧绿灯相接：合并为一条连续分量 ----------

def test_merge_at_switch_point_forms_continuous_green():
    # 旧计划绿灯 [0,30)、新计划 [30,50)，switch_at=30，行驶时间 [0,10]：
    # 有效绿灯在切换点相接合并为 [0,50)，收缩 10 秒波动后窗口为 [0,40)；
    # t=25 的抵达区间 [25,35] 跨切换点，两侧分别按各自计划遇绿，故可行。
    nodes = [it(
        green_start=0, green_end=30, travel=0, travel_max=10,
        next_plan={"cycle": 60, "offset": 0, "green_start": 30, "green_end": 50},
    )]
    assert compute_plan_switch_windows(nodes, 0, 120, 30) == [
        [0, 40], [90, 100],
    ]


# ---------- 切换点落在抵达波动区间内：两侧都必须检查 ----------

def test_both_sides_checked_when_switch_inside_arrival_interval():
    # 新旧计划绿灯同为 [0,30)、switch_at=30、行驶时间 [0,10]：
    # t=25 的抵达区间 [25,35] 跨切换点，30..35 按新计划为红灯，不可行；
    # 只按最早或最晚抵达选定一套计划都会给出错误答案。
    nodes = [it(
        green_start=0, green_end=30, travel=0, travel_max=10,
        next_plan={"cycle": 60, "offset": 0, "green_start": 0, "green_end": 30},
    )]
    assert compute_plan_switch_windows(nodes, 0, 120, 30) == [
        [0, 20], [60, 80],
    ]


def test_far_side_red_blocks_crossing_departure():
    # 旧计划绿灯 [0,40)、新计划 [20,30)，switch_at=35，行驶时间 [0,10]：
    # t=25 的抵达区间 [25,35] 中到达时刻 35 按新计划（相位 35 红灯）不可行，
    # 窗口右端由旧计划绿灯终点 40 进一步被切换点压回 25。
    nodes = [it(
        green_start=0, green_end=40, travel=0, travel_max=10,
        next_plan={"cycle": 60, "offset": 0, "green_start": 20, "green_end": 30},
    )]
    assert compute_plan_switch_windows(nodes, 0, 120, 35) == [[0, 25]]


def test_arrival_exactly_at_switch_uses_new_plan():
    # 旧计划常绿、新计划绿灯 [0,10)，switch_at=20，无波动：
    # 到达时刻 20 恰好等于 switch_at，按新计划判定（相位 20 红灯），
    # 故 t=20 不可行，窗口为 [0,20)。
    nodes = [it(
        next_plan={"cycle": 60, "offset": 0, "green_start": 0, "green_end": 10},
    )]
    assert compute_plan_switch_windows(nodes, 0, 60, 20) == [[0, 20]]


def test_slack_spanning_switch_leaves_red_gap():
    # 旧计划绿灯 [0,30)、新计划常绿，switch_at=35，行驶时间 [0,10]：
    # 抵达区间跨切换点时 30..34 仍按旧计划（红灯），故 (20,35) 不可行。
    nodes = [it(
        green_start=0, green_end=30, travel=0, travel_max=10,
        next_plan={"cycle": 60, "offset": 0, "green_start": 0, "green_end": 0},
    )]
    assert compute_plan_switch_windows(nodes, 0, 120, 35) == [
        [0, 20], [35, 120],
    ]


# ---------- 新计划可采用不同周期 ----------

def test_next_plan_may_have_different_cycle():
    # 旧计划周期 60、绿灯 [0,30)；新计划周期 40、绿灯 [0,20)；switch_at=100。
    # 切换后按 40 秒周期判定：仅 [120,140) 内出发全程遇绿。
    nodes = [it(
        green_start=0, green_end=30,
        next_plan={"cycle": 40, "offset": 0, "green_start": 0, "green_end": 20},
    )]
    assert compute_plan_switch_windows(nodes, 90, 150, 100) == [[120, 140]]


# ---------- 跨零点绿灯与常绿 ----------

def test_wrapping_old_plan_and_always_green_next_plan():
    # 旧计划绿灯跨零点 [50,60)∪[0,10)（连续段 [50,70)），新计划常绿，
    # switch_at=65：旧分量 [50,70) 截断为 [50,65) 后与常绿新分量在 65
    # 相接合并，[50,120) 内出发全程遇绿。
    nodes = [it(
        green_start=50, green_end=10,
        next_plan={"cycle": 60, "offset": 0, "green_start": 0, "green_end": 0},
    )]
    assert compute_plan_switch_windows(nodes, 0, 120, 65) == [
        [0, 10], [50, 120],
    ]


def test_both_plans_always_green_full_range():
    nodes = [it(
        next_plan={"cycle": 60, "offset": 0, "green_start": 5, "green_end": 5},
    )]
    assert compute_plan_switch_windows(nodes, 10, 200, 55) == [[10, 200]]


# ---------- 多路口联动与无解 ----------

def test_two_intersections_intersected():
    # 路口 1：新旧绿灯均 [0,40)；路口 2：新旧绿灯均 [10,50)；switch_at=30。
    # 路口 1 有效窗口 [0,40) [60,100)，路口 2 为 [10,50) [70,110)。
    nodes = [
        it(
            green_start=0, green_end=40,
            next_plan={"cycle": 60, "offset": 0,
                       "green_start": 0, "green_end": 40},
        ),
        it(
            green_start=10, green_end=50,
            next_plan={"cycle": 60, "offset": 0,
                       "green_start": 10, "green_end": 50},
        ),
    ]
    assert compute_plan_switch_windows(nodes, 0, 120, 30) == [
        [10, 40], [70, 100],
    ]


def test_empty_array_on_no_solution():
    # 切换前后绿灯均为 [0,10)：发车范围 [40,60) 内任何到达时刻都遇红。
    nodes = [it(
        green_start=0, green_end=10,
        next_plan={"cycle": 60, "offset": 0, "green_start": 0, "green_end": 10},
    )]
    assert compute_plan_switch_windows(nodes, 40, 60, 50) == []


# ---------- 随机用例：与逐秒 × 切换点分段判定的参考实现对照 ----------

def test_plan_switch_brute_force_equivalence_on_random_cases():
    """与逐秒判定的参考实现完全一致（测试本身才允许逐秒）。

    参考实现把抵达闭区间 [t+T, t+T_max] 在 switch_at 处分成两段：
    小于 switch_at 的部分必须整体落在旧计划的某个绿灯连续分量内，
    等于或大于 switch_at 的部分必须整体落在新计划的某个绿灯连续
    分量内——两侧分别检查，不只按最早或最晚抵达选定计划。
    """
    random.seed(2026)

    def covered(plan, start, end):
        """闭区间 [start, end] 内的所有实数时刻是否都在该计划的某个绿灯
        连续分量内（start <= end）。"""
        c, o = plan["cycle"], plan["offset"]
        gs, ge = plan["green_start"], plan["green_end"]
        if gs == ge:
            return True
        for q in range(start // c - 2, end // c + 4):
            if gs < ge:
                a, b = q * c + gs - o, q * c + ge - o
            else:  # 跨零点：连续分量 [qC+gs-o, (q+1)C+ge-o)
                a, b = q * c + gs - o, (q + 1) * c + ge - o
            if a <= start and end < b:
                return True
        return False

    def feasible(node, switch_at, t):
        """整数出发时刻 t 在任意实数行驶时间下是否对该路口可行。"""
        tmax = node.get("travel_max", node["travel"])
        arrive_lo = t + node["travel"]
        arrive_hi = t + tmax
        # 到达时刻 < switch_at 的部分按旧计划判定（闭区间 [arrive_lo, old_hi]）
        old_hi = min(arrive_hi, switch_at - 1)
        if arrive_lo <= old_hi and not covered(node, arrive_lo, old_hi):
            return False
        # 到达时刻 >= switch_at 的部分按新计划判定（闭区间 [new_lo, arrive_hi]）
        new_lo = max(arrive_lo, switch_at)
        if new_lo <= arrive_hi and not covered(
            node["next_plan"], new_lo, arrive_hi
        ):
            return False
        return True

    def brute(nodes, switch_at, lo, hi):
        expected = []
        run_start = None
        for t in range(lo, hi + 1):
            ok = t < hi and all(feasible(n, switch_at, t) for n in nodes)
            if ok:
                if run_start is None:
                    run_start = t
            else:
                if run_start is not None:
                    expected.append([run_start, t])
                    run_start = None
        if run_start is not None:
            expected.append([run_start, hi])
        return expected

    for _ in range(600):
        nodes = []
        for _ in range(random.randint(1, 8)):
            c = random.randint(1, 90)
            travel = random.randint(0, 120)
            kwargs = dict(
                cycle=c,
                offset=random.randint(0, c - 1),
                green_start=random.randint(0, c - 1),
                green_end=random.randint(0, c - 1),
                travel=travel,
            )
            # 约一半用例带行驶时间波动
            if random.random() < 0.5:
                kwargs["travel_max"] = travel + random.randint(0, 30)
            nc = random.randint(1, 90)
            next_plan = {
                "cycle": nc,
                "offset": random.randint(0, nc - 1),
                "green_start": random.randint(0, nc - 1),
                "green_end": random.randint(0, nc - 1),
            }
            nodes.append(it(next_plan=next_plan, **kwargs))
        length = random.randint(1, 600)
        lo = random.randint(0, 300)
        hi = lo + length
        r = random.random()
        if r < 0.1:
            switch_at = 0  # 纯新计划
        elif r < 0.2:
            switch_at = hi + 200  # 纯旧计划
        else:
            switch_at = random.randint(max(0, lo - 100), hi + 200)
        assert compute_plan_switch_windows(nodes, lo, hi, switch_at) == brute(
            nodes, switch_at, lo, hi
        )
