"""干线绿波发车窗口核心算法。

所有时间量均为整数秒，路口信号以固定周期循环。

相位约定
--------
对路口 i，给定：

* ``cycle``  : 信号周期 C（秒，正整数）；
* ``offset`` : 相位偏移 o（0 <= o < C），绝对时间 t 在该路口的本地相位为
               ``(t + o) mod C``；
* ``green_start`` / ``green_end`` : 周期内绿灯半开区间端点，取值 [0, C)：
    - g_start < g_end：绿灯为 ``[g_start, g_end)``；
    - g_start > g_end：绿灯跨零点，为 ``[g_start, C) ∪ [0, g_end)``；
    - g_start == g_end：整周期常绿 ``[0, C)``；
* ``travel`` : 从起点到该路口的行驶秒数下限 T（默认即固定行驶时间）；
* ``travel_max`` : 可选的行驶秒数上限，缺省等于 ``travel``。实际行驶时间
  可取闭区间 ``[T, T_max]`` 内的**任意实数**秒，出发窗口必须对其中每一种
  取值都保证遇绿。

出发时刻为 t 时，抵达时刻覆盖闭区间 ``[t + T, t + T_max]``。令波动量
``slack = T_max - T``，在展开（不绕零点）的时间轴上，绿灯半开区间 ``[a,b)``
能容下整个抵达闭区间，当且仅当 ``a <= t+T+o`` 且 ``t+T_max+o < b``，即
绿灯段被右端收缩为 ``[a, b - slack)``（slack >= 段长时该路口永不可行）。
跨零点绿灯须先视为连续时段 ``[g_start, C + g_end)`` **再收缩**，不得拆成
两段分别收缩；整周期常绿不受任何波动影响。每个路口收缩后的可行出发时刻是
一组周期重复的半开整数区间 ``[kC + a - shift, kC + b - shift)``
（shift = (T + o) mod C），再与请求的发车范围
``[depart_from, depart_to)`` 求交；各路口结果两两求交并合并相邻
（端点相接）区间，即得全部极大半开发车窗口。全程只做区间运算，不逐秒试探。

最大覆盖
--------
:func:`compute_max_coverage` 复用上述**单路口可行窗口**：对范围内每个整数
出发时刻，统计"行驶时间取 [T, T_max] 内任意实数仍遇绿"的路口数（覆盖数）。
覆盖数只可能在某个可行窗口的端点处变化，故把每个半开窗口 ``[a,b)`` 记为
``a`` 处 +1、``b`` 处 -1 的差分事件，按位置排序后逐段扫描即可，不逐秒统计。
取全局最大覆盖数，合并相邻（端点相接）的最优段，即得全部极大半开发车区间；
最大值为 0 时没有任何事件，唯一的极大区间即完整发车范围。

整段时隙可调 offset
--------------------
:func:`compute_slot_offsets` 要求发车范围 ``[depart_from, depart_to)`` 内
**每个整数秒**都能发车（其余路口保持原 offset），求唯一可调路口的全部可行
整数 ``offset ∈ [0, C)``。

固定路口直接复用单路口可行窗口：覆盖全部整数秒当且仅当合并后窗口恰为完整
发车范围（半开区间之间任何正长度间隙都至少漏掉一个整数秒）。

可调路口仍受自身行驶波动 ``[T, T_max]`` 约束。波动收缩后的可行本地相位是
周期圆上一条半开弧 E（跨零点绿灯先连续再收缩，常绿为整圆；弧长 L 即
:func:`eroded_green_components` 给出的连续分量长度）。令 n 为发车范围秒数，
``shift = (T + o) mod C``，范围内 n 个整数出发时刻对应的本地相位恰是圆上
n 个连续点 ``[lo+shift, hi+shift) mod C``；它们整体落入弧 E，当且仅当
``L >= n``，且 shift 可取一段长 ``M = L - n + 1`` 的连续闭整数弧（半开表示
为 ``[r0, r0+M) mod C``，``r0 = (e0 - lo) mod C``）。offset 弧由其整体旋转
``-T`` 得到，跨零点时在 ``[0, C)`` 内分两段。``L >= C``（整圆常绿）时每个
offset 都可行；``L < n``（含 n >= C 而非常绿）时可行集为空。

车队首车发车窗口
----------------
:func:`compute_convoy_starts` 复用上述**单车可行窗口**：车队共 n 辆车、以
固定间隔 h 秒依次发车，首车时刻为 t 时第 k 辆（k = 0..n-1）在
``t + k*h`` 出发。所有车辆都必须落在发车范围内，且每辆车在每个路口都满足
单车可行条件（对 ``[T, T_max]`` 内任意实际行驶时间遇绿）。首车时刻 t 可行，
当且仅当每个路口、每个 k 都有 ``t + k*h`` 落在该路口单车可行窗口内，即 t
落在窗口整体左移 ``k*h`` 后的集合中。逐路口把单车窗口按 k = 0..n-1 平移后
求交（中间车辆同样检查，不能只验首末车：周期绿灯的间隙可能恰好卡住中间
某辆车），再跨路口求交，最后裁剪进首车范围
``[depart_from, depart_to - (n-1)*h)`` 并合并相邻（端点相接）区间，即得
全部极大半开首车发车区间。

计划切换发车窗口
----------------
:func:`compute_plan_switch_windows` 在旧计划之外另给全局切换时刻 S
（``switch_at``，整数秒）与每个路口的新计划（各自独立的 cycle / offset /
green_start / green_end）。到达时刻严格小于 S 的部分按旧计划判灯，等于或
大于 S 的部分按新计划判灯。按出发时刻 t 的抵达闭区间
``[t+T, t+T_max]`` 与 S 的关系分三段：

* ``t + T_max < S``（整段抵达都在切换前）：直接复用旧计划的收缩窗口；
* ``t + T >= S``（整段抵达都在切换后）：直接复用新计划的收缩窗口；
* ``t + T < S <= t + T_max``（切换点落在抵达波动区间内）：旧计划侧为
  半开抵达段 ``[t+T, S)``、新计划侧为闭抵达段 ``[S, t+T_max]``，**两侧
  必须分别**整体落入各自计划展开时间轴上的一条连续绿灯分量——不能只按最早
  或最晚抵达选择计划。令 rho 为 S 沿绿灯方向距该计划（跨零点先连续化后的）
  绿灯分量起点的秒数、L 为分量长度：旧侧半开段可行当且仅当常绿或
  ``1 <= rho <= L``（rho=0 时 S 恰为绿灯起点、紧邻前方为红灯；rho=L 时
  S 恰为分量右端不含，半开段仍可整体落入），给出 t 下界
  ``S - T - rho``；新侧闭段可行当且仅当常绿或 ``0 <= rho < L``
  （rho=L 时 S 恰为绿灯终点不含），给出 t 上界
  ``S + L - rho - 1 - T_max``。混合整数段取交至多形成一条半开区间。三段
  各自裁剪、合并后再跨路口求交。
"""

from __future__ import annotations

from typing import Iterable

# 周期上限仅用于防御异常输入，一日之内足够覆盖任何实际信号周期。
MAX_CYCLE = 86_400
MAX_WINDOW_SECONDS = 3_600
MAX_INTERSECTIONS = 8
MAX_OBSERVATIONS = 2_000
MAX_CONVOY_VEHICLES = 50

Interval = list[int]  # 长度为 2 的半开区间 [start, end)


def local_green_intervals(cycle: int, green_start: int, green_end: int) -> list[Interval]:
    """展开单个周期 [0, C) 内的绿灯半开区间。

    端点相等时视为整周期常绿；跨零点时展开为两段。
    """
    if green_start == green_end:
        return [[0, cycle]]
    if green_start < green_end:
        return [[green_start, green_end]]
    return [[green_start, cycle], [0, green_end]]


def merge_intervals(intervals: Iterable[Interval]) -> list[Interval]:
    """合并重叠或端点相接的半开区间（[a,b) 与 [b,c) 合并为 [a,c)）。"""
    merged: list[Interval] = []
    for start, end in sorted(intervals):
        if start >= end:
            continue
        if merged and start <= merged[-1][1]:
            if end > merged[-1][1]:
                merged[-1][1] = end
        else:
            merged.append([start, end])
    return merged


def clip_intervals(
    intervals: Iterable[Interval], lower: int, upper: int
) -> list[Interval]:
    """把半开区间裁剪进 [lower, upper)。"""
    clipped: list[Interval] = []
    for start, end in intervals:
        start = max(start, lower)
        end = min(end, upper)
        if start < end:
            clipped.append([start, end])
    return clipped


def eroded_green_components(
    cycle: int,
    green_start: int,
    green_end: int,
    slack: int,
) -> tuple[bool, list[Interval]]:
    """波动量 slack 下、每个周期内的"连续"绿灯分量（出发轴视角）。

    返回 ``(always_green, components)``：

    * ``green_start == green_end``（整周期常绿）：返回 ``(True, [])``，
      常绿不受行驶时间波动影响；
    * 普通绿灯 ``[g_start, g_end)``：右端收缩为
      ``[g_start, g_end - slack)``，slack >= 绿灯长度时为空；
    * 跨零点绿灯：**先**视为连续时段 ``[g_start, C + g_end)`` **再收缩**
      为 ``[g_start, C + g_end - slack)``（可能长于一个周期）。

    收缩是针对"抵达闭区间整体落入绿灯半开区间"的最坏情形：
    到达最早时刻可压在左端点（含），到达最晚时刻必须严格早于右端点。
    """
    if green_start == green_end:
        return True, []
    if green_start < green_end:
        if slack >= green_end - green_start:
            return False, []
        return False, [[green_start, green_end - slack]]
    # 跨零点：先视为连续的 [g_start, C + g_end)，再做右端收缩。
    extended_end = cycle + green_end - slack
    if extended_end <= green_start:
        return False, []
    return False, [[green_start, extended_end]]


def departure_windows_for_intersection(
    cycle: int,
    offset: int,
    green_start: int,
    green_end: int,
    travel: int,
    depart_from: int,
    depart_to: int,
    travel_max: int | None = None,
) -> list[Interval]:
    """单个路口在 [depart_from, depart_to) 内的全部可行出发区间（已合并）。

    ``travel_max`` 缺省等于 ``travel``（无波动）。行驶时间可能取闭区间
    [travel, travel_max] 内任意实数秒时，只有对所有取值均遇绿的出发时刻
    才会被保留。
    """
    if travel_max is None:
        travel_max = travel
    slack = travel_max - travel
    always_green, components = eroded_green_components(
        cycle, green_start, green_end, slack
    )
    if always_green:
        return [[depart_from, depart_to]]
    # (t + T + o) mod C 与 (t + shift) mod C 等价；slack 只影响绿灯长度。
    shift = (travel + offset) % cycle

    raw: list[Interval] = []
    for local_start, local_end in components:
        # 本地（已收缩）绿灯 [a, b) 对应出发区间 [kC + a - shift, kC + b - shift)。
        # 只需枚举与发车范围有非空交的周期 k：
        #   kC + b - shift > depart_from  ⇔  k > (depart_from - b + shift) / C
        #   kC + a - shift < depart_to    ⇔  k < (depart_to   - a + shift) / C
        k_min = (depart_from - local_end + shift) // cycle + 1
        k_max = -((-(depart_to - local_start + shift)) // cycle) - 1
        for k in range(k_min, k_max + 1):
            base = k * cycle
            raw.append(
                [base + local_start - shift, base + local_end - shift]
            )

    return merge_intervals(clip_intervals(raw, depart_from, depart_to))


def intersect_interval_lists(
    left: list[Interval], right: list[Interval]
) -> list[Interval]:
    """两组各自有序、互不相交的半开区间列表的交集（双指针）。"""
    i = j = 0
    result: list[Interval] = []
    while i < len(left) and j < len(right):
        start = max(left[i][0], right[j][0])
        end = min(left[i][1], right[j][1])
        if start < end:
            result.append([start, end])
        if left[i][1] < right[j][1]:
            i += 1
        else:
            j += 1
    return result


def compute_windows(
    intersections: Iterable[dict],
    depart_from: int,
    depart_to: int,
) -> list[Interval]:
    """求所有路口均遇绿灯的全部极大半开发车区间，按起点排序。

    ``intersections`` 中的每个字典包含 cycle / offset / green_start /
    green_end / travel 字段，以及可选的 travel_max 字段（缺省等于 travel，
    表示行驶时间可能取 [travel, travel_max] 内任意实数秒），由 Pydantic 模型
    保证合法。
    """
    common: list[Interval] | None = None
    for item in intersections:
        windows = departure_windows_for_intersection(
            cycle=item["cycle"],
            offset=item["offset"],
            green_start=item["green_start"],
            green_end=item["green_end"],
            travel=item["travel"],
            travel_max=item.get("travel_max"),
            depart_from=depart_from,
            depart_to=depart_to,
        )
        if common is None:
            common = windows
        else:
            common = intersect_interval_lists(common, windows)
        if not common:
            return []
    # 求交不会产生相接区间，最后再合并一次以策安全。
    return merge_intervals(common or [])


def compute_convoy_starts(
    intersections: Iterable[dict],
    depart_from: int,
    depart_to: int,
    vehicle_count: int,
    headway: int,
) -> list[Interval]:
    """求车队首车发车时刻的全部极大半开区间，按起点排序。

    车队共 ``vehicle_count`` 辆（n >= 2）车，以固定间隔 ``headway`` 秒
    （h >= 1）依次发车：首车时刻为 t 时第 k 辆（k = 0..n-1）在
    ``t + k*h`` 出发。要求每辆车的出发时刻都落在发车范围
    ``[depart_from, depart_to)`` 内（等价于首车范围
    ``[depart_from, depart_to - (n-1)*h)``，由请求侧保证其非空），且每辆车
    在每个路口都满足单车可行条件（对 [travel, travel_max] 内任意实际行驶
    时间遇绿，语义与 :func:`compute_windows` 完全一致）。

    首车时刻 t 可行，当且仅当对每个路口、每个 k，``t + k*h`` 都落在该路口
    的单车可行窗口内，即 t 落在窗口整体左移 ``k*h`` 后的集合中。逐路口先把
    单车窗口按 k = 0..n-1 平移后求交——中间车辆同样检查，不能只验首末车：
    周期绿灯的间隙可能恰好卡住中间某辆车（首末车均遇绿而中间车遇红）——
    再跨路口求交，最后裁剪进首车范围并合并相邻（端点相接）区间。全程只做
    区间平移与求交，不逐秒试探；无任何可行首车时刻时返回空数组。
    """
    span = (vehicle_count - 1) * headway
    first_to = depart_to - span  # 请求侧已保证 span < 范围长度，故 first_to > depart_from
    common: list[Interval] | None = None
    for item in intersections:
        windows = departure_windows_for_intersection(
            cycle=item["cycle"],
            offset=item["offset"],
            green_start=item["green_start"],
            green_end=item["green_end"],
            travel=item["travel"],
            travel_max=item.get("travel_max"),
            depart_from=depart_from,
            depart_to=depart_to,
        )
        # 第 k 辆车在 t + k*h 出发 ⇔ 首车时刻 t 落在窗口左移 k*h 后的集合中。
        convoy: list[Interval] | None = None
        for k in range(vehicle_count):
            lag = k * headway
            shifted = [[start - lag, end - lag] for start, end in windows]
            if convoy is None:
                convoy = shifted
            else:
                convoy = intersect_interval_lists(convoy, shifted)
            if not convoy:
                break
        if not convoy:
            return []
        if common is None:
            common = convoy
        else:
            common = intersect_interval_lists(common, convoy)
        if not common:
            return []
    # 平移求交已隐含首车范围约束，这里再显式裁剪并合并相邻区间以策安全。
    return merge_intervals(clip_intervals(common or [], depart_from, first_to))


def compute_max_coverage(
    intersections: Iterable[dict],
    depart_from: int,
    depart_to: int,
) -> tuple[int, list[Interval]]:
    """求范围内"最多有多少个路口任意行驶时长下都遇绿"及全部极大最优区间。

    对发车范围 ``[depart_from, depart_to)`` 内的每个整数出发时刻 t，覆盖数
    为满足下述条件的路口数：行驶时间取 ``[T, T_max]`` 内**任意实数**时抵达
    该路口都遇绿灯（即可行窗口的判定与 :func:`compute_windows` 完全一致，
    直接复用 :func:`departure_windows_for_intersection`）。

    返回 ``(最大覆盖数, 极大半开区间列表)``：区间按起点升序，区间内每个出发
    时刻的覆盖数都等于最大值，且区间两端已覆盖数下降或到达发车范围边界。
    采用区间端点事件扫描（每个可行窗口在起点 +1、终点 -1），覆盖数只在端点
    处变化，故不逐秒统计；最大值为 0（无任何事件）时返回
    ``(0, [[depart_from, depart_to]])``，即完整发车范围仍是唯一的极大区间。
    """
    # 事件：(位置, 增量)。半开窗口 [a,b) 内贡献 1，故 a 处 +1、b 处 -1。
    events: list[tuple[int, int]] = []
    for item in intersections:
        for start, end in departure_windows_for_intersection(
            cycle=item["cycle"],
            offset=item["offset"],
            green_start=item["green_start"],
            green_end=item["green_end"],
            travel=item["travel"],
            travel_max=item.get("travel_max"),
            depart_from=depart_from,
            depart_to=depart_to,
        ):
            events.append((start, 1))
            events.append((end, -1))

    if not events:
        # 没有任何路口在范围内可行（含范围上的退化情形）：覆盖数恒为 0。
        return 0, [[depart_from, depart_to]]

    events.sort()
    segments: list[tuple[int, int, int]] = []  # (起点, 终点, 覆盖数)
    coverage = 0
    prev = depart_from
    i = 0
    while i < len(events):
        pos = events[i][0]
        if prev < pos:
            segments.append((prev, pos, coverage))
        delta = 0
        while i < len(events) and events[i][0] == pos:
            delta += events[i][1]
            i += 1
        coverage += delta
        prev = pos
    if prev < depart_to:
        segments.append((prev, depart_to, coverage))

    maximum = max(seg[2] for seg in segments)
    # 合并相邻（端点相接）且同为最大覆盖数的段，得到全部极大半开区间。
    optimal: list[Interval] = []
    for start, end, value in segments:
        if value != maximum:
            continue
        if optimal and optimal[-1][1] == start:
            optimal[-1][1] = end
        else:
            optimal.append([start, end])
    return maximum, optimal


# ---------- 单路口 offset 诊断 ----------

Observation = tuple[int, bool]  # (arrival_time, observed_green)


def _shifted_circle_intervals(
    cycle: int,
    green_start: int,
    green_end: int,
    shift: int,
) -> list[Interval]:
    """周期圆上的绿灯集合平移 -shift 后、落在 [0, C) 内的半开整数区间。

    即满足 ``(p + shift) mod C`` 落在绿灯集合中的 p 集合
    （本地绿灯 ``[a,b)`` 平移后为 ``[a-shift, b-shift) mod C``）。
    ``green_start == green_end``（整周期常绿）时返回 ``[[0, C]]``；
    跨零点的平移结果可能在 [0, C) 内断成两段。
    """
    if green_start == green_end:
        return [[0, cycle]]
    shift %= cycle
    intervals: list[Interval] = []
    # 非跨零点绿灯 [gs, ge) 是一段；跨零点先视为连续段 [gs, C+ge)。
    raw = (
        [(green_start, green_end)]
        if green_start < green_end
        else [(green_start, cycle + green_end)]
    )
    for a, b in raw:
        lo = (a - shift) % cycle
        hi = (b - shift) % cycle
        length = b - a  # 非常绿段长度必小于 C，平移后至多绕零点一次
        if lo < hi:
            intervals.append([lo, hi])
        else:
            # 半开长度不变：从 lo 起 length 秒，跨零点则断成两段。
            intervals.append([lo, min(cycle, lo + length)])
            tail = (lo + length) % cycle
            if tail:
                intervals.append([0, tail])
    return merge_intervals(intervals)


def _complement_in_cycle(intervals: list[Interval], cycle: int) -> list[Interval]:
    """[0, C) 内一组有序互不相交半开区间的补集（同为半开区间）。"""
    result: list[Interval] = []
    cursor = 0
    for start, end in intervals:
        if cursor < start:
            result.append([cursor, start])
        cursor = end
    if cursor < cycle:
        result.append([cursor, cycle])
    return result


def _mismatch_intervals_for_observation(
    cycle: int,
    green_start: int,
    green_end: int,
    arrival_time: int,
    observed_green: bool,
) -> list[Interval]:
    """单条观测的"预测灯色 != 观测灯色" offset 集合（[0,C) 内半开区间）。

    offset 预测灯色为绿，当且仅当 ``(arrival_time + offset) mod C``
    落在绿灯集合中；观测为绿时错配集合为其补集（预测为红），观测为红时
    错配集合即预测为绿的集合。
    """
    green_offsets = _shifted_circle_intervals(
        cycle, green_start, green_end, arrival_time % cycle
    )
    if not observed_green:
        return green_offsets
    return _complement_in_cycle(green_offsets, cycle)


def offset_diagnostics(
    cycle: int,
    green_start: int,
    green_end: int,
    observations: Iterable[Observation],
) -> tuple[int, list[Interval]]:
    """求最少错配条数及取到最小值的全部极大半开 offset 区间。

    对整数 ``offset ∈ [0, C)``，按 ``(arrival_time + offset) mod C``
    预测灯色（绿灯半开区间、跨零点、端点相等为常绿的语义与其余接口一致），
    统计预测与 ``observed_green`` 不符的观测条数。返回
    ``(最少错配条数, 最优 offset 区间列表)``，区间按起点升序。

    采用周期区间事件扫描：每条观测的错配集合至多两条半开区间，仅在区间
    端点放置 +1/-1 差分事件，按事件位置排序后逐段维护当前错配数，全程不
    枚举 offset × 观测（复杂度 O(n log n)，与 cycle 大小无关）。跨零点的
    最优集合天然在 [0, C) 内分段表示；同错配数的相邻段合并为极大区间。
    """
    # 事件：(位置, 增量)。错配数只在整数区间端点处变化，段内恒定。
    events: list[tuple[int, int]] = []
    for arrival_time, observed_green in observations:
        for start, end in _mismatch_intervals_for_observation(
            cycle, green_start, green_end, arrival_time, observed_green
        ):
            events.append((start, 1))
            events.append((end, -1))

    if not events:
        # 无任何错配事件 ⇔ 所有观测均与整周期常绿一致：每个 offset 都最优。
        return 0, [[0, cycle]]

    events.sort()
    segments: list[tuple[int, int, int]] = []  # (起点, 终点, 错配数)
    mismatches = 0
    prev = 0
    i = 0
    while i < len(events):
        pos = events[i][0]
        if prev < pos:
            segments.append((prev, pos, mismatches))
        delta = 0
        while i < len(events) and events[i][0] == pos:
            delta += events[i][1]
            i += 1
        mismatches += delta
        prev = pos
    if prev < cycle:
        segments.append((prev, cycle, mismatches))

    minimum = min(seg[2] for seg in segments)
    # 合并相邻且同为最小错配数的段，得到全部极大半开区间。
    optimal: list[Interval] = []
    for start, end, value in segments:
        if value != minimum:
            continue
        if optimal and optimal[-1][1] == start:
            optimal[-1][1] = end
        else:
            optimal.append([start, end])
    return minimum, optimal


# ---------- 整段时隙可调 offset ----------


def _arc_intervals(start: int, length: int, cycle: int) -> list[Interval]:
    """周期圆上自 start 起 length 个整数的连续弧，用 [0,C) 内半开区间表示。

    length >= cycle 时整圆为 ``[[0, cycle]]``；跨零点时分两段，按起点升序
    （``[0, tail]`` 在前、``[start, cycle]`` 在后）。
    """
    if length >= cycle:
        return [[0, cycle]]
    start %= cycle
    tail = start + length
    if tail <= cycle:
        return [[start, tail]]
    return [[0, tail - cycle], [start, cycle]]


def _recommend_offset(
    intervals: list[Interval], original: int, cycle: int
) -> int:
    """从可行 offset 区间（[0,C) 内有序半开区间，至多跨零点两段）中取建议值。

    建议值为距 ``original`` 环形距离最小的可行整数 offset；正向（增大）与
    反向（减小，绕零点）等距时取较小的 offset。``original`` 自身可行时
    距离为 0，直接返回；否则只需比较环形正向最近点与环形反向最近点。
    """
    for start, end in intervals:
        if start <= original < end:
            return original

    # 环形正向最近点：线性意义下第一个起点不小于 original 的段；没有则绕到
    # 整圆上最小的可行点（首段起点）。
    forward = next(
        (start for start, end in intervals if start >= original),
        intervals[0][0],
    )
    # 环形反向最近点：线性意义下最后一个整段位于 original 之前的段末点；
    # 没有则绕到整圆上最大的可行点（末段终点 - 1）。
    backward = next(
        (end - 1 for start, end in reversed(intervals) if end - 1 <= original),
        intervals[-1][1] - 1,
    )

    def ring_distance(value: int) -> int:
        delta = (value - original) % cycle
        return min(delta, cycle - delta)

    candidates = [forward, backward]
    return min(candidates, key=lambda value: (ring_distance(value), value))


def compute_slot_offsets(
    intersections: Iterable[dict],
    depart_from: int,
    depart_to: int,
    adjustable_index: int,
) -> tuple[list[Interval], int | None]:
    """求唯一可调路口的全部可行整数 offset（整段时隙语义）及建议值。

    要求发车范围 ``[depart_from, depart_to)`` 内**每个整数秒**出发，在各
    路口任意实际行驶时间（``[T, T_max]`` 内任意实数）下都遇绿。除
    ``intersections[adjustable_index]`` 外的路口保持请求中的原 offset；
    可调路口的原 offset 仅用于给出环形距离最小的建议值，不参与可行性。

    返回 ``(极大半开 offset 区间列表, 建议 offset)``：区间内每个整数
    offset 均可行，按起点升序，跨零点的可行弧在 ``[0, cycle)`` 内分两段；
    无任何可行 offset（含某个固定路口无法覆盖整段、可调路口收缩后绿灯
    长度不足）时返回 ``([], None)``。
    """
    nodes = list(intersections)
    for index, item in enumerate(nodes):
        if index == adjustable_index:
            continue
        windows = departure_windows_for_intersection(
            cycle=item["cycle"],
            offset=item["offset"],
            green_start=item["green_start"],
            green_end=item["green_end"],
            travel=item["travel"],
            travel_max=item.get("travel_max"),
            depart_from=depart_from,
            depart_to=depart_to,
        )
        # 半开区间之间任何正长度间隙都会漏掉至少一个整数秒；覆盖全部整数秒
        # 当且仅当合并结果恰为完整的 [depart_from, depart_to)。
        if windows != [[depart_from, depart_to]]:
            return [], None

    target = nodes[adjustable_index]
    cycle = target["cycle"]
    travel_max = target.get("travel_max")
    if travel_max is None:
        travel_max = target["travel"]
    slack = travel_max - target["travel"]
    always_green, components = eroded_green_components(
        cycle, target["green_start"], target["green_end"], slack
    )
    if always_green:
        intervals = [[0, cycle]]
    else:
        # 跨零点绿灯在 eroded_green_components 中已先视为连续段再收缩，
        # 这里至多得到一条分量 [e0, e0+L)（L 为收缩后半开长度，< C）。
        span = depart_to - depart_from
        if not components:
            return [], None
        e0, e1 = components[0]
        feasible_count = (e1 - e0) - span + 1
        if feasible_count <= 0:
            return [], None
        # 可行 shift 弧起点 r0 = (e0 - lo) mod C（hi 端点已计入 M = L-n+1）；
        # offset = shift - T (mod C)。
        origin = (e0 - depart_from - target["travel"]) % cycle
        intervals = _arc_intervals(origin, feasible_count, cycle)

    recommended = _recommend_offset(
        intervals, target["offset"], cycle
    )
    return intervals, recommended


# ---------- 计划切换发车窗口 ----------


def _switch_phase_position(
    cycle: int,
    green_start: int,
    green_end: int,
    switch_at: int,
    offset: int,
) -> tuple[bool, int, int]:
    """切换时刻 S 在某计划绿灯周期结构中的位置。

    返回 ``(always_green, rho, length)``：

    * ``green_start == green_end``（整周期常绿）时返回
      ``(True, 0, cycle)``；
    * 否则跨零点绿灯先视为连续段（``[g_start, C) ∪ [0, g_end)`` 等价于
      一条长度 ``L = (green_end - green_start) mod C`` 的连续分量），
      ``rho = ((S + offset) mod C - green_start) mod C`` 即 S 沿绿灯方向
      距分量起点的秒数，``length = L``。

    旧计划侧（半开抵达段在 S 前结束）存在右端不早于 S 的连续绿灯分量，
    当且仅当 ``1 <= rho <= L``；新计划侧（闭抵达段自 S 开始）存在覆盖 S
    的连续绿灯分量，当且仅当 ``0 <= rho < L``。rho=0 时 S 恰为绿灯起点
    （其紧邻前方是红灯），rho=L 时 S 恰为绿灯终点（不含）。
    """
    if green_start == green_end:
        return True, 0, cycle
    length = (green_end - green_start) % cycle
    local_phase = (switch_at + offset) % cycle
    rho = (local_phase - green_start) % cycle
    return False, rho, length


def compute_plan_switch_windows(
    intersections: Iterable[dict],
    depart_from: int,
    depart_to: int,
    switch_at: int,
) -> list[Interval]:
    """求计划在 ``switch_at`` 时刻切换时的全部极大半开发车区间。

    每个路口字典沿用 cycle / offset / green_start / green_end / travel /
    travel_max（缺省等于 travel）描述**旧计划**，另含 ``next_plan`` 字典
    （cycle / offset / green_start / green_end）描述新计划。车辆到达时刻
    严格小于 ``switch_at`` 时按旧计划判灯，等于或大于时按新计划判灯；
    绿灯的半开与跨周期语义与 :func:`compute_windows` 完全一致。

    对整数出发时刻 t，实际行驶时间可取闭区间 ``[T, T_max]`` 内任意实数，
    抵达时刻覆盖闭区间 ``[t+T, t+T_max]``，分三种情形：

    1. ``t + T_max < switch_at``：全部抵达都在切换前，t 落在旧计划的
       收缩可行窗口内（复用 :func:`departure_windows_for_intersection`）；
    2. ``t + T >= switch_at``：全部抵达都在切换后，t 落在新计划的收缩
       可行窗口内；
    3. ``t + T < switch_at <= t + T_max``：切换点落在抵达波动区间内，
       旧计划侧半开段 ``[t+T, S)`` 与新计划侧闭段 ``[S, t+T_max]``
       **分别**都必须整体落入各自计划展开时间轴上的一条连续绿灯分量
       （不能只按最早或最晚抵达选择一侧）。令 rho 为 S 在该计划下沿绿灯
       方向距（跨零点先连续化后的）绿灯分量起点的秒数、L 为该分量长度：
       旧侧半开段可行当且仅当常绿或 ``1 <= rho <= L``（rho=0 时 S 恰为
       绿灯起点、其紧邻前方是红灯），给出 t 下界 ``S - T - rho``；
       新侧闭段可行当且仅当常绿或 ``0 <= rho < L``（rho=L 时 S 恰为绿灯
       终点不含），给出 t 上界 ``S + L - rho - 1 - T_max``。混合整数段
       为 ``t ∈ [S - T_max, S - T)``，与旧侧下界、新侧上界取交至多形成
       一条半开整数区间。

    三段结果裁剪进 ``[depart_from, depart_to)`` 合并后，跨路口两两求交，
    即得全部极大半开发车区间；无解返回空数组。全程只做区间运算，不逐秒
    试探。
    """
    common: list[Interval] | None = None
    for item in intersections:
        travel = item["travel"]
        travel_max = item.get("travel_max")
        if travel_max is None:
            travel_max = travel
        slack = travel_max - travel

        old_windows = departure_windows_for_intersection(
            cycle=item["cycle"],
            offset=item["offset"],
            green_start=item["green_start"],
            green_end=item["green_end"],
            travel=travel,
            travel_max=travel_max,
            depart_from=depart_from,
            depart_to=depart_to,
        )
        nxt = item["next_plan"]
        new_windows = departure_windows_for_intersection(
            cycle=nxt["cycle"],
            offset=nxt["offset"],
            green_start=nxt["green_start"],
            green_end=nxt["green_end"],
            travel=travel,
            travel_max=travel_max,
            depart_from=depart_from,
            depart_to=depart_to,
        )

        # 情形 1：t + T_max < S ⇔ 整数 t <= S - T_max - 1，半开上界 S-T_max。
        old_only = clip_intervals(
            old_windows, depart_from, switch_at - travel_max
        )
        # 情形 2：t + T >= S ⇔ 整数 t >= S - T（t+T=S 归新计划）。
        new_only = clip_intervals(new_windows, switch_at - travel, depart_to)

        # 情形 3：混合段，整数 t ∈ [S - T_max, S - T)。
        mixed: list[Interval] = []
        # slack == 0 时该整数区间为空（下界 == 上界），无需两侧分别检查。
        if slack > 0:
            # 旧计划侧：半开抵达段 [t+T, S) 须整体落入右端不早于 S 的一条
            # 连续绿灯分量内（各计划按自身 offset 计算 S 的本地相位）。
            old_always, old_rho, old_length = _switch_phase_position(
                item["cycle"],
                item["green_start"],
                item["green_end"],
                switch_at,
                item["offset"],
            )
            # 新计划侧：闭抵达段 [S, t+T_max] 须整体落在覆盖 S 的一条连续
            # 绿灯分量内。
            new_always, new_rho, new_length = _switch_phase_position(
                nxt["cycle"],
                nxt["green_start"],
                nxt["green_end"],
                switch_at,
                nxt["offset"],
            )

            lo = switch_at - travel_max
            hi = switch_at - travel  # 半开 [lo, hi)

            lower = lo
            upper = hi - 1
            feasible = True
            if old_always:
                pass  # 常绿：旧侧不增加约束
            elif 1 <= old_rho <= old_length:
                # 右端不早于 S 的旧分量左端点 a* = S - rho。半开抵达段
                # [t+T, S) 整体落入该分量当且仅当 t + T >= a*
                # ⇔ t >= S - T - rho（对 rho=L：S 恰为分量右端不含，
                # [t+T, S) ⊂ [a*, S) 仍成立——到达最晚的实数时刻可任意
                # 逼近 S）。
                lower = max(lower, switch_at - travel - old_rho)
            else:
                # rho=0 时 S 恰为绿灯起点、其紧邻前方是红灯；rho>L 时 S 在
                # 红灯段内：非空半开段 [t+T, S) 无法整体落入绿灯。
                feasible = False
            if new_always:
                pass  # 常绿：新侧不增加约束
            elif 0 <= new_rho < new_length:
                # 覆盖 S 的分量右端点 b* = S + L - rho。闭抵达段
                # [S, t+T_max] 整体落入 [a*, b*) 当且仅当 t + T_max < b*
                # ⇔ t <= S + L - rho - 1 - T_max（闭右端必须严格小于 b*）。
                upper = min(
                    upper,
                    switch_at + new_length - new_rho - 1 - travel_max,
                )
            else:
                # rho=L 时 S 恰为绿灯终点（不含），否则 S 在红灯段内：
                # 闭段 [S, t+T_max] 无法整体落入绿灯。
                feasible = False

            if feasible and lower <= upper:
                mixed = [[lower, upper + 1]]

        windows = merge_intervals(
            clip_intervals(old_only + new_only + mixed, depart_from, depart_to)
        )
        if common is None:
            common = windows
        else:
            common = intersect_interval_lists(common, windows)
        if not common:
            return []
    return merge_intervals(common or [])
