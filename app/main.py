"""FastAPI 应用入口。"""

from __future__ import annotations

from fastapi import FastAPI

from .greenwave import (
    compute_convoy_starts,
    compute_max_coverage,
    compute_plan_switch_windows,
    compute_slot_offsets,
    compute_windows,
    offset_diagnostics,
)
from .schemas import (
    ConvoyStartsRequest,
    GreenWaveRequest,
    MaxCoverageResponse,
    OffsetDiagnosticsRequest,
    OffsetDiagnosticsResponse,
    OffsetInterval,
    PlanSwitchRequest,
    SlotOffsetsRequest,
    SlotOffsetsResponse,
    Window,
)

app = FastAPI(
    title="干线绿波发车窗口 API",
    description=(
        "给定干线上至多 8 个路口的信号周期、相位偏移、周期内绿灯半开区间"
        "（可跨周期零点）及行驶秒数（可选上限 travel_max，表示行驶时间可取"
        "[travel, travel_max] 内任意实数），计算在指定发车范围（<= 3600 秒）"
        "内、对所有可能的行驶时间抵达每个路口均遇绿灯的全部极大半开"
        "发车区间。\n\n"
        "另提供单路口偏移诊断接口 `POST /api/green-wave/offset-diagnostics`："
        "给定一个路口的周期、绿灯相位及 1..2000 条（到达时刻, 实际灯色）"
        "观测，对整数 offset∈[0,cycle) 按 (arrival_time+offset) mod cycle "
        "预测灯色，返回最少错配条数与取到该值的全部极大半开 offset 区间"
        "（周期区间事件扫描，不逐一计算 offset×观测）。\n\n"
        "另有最大覆盖接口 `POST /api/green-wave/max-coverage`：对范围内每个"
        "出发时刻统计（任意行驶时长抵达都）遇绿的路口数，返回全局最大路口数"
        "及取到该值的全部极大半开发车区间（复用单路口可行窗口并按区间端点"
        "扫描覆盖数，不逐秒统计）。\n\n"
        "整段时隙接口 `POST /api/green-wave/slot-offsets`：复用路口参数与"
        "发车范围，另传 adjustable_index 指定唯一可调路口，求使范围内每个"
        "整数秒都能发车（其余路口保持原 offset、各路口均须对 travel 到 "
        "travel_max 的任意实际行驶时间遇绿）的全部可行整数 offset（按极大"
        "半开区间输出，跨零点分段），并给出距原 offset 环形最近的建议值"
        "（等距取较小者，无可行值为 null）。\n\n"
        "车队接口 `POST /api/green-wave/convoy-starts`：复用路口参数与发车"
        "范围，另传 vehicle_count（2..50）与 headway（正整数秒，要求 "
        "(vehicle_count-1)×headway 小于范围长度）。车队按固定间隔发车，"
        "每辆车都在给定范围内且在每个路口对任意实际行驶时间遇绿；基于单车"
        "可行窗口平移、求交并合并相邻区间（不只检查首末车），返回全部极大"
        "半开首车发车区间，无解时返回空数组。\n\n"
        "计划切换接口 `POST /api/green-wave/plan-switch-windows`：沿用发车"
        "范围、路口旧计划与行驶时间参数，另传全局整数秒 switch_at 与每个"
        "路口的 next_plan（cycle/offset/green_start/green_end，周期可与"
        "旧计划不同）。到达时刻小于 switch_at 用旧计划、等于或大于用新计"
        "划；切换点落在抵达波动区间内时两侧分别按各自计划检查（不只按最"
        "早或最晚抵达选定计划）。返回范围内全部极大半开发车窗口，无解时"
        "返回空数组。"
    ),
    version="1.5.0",
)


def _intersection_payload(item) -> dict:
    """把请求中的路口模型转换为算法层使用的参数字典。"""
    return {
        "cycle": item.cycle,
        "offset": item.offset,
        "green_start": item.green_start,
        "green_end": item.green_end,
        "travel": item.travel,
        "travel_max": item.effective_travel_max,
    }


def _plan_switch_payload(item) -> dict:
    """把计划切换请求中的路口模型转换为算法层使用的参数字典。"""
    payload = _intersection_payload(item)
    payload["next_plan"] = {
        "cycle": item.next_plan.cycle,
        "offset": item.next_plan.offset,
        "green_start": item.next_plan.green_start,
        "green_end": item.next_plan.green_end,
    }
    return payload


@app.get("/health", tags=["系统"], summary="健康检查")
def health() -> dict[str, str]:
    return {"status": "ok"}


@app.post(
    "/api/green-wave/windows",
    response_model=list[Window],
    tags=["绿波"],
    summary="计算全部极大绿波发车窗口",
)
def green_wave_windows(request: GreenWaveRequest) -> list[Window]:
    request.validate_ranges()
    windows = compute_windows(
        intersections=[
            _intersection_payload(item) for item in request.intersections
        ],
        depart_from=request.depart_from,
        depart_to=request.depart_to,
    )
    return [Window(start=start, end=end) for start, end in windows]


@app.post(
    "/api/green-wave/max-coverage",
    response_model=MaxCoverageResponse,
    tags=["绿波"],
    summary="计算最大遇绿路口数及取到该值的全部极大发车区间",
)
def green_wave_max_coverage(request: GreenWaveRequest) -> MaxCoverageResponse:
    request.validate_ranges()
    maximum, intervals = compute_max_coverage(
        intersections=[
            _intersection_payload(item) for item in request.intersections
        ],
        depart_from=request.depart_from,
        depart_to=request.depart_to,
    )
    return MaxCoverageResponse(
        max_coverage=maximum,
        optimal_intervals=[
            Window(start=start, end=end) for start, end in intervals
        ],
    )


@app.post(
    "/api/green-wave/offset-diagnostics",
    response_model=OffsetDiagnosticsResponse,
    tags=["绿波"],
    summary="单路口 offset 诊断：最少错配条数与最优 offset 区间",
)
def green_wave_offset_diagnostics(
    request: OffsetDiagnosticsRequest,
) -> OffsetDiagnosticsResponse:
    request.validate_ranges()
    minimum, intervals = offset_diagnostics(
        cycle=request.cycle,
        green_start=request.green_start,
        green_end=request.green_end,
        observations=(
            (item.arrival_time, item.observed_green)
            for item in request.observations
        ),
    )
    return OffsetDiagnosticsResponse(
        min_mismatches=minimum,
        optimal_intervals=[
            OffsetInterval(start=start, end=end) for start, end in intervals
        ],
    )


@app.post(
    "/api/green-wave/slot-offsets",
    response_model=SlotOffsetsResponse,
    tags=["绿波"],
    summary="整段时隙：求唯一可调路口的全部可行 offset 与最近建议值",
)
def green_wave_slot_offsets(
    request: SlotOffsetsRequest,
) -> SlotOffsetsResponse:
    request.validate_ranges()
    intervals, recommended = compute_slot_offsets(
        intersections=[
            _intersection_payload(item) for item in request.intersections
        ],
        depart_from=request.depart_from,
        depart_to=request.depart_to,
        adjustable_index=request.adjustable_index,
    )
    return SlotOffsetsResponse(
        adjustable_index=request.adjustable_index,
        feasible_intervals=[
            OffsetInterval(start=start, end=end) for start, end in intervals
        ],
        recommended_offset=recommended,
    )


@app.post(
    "/api/green-wave/convoy-starts",
    response_model=list[Window],
    tags=["绿波"],
    summary="计算车队首车发车的全部极大半开区间",
)
def green_wave_convoy_starts(request: ConvoyStartsRequest) -> list[Window]:
    request.validate_ranges()
    starts = compute_convoy_starts(
        intersections=[
            _intersection_payload(item) for item in request.intersections
        ],
        depart_from=request.depart_from,
        depart_to=request.depart_to,
        vehicle_count=request.vehicle_count,
        headway=request.headway,
    )
    return [Window(start=start, end=end) for start, end in starts]


@app.post(
    "/api/green-wave/plan-switch-windows",
    response_model=list[Window],
    tags=["绿波"],
    summary="计划切换下计算全部极大绿波发车窗口",
)
def green_wave_plan_switch_windows(request: PlanSwitchRequest) -> list[Window]:
    request.validate_ranges()
    windows = compute_plan_switch_windows(
        intersections=[
            _plan_switch_payload(item) for item in request.intersections
        ],
        depart_from=request.depart_from,
        depart_to=request.depart_to,
        switch_at=request.switch_at,
    )
    return [Window(start=start, end=end) for start, end in windows]
