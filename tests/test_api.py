import pytest
from fastapi import HTTPException
from pydantic import ValidationError

from app.main import (
    green_wave_convoy_starts,
    green_wave_max_coverage,
    green_wave_offset_diagnostics,
    green_wave_plan_switch_windows,
    green_wave_slot_offsets,
    green_wave_windows,
)
from app.schemas import (
    ConvoyStartsRequest,
    GreenWaveRequest,
    Intersection,
    OffsetDiagnosticsRequest,
    PlanSwitchRequest,
    SlotOffsetsRequest,
)


def make_request(payload):
    return GreenWaveRequest.model_validate(payload)


def test_api_returns_windows_payload():
    req = make_request(
        {
            "depart_from": 0,
            "depart_to": 300,
            "intersections": [
                {"cycle": 60, "offset": 0, "green_start": 0, "green_end": 30, "travel": 0},
                {"cycle": 60, "offset": 30, "green_start": 0, "green_end": 30, "travel": 30},
            ],
        }
    )
    windows = green_wave_windows(req)
    assert [[w.start, w.end] for w in windows] == [
        [0, 30], [60, 90], [120, 150], [180, 210], [240, 270],
    ]


def test_api_empty_array_on_no_solution():
    req = make_request(
        {
            "depart_from": 0,
            "depart_to": 60,
            "intersections": [
                {"cycle": 60, "offset": 0, "green_start": 0, "green_end": 10, "travel": 0},
                {"cycle": 60, "offset": 0, "green_start": 20, "green_end": 30, "travel": 0},
            ],
        }
    )
    assert green_wave_windows(req) == []


def test_reject_invalid_cycle():
    with pytest.raises(ValidationError):
        Intersection(cycle=0, offset=0, green_start=0, green_end=10, travel=0)


def test_reject_negative_travel():
    with pytest.raises(ValidationError):
        Intersection(cycle=60, offset=0, green_start=0, green_end=10, travel=-1)


def test_reject_offset_outside_cycle():
    req = make_request(
        {
            "depart_from": 0,
            "depart_to": 60,
            "intersections": [
                {"cycle": 60, "offset": 60, "green_start": 0, "green_end": 10, "travel": 0}
            ],
        }
    )
    with pytest.raises(HTTPException) as exc:
        req.validate_ranges()
    assert exc.value.status_code == 422


def test_reject_green_endpoint_outside_cycle():
    req = make_request(
        {
            "depart_from": 0,
            "depart_to": 60,
            "intersections": [
                {"cycle": 60, "offset": 0, "green_start": 0, "green_end": 60, "travel": 0}
            ],
        }
    )
    with pytest.raises(HTTPException) as exc:
        req.validate_ranges()
    assert exc.value.status_code == 422


def test_reject_range_over_one_hour():
    req = make_request(
        {
            "depart_from": 0,
            "depart_to": 3601,
            "intersections": [
                {"cycle": 60, "offset": 0, "green_start": 0, "green_end": 10, "travel": 0}
            ],
        }
    )
    with pytest.raises(HTTPException) as exc:
        req.validate_ranges()
    assert exc.value.status_code == 422


def test_reject_too_many_intersections():
    with pytest.raises(ValidationError):
        make_request(
            {
                "depart_from": 0,
                "depart_to": 60,
                "intersections": [
                    {"cycle": 60, "offset": 0, "green_start": 0, "green_end": 10}
                    for _ in range(9)
                ],
            }
        )


def test_reject_empty_intersections():
    with pytest.raises(ValidationError):
        make_request({"depart_from": 0, "depart_to": 60, "intersections": []})


# ---------- travel_max 波动行驶时间 ----------

def test_travel_max_shrinks_windows():
    # 绿灯 [20,40)，行驶时间 [0,10]：窗口由 20 秒收缩为 10 秒
    req = make_request(
        {
            "depart_from": 0,
            "depart_to": 120,
            "intersections": [
                {"cycle": 60, "offset": 0, "green_start": 20,
                 "green_end": 40, "travel": 0, "travel_max": 10}
            ],
        }
    )
    windows = green_wave_windows(req)
    assert [[w.start, w.end] for w in windows] == [[20, 30], [80, 90]]


def test_travel_max_defaults_to_travel():
    base = {
        "cycle": 60, "offset": 0, "green_start": 20, "green_end": 40,
        "travel": 5,
    }
    req = make_request(
        {"depart_from": 0, "depart_to": 120, "intersections": [base]}
    )
    req2 = make_request(
        {"depart_from": 0, "depart_to": 120,
         "intersections": [{**base, "travel_max": 5}]}
    )
    assert green_wave_windows(req) == green_wave_windows(req2)


def test_reject_travel_max_less_than_travel():
    req = make_request(
        {
            "depart_from": 0,
            "depart_to": 60,
            "intersections": [
                {"cycle": 60, "offset": 0, "green_start": 0,
                 "green_end": 30, "travel": 20, "travel_max": 19}
            ],
        }
    )
    with pytest.raises(HTTPException) as exc:
        req.validate_ranges()
    assert exc.value.status_code == 422


def test_reject_travel_max_over_86400():
    req = make_request(
        {
            "depart_from": 0,
            "depart_to": 60,
            "intersections": [
                {"cycle": 60, "offset": 0, "green_start": 0,
                 "green_end": 30, "travel": 0, "travel_max": 86401}
            ],
        }
    )
    with pytest.raises(HTTPException) as exc:
        req.validate_ranges()
    assert exc.value.status_code == 422


def test_travel_max_86400_boundary_accepted():
    req = make_request(
        {
            "depart_from": 0,
            "depart_to": 60,
            "intersections": [
                # 整周期常绿：波动再大也不受影响
                {"cycle": 60, "offset": 0, "green_start": 0,
                 "green_end": 0, "travel": 0, "travel_max": 86400}
            ],
        }
    )
    req.validate_ranges()
    windows = green_wave_windows(req)
    assert [[w.start, w.end] for w in windows] == [[0, 60]]


# ---------- POST /api/green-wave/max-coverage ----------

def test_max_coverage_payload_overlap():
    req = make_request(
        {
            "depart_from": 0,
            "depart_to": 120,
            "intersections": [
                {"cycle": 60, "offset": 0, "green_start": 0, "green_end": 40, "travel": 0},
                {"cycle": 60, "offset": 0, "green_start": 20, "green_end": 50, "travel": 0},
            ],
        }
    )
    resp = green_wave_max_coverage(req)
    assert resp.max_coverage == 2
    assert [[w.start, w.end] for w in resp.optimal_intervals] == [
        [20, 40], [80, 100],
    ]


def test_max_coverage_zero_returns_complete_range():
    req = make_request(
        {
            "depart_from": 10,
            "depart_to": 70,
            "intersections": [
                {"cycle": 60, "offset": 0, "green_start": 20, "green_end": 40,
                 "travel": 0, "travel_max": 30}
            ],
        }
    )
    resp = green_wave_max_coverage(req)
    assert resp.max_coverage == 0
    assert [[w.start, w.end] for w in resp.optimal_intervals] == [[10, 70]]


def test_max_coverage_reuses_request_validation():
    # 与 /windows 相同的跨字段校验：发车范围超过 3600 秒仍返回 422。
    req = make_request(
        {
            "depart_from": 0,
            "depart_to": 3601,
            "intersections": [
                {"cycle": 60, "offset": 0, "green_start": 0, "green_end": 10, "travel": 0}
            ],
        }
    )
    with pytest.raises(HTTPException) as exc:
        green_wave_max_coverage(req)
    assert exc.value.status_code == 422


def test_max_coverage_route_is_in_openapi():
    from app.main import app

    paths = app.openapi()["paths"]
    assert "/api/green-wave/max-coverage" in paths
    assert "post" in paths["/api/green-wave/max-coverage"]


# ---------- POST /api/green-wave/offset-diagnostics ----------

def make_diag_request(payload):
    return OffsetDiagnosticsRequest.model_validate(payload)


def test_offset_diagnostics_payload_plain_green():
    req = make_diag_request(
        {
            "cycle": 60,
            "green_start": 20,
            "green_end": 40,
            "observations": [
                {"arrival_time": 0, "observed_green": True},
                {"arrival_time": 5, "observed_green": True},
            ],
        }
    )
    resp = green_wave_offset_diagnostics(req)
    # arrival=0 观测绿 ⇔ offset∈[20,40)；arrival=5 观测绿 ⇔ offset∈[15,35)
    # 零错配集合为两者之交 [20,35)
    assert resp.min_mismatches == 0
    assert [[w.start, w.end] for w in resp.optimal_intervals] == [[20, 35]]


def test_offset_diagnostics_payload_wrapping_optimal_set():
    req = make_diag_request(
        {
            "cycle": 60,
            "green_start": 50,
            "green_end": 10,
            "observations": [{"arrival_time": 0, "observed_green": True}],
        }
    )
    resp = green_wave_offset_diagnostics(req)
    assert resp.min_mismatches == 0
    assert [[w.start, w.end] for w in resp.optimal_intervals] == [
        [0, 10], [50, 60]
    ]


def test_offset_diagnostics_min_mismatches_positive():
    # 常绿路口但观测到红灯：任何 offset 都错 1 条
    req = make_diag_request(
        {
            "cycle": 60,
            "green_start": 30,
            "green_end": 30,
            "observations": [{"arrival_time": 9, "observed_green": False}],
        }
    )
    resp = green_wave_offset_diagnostics(req)
    assert resp.min_mismatches == 1
    assert [[w.start, w.end] for w in resp.optimal_intervals] == [[0, 60]]


def test_offset_diagnostics_accepts_negative_arrival_time():
    req = make_diag_request(
        {
            "cycle": 60,
            "green_start": 20,
            "green_end": 40,
            "observations": [{"arrival_time": -1, "observed_green": True}],
        }
    )
    resp = green_wave_offset_diagnostics(req)
    assert resp.min_mismatches == 0
    assert [[w.start, w.end] for w in resp.optimal_intervals] == [[21, 41]]


def test_offset_diagnostics_rejects_zero_cycle():
    with pytest.raises(ValidationError):
        make_diag_request(
            {
                "cycle": 0,
                "green_start": 0,
                "green_end": 10,
                "observations": [{"arrival_time": 0, "observed_green": True}],
            }
        )


def test_offset_diagnostics_rejects_cycle_over_86400():
    with pytest.raises(ValidationError):
        make_diag_request(
            {
                "cycle": 86401,
                "green_start": 0,
                "green_end": 10,
                "observations": [{"arrival_time": 0, "observed_green": True}],
            }
        )


def test_offset_diagnostics_rejects_green_start_outside_cycle():
    req = make_diag_request(
        {
            "cycle": 60,
            "green_start": 60,
            "green_end": 10,
            "observations": [{"arrival_time": 0, "observed_green": True}],
        }
    )
    with pytest.raises(HTTPException) as exc:
        req.validate_ranges()
    assert exc.value.status_code == 422


def test_offset_diagnostics_rejects_negative_green_end():
    with pytest.raises(ValidationError):
        make_diag_request(
            {
                "cycle": 60,
                "green_start": 0,
                "green_end": -1,
                "observations": [{"arrival_time": 0, "observed_green": True}],
            }
        )


def test_offset_diagnostics_rejects_empty_observations():
    with pytest.raises(ValidationError):
        make_diag_request(
            {"cycle": 60, "green_start": 0, "green_end": 10, "observations": []}
        )


def test_offset_diagnostics_rejects_too_many_observations():
    with pytest.raises(ValidationError):
        make_diag_request(
            {
                "cycle": 60,
                "green_start": 0,
                "green_end": 10,
                "observations": [
                    {"arrival_time": i, "observed_green": True}
                    for i in range(2001)
                ],
            }
        )


def test_offset_diagnostics_accepts_2000_observations_boundary():
    req = make_diag_request(
        {
            "cycle": 60,
            "green_start": 0,
            "green_end": 0,  # 常绿
            "observations": [
                {"arrival_time": i, "observed_green": True}
                for i in range(2000)
            ],
        }
    )
    req.validate_ranges()
    resp = green_wave_offset_diagnostics(req)
    assert resp.min_mismatches == 0
    assert [[w.start, w.end] for w in resp.optimal_intervals] == [[0, 60]]


def test_offset_diagnostics_route_is_in_openapi():
    from app.main import app

    paths = app.openapi()["paths"]
    assert "/api/green-wave/offset-diagnostics" in paths
    assert "post" in paths["/api/green-wave/offset-diagnostics"]


# ---------- POST /api/green-wave/slot-offsets ----------

def make_slot_request(payload):
    return SlotOffsetsRequest.model_validate(payload)


def test_slot_offsets_plain_payload():
    req = make_slot_request(
        {
            "depart_from": 0,
            "depart_to": 10,
            "adjustable_index": 0,
            "intersections": [
                {"cycle": 60, "offset": 0, "green_start": 0,
                 "green_end": 30, "travel": 0}
            ],
        }
    )
    resp = green_wave_slot_offsets(req)
    assert resp.adjustable_index == 0
    assert [[w.start, w.end] for w in resp.feasible_intervals] == [[0, 21]]
    assert resp.recommended_offset == 0


def test_slot_offsets_wrapping_payload_and_recommendation():
    req = make_slot_request(
        {
            "depart_from": 0,
            "depart_to": 5,
            "adjustable_index": 0,
            "intersections": [
                {"cycle": 60, "offset": 30, "green_start": 50,
                 "green_end": 10, "travel": 0}
            ],
        }
    )
    resp = green_wave_slot_offsets(req)
    assert [[w.start, w.end] for w in resp.feasible_intervals] == [
        [0, 6], [50, 60]
    ]
    assert resp.recommended_offset == 50


def test_slot_offsets_empty_when_fixed_intersection_cannot_cover_range():
    req = make_slot_request(
        {
            "depart_from": 0,
            "depart_to": 30,
            "adjustable_index": 0,
            "intersections": [
                {"cycle": 60, "offset": 0, "green_start": 0,
                 "green_end": 0, "travel": 0},
                {"cycle": 60, "offset": 0, "green_start": 0,
                 "green_end": 10, "travel": 0},
            ],
        }
    )
    resp = green_wave_slot_offsets(req)
    assert resp.feasible_intervals == []
    assert resp.recommended_offset is None


def test_slot_offsets_slack_is_honored():
    # 绿灯 [20,50)、slack=10 收缩为 [20,40)：范围 15 秒只剩 6 个可行 offset
    req = make_slot_request(
        {
            "depart_from": 0,
            "depart_to": 15,
            "adjustable_index": 0,
            "intersections": [
                {"cycle": 60, "offset": 23, "green_start": 20,
                 "green_end": 50, "travel": 0, "travel_max": 10}
            ],
        }
    )
    resp = green_wave_slot_offsets(req)
    assert [[w.start, w.end] for w in resp.feasible_intervals] == [[20, 26]]
    assert resp.recommended_offset == 23


def test_slot_offsets_rejects_index_out_of_range():
    req = make_slot_request(
        {
            "depart_from": 0,
            "depart_to": 60,
            "adjustable_index": 1,
            "intersections": [
                {"cycle": 60, "offset": 0, "green_start": 0,
                 "green_end": 30, "travel": 0}
            ],
        }
    )
    with pytest.raises(HTTPException) as exc:
        green_wave_slot_offsets(req)
    assert exc.value.status_code == 422


def test_slot_offsets_rejects_negative_index():
    with pytest.raises(ValidationError):
        make_slot_request(
            {
                "depart_from": 0,
                "depart_to": 60,
                "adjustable_index": -1,
                "intersections": [
                    {"cycle": 60, "offset": 0, "green_start": 0,
                     "green_end": 30, "travel": 0}
                ],
            }
        )


def test_slot_offsets_reuses_green_wave_validation():
    # 与 /windows 相同的跨字段校验：路口 offset 越界与范围过长均 422。
    req = make_slot_request(
        {
            "depart_from": 0,
            "depart_to": 60,
            "adjustable_index": 0,
            "intersections": [
                {"cycle": 60, "offset": 60, "green_start": 0,
                 "green_end": 10, "travel": 0}
            ],
        }
    )
    with pytest.raises(HTTPException) as exc:
        green_wave_slot_offsets(req)
    assert exc.value.status_code == 422

    req2 = make_slot_request(
        {
            "depart_from": 0,
            "depart_to": 3601,
            "adjustable_index": 0,
            "intersections": [
                {"cycle": 60, "offset": 0, "green_start": 0,
                 "green_end": 10, "travel": 0}
            ],
        }
    )
    with pytest.raises(HTTPException) as exc:
        green_wave_slot_offsets(req2)
    assert exc.value.status_code == 422


def test_slot_offsets_route_is_in_openapi():
    from app.main import app

    paths = app.openapi()["paths"]
    assert "/api/green-wave/slot-offsets" in paths
    assert "post" in paths["/api/green-wave/slot-offsets"]


# ---------- POST /api/green-wave/convoy-starts ----------

def make_convoy_request(payload):
    return ConvoyStartsRequest.model_validate(payload)


def test_convoy_starts_payload():
    # 两路口绿波带，2 辆车间隔 10 秒：首车窗口由 30 秒收缩为 20 秒
    req = make_convoy_request(
        {
            "depart_from": 0,
            "depart_to": 300,
            "vehicle_count": 2,
            "headway": 10,
            "intersections": [
                {"cycle": 60, "offset": 0, "green_start": 0, "green_end": 30, "travel": 0},
                {"cycle": 60, "offset": 30, "green_start": 0, "green_end": 30, "travel": 30},
            ],
        }
    )
    starts = green_wave_convoy_starts(req)
    assert [[w.start, w.end] for w in starts] == [
        [0, 20], [60, 80], [120, 140], [180, 200], [240, 260],
    ]


def test_convoy_starts_empty_array_on_no_solution():
    # 3 辆车间隔 40 秒：首末车检查会误判可行，中间车必卡红灯，返回空数组
    req = make_convoy_request(
        {
            "depart_from": 0,
            "depart_to": 120,
            "vehicle_count": 3,
            "headway": 40,
            "intersections": [
                {"cycle": 60, "offset": 0, "green_start": 0, "green_end": 30, "travel": 0}
            ],
        }
    )
    assert green_wave_convoy_starts(req) == []


def test_convoy_rejects_vehicle_count_out_of_range():
    for count in (1, 51):
        with pytest.raises(ValidationError):
            make_convoy_request(
                {
                    "depart_from": 0,
                    "depart_to": 300,
                    "vehicle_count": count,
                    "headway": 5,
                    "intersections": [
                        {"cycle": 60, "offset": 0, "green_start": 0,
                         "green_end": 30, "travel": 0}
                    ],
                }
            )


def test_convoy_rejects_non_positive_headway():
    with pytest.raises(ValidationError):
        make_convoy_request(
            {
                "depart_from": 0,
                "depart_to": 300,
                "vehicle_count": 2,
                "headway": 0,
                "intersections": [
                    {"cycle": 60, "offset": 0, "green_start": 0,
                     "green_end": 30, "travel": 0}
                ],
            }
        )


def test_convoy_rejects_span_not_less_than_range():
    # (3-1)×10 = 20 不小于范围长度 20：首车不存在可行时刻，422
    req = make_convoy_request(
        {
            "depart_from": 0,
            "depart_to": 20,
            "vehicle_count": 3,
            "headway": 10,
            "intersections": [
                {"cycle": 60, "offset": 0, "green_start": 0, "green_end": 30, "travel": 0}
            ],
        }
    )
    with pytest.raises(HTTPException) as exc:
        green_wave_convoy_starts(req)
    assert exc.value.status_code == 422


def test_convoy_span_one_less_than_range_accepted():
    # (3-1)×9 = 18 < 20：常绿路口下首车范围为 [0, 2)
    req = make_convoy_request(
        {
            "depart_from": 0,
            "depart_to": 20,
            "vehicle_count": 3,
            "headway": 9,
            "intersections": [
                {"cycle": 60, "offset": 0, "green_start": 0, "green_end": 0, "travel": 0}
            ],
        }
    )
    starts = green_wave_convoy_starts(req)
    assert [[w.start, w.end] for w in starts] == [[0, 2]]


def test_convoy_reuses_green_wave_validation():
    # 与 /windows 相同的跨字段校验：发车范围超过 3600 秒仍返回 422。
    req = make_convoy_request(
        {
            "depart_from": 0,
            "depart_to": 3601,
            "vehicle_count": 2,
            "headway": 5,
            "intersections": [
                {"cycle": 60, "offset": 0, "green_start": 0, "green_end": 10, "travel": 0}
            ],
        }
    )
    with pytest.raises(HTTPException) as exc:
        green_wave_convoy_starts(req)
    assert exc.value.status_code == 422


def test_convoy_route_is_in_openapi():
    from app.main import app

    paths = app.openapi()["paths"]
    assert "/api/green-wave/convoy-starts" in paths
    assert "post" in paths["/api/green-wave/convoy-starts"]


# ---------- POST /api/green-wave/plan-switch-windows ----------

def make_plan_switch_request(payload):
    return PlanSwitchRequest.model_validate(payload)


def plan_switch_payload(**overrides):
    payload = {
        "depart_from": 0,
        "depart_to": 120,
        "switch_at": 30,
        "intersections": [
            {
                "cycle": 60, "offset": 0, "green_start": 0, "green_end": 30,
                "travel": 0, "travel_max": 10,
                "next_plan": {
                    "cycle": 60, "offset": 0,
                    "green_start": 30, "green_end": 50,
                },
            }
        ],
    }
    payload.update(overrides)
    return payload


def test_plan_switch_windows_payload_merge_across_switch():
    # 旧计划绿灯 [0,30)、新计划 [30,50)，switch_at=30，行驶时间 [0,10]：
    # 有效绿灯在切换点相接合并为连续分量 [0,50)，收缩 10 秒波动后为 [0,40)
    req = make_plan_switch_request(plan_switch_payload())
    windows = green_wave_plan_switch_windows(req)
    assert [[w.start, w.end] for w in windows] == [[0, 40], [90, 100]]


def test_plan_switch_pure_old_plan_when_switch_beyond_range():
    req = make_plan_switch_request(
        plan_switch_payload(
            switch_at=100000,
            intersections=[
                {
                    "cycle": 60, "offset": 0, "green_start": 0, "green_end": 30,
                    "travel": 0,
                    "next_plan": {
                        "cycle": 60, "offset": 0,
                        "green_start": 10, "green_end": 40,
                    },
                }
            ],
        )
    )
    windows = green_wave_plan_switch_windows(req)
    assert [[w.start, w.end] for w in windows] == [[0, 30], [60, 90]]


def test_plan_switch_empty_array_on_no_solution():
    req = make_plan_switch_request(
        plan_switch_payload(
            depart_from=40,
            depart_to=60,
            switch_at=50,
            intersections=[
                {
                    "cycle": 60, "offset": 0, "green_start": 0, "green_end": 10,
                    "travel": 0,
                    "next_plan": {
                        "cycle": 60, "offset": 0,
                        "green_start": 0, "green_end": 10,
                    },
                }
            ],
        )
    )
    assert green_wave_plan_switch_windows(req) == []


def test_plan_switch_rejects_invalid_next_plan():
    # next_plan 的 offset / green_start / green_end 以其自身 cycle 为上界
    for bad_next_plan in (
        {"cycle": 60, "offset": 60, "green_start": 0, "green_end": 30},
        {"cycle": 60, "offset": 0, "green_start": 60, "green_end": 30},
        {"cycle": 60, "offset": 0, "green_start": 0, "green_end": 60},
    ):
        req = make_plan_switch_request(
            plan_switch_payload(
                intersections=[
                    {
                        "cycle": 60, "offset": 0, "green_start": 0,
                        "green_end": 30, "travel": 0,
                        "next_plan": bad_next_plan,
                    }
                ],
            )
        )
        with pytest.raises(HTTPException) as exc:
            green_wave_plan_switch_windows(req)
        assert exc.value.status_code == 422


def test_plan_switch_requires_next_plan():
    with pytest.raises(ValidationError):
        make_plan_switch_request(
            plan_switch_payload(
                intersections=[
                    {"cycle": 60, "offset": 0, "green_start": 0,
                     "green_end": 30, "travel": 0}
                ],
            )
        )


def test_plan_switch_rejects_invalid_next_plan_cycle():
    with pytest.raises(ValidationError):
        make_plan_switch_request(
            plan_switch_payload(
                intersections=[
                    {
                        "cycle": 60, "offset": 0, "green_start": 0,
                        "green_end": 30, "travel": 0,
                        "next_plan": {"cycle": 0, "offset": 0,
                                      "green_start": 0, "green_end": 30},
                    }
                ],
            )
        )


def test_plan_switch_rejects_negative_switch_at():
    with pytest.raises(ValidationError):
        make_plan_switch_request(plan_switch_payload(switch_at=-1))


def test_plan_switch_reuses_green_wave_validation():
    # 与 /windows 相同的跨字段校验：发车范围超过 3600 秒、旧计划 offset
    # 越界均返回 422。
    req = make_plan_switch_request(plan_switch_payload(depart_to=3601))
    with pytest.raises(HTTPException) as exc:
        green_wave_plan_switch_windows(req)
    assert exc.value.status_code == 422

    req2 = make_plan_switch_request(
        plan_switch_payload(
            intersections=[
                {
                    "cycle": 60, "offset": 60, "green_start": 0,
                    "green_end": 30, "travel": 0,
                    "next_plan": {"cycle": 60, "offset": 0,
                                  "green_start": 0, "green_end": 30},
                }
            ],
        )
    )
    with pytest.raises(HTTPException) as exc:
        green_wave_plan_switch_windows(req2)
    assert exc.value.status_code == 422


def test_plan_switch_route_is_in_openapi():
    from app.main import app

    paths = app.openapi()["paths"]
    assert "/api/green-wave/plan-switch-windows" in paths
    assert "post" in paths["/api/green-wave/plan-switch-windows"]
