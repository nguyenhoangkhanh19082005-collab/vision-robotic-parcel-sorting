import numpy as np
import pytest


def test_conveyor_bounds_match_measured_dimensions(station_config):
    from sorting_station.conveyor_model import ConveyorSpec, build_merged

    spec = ConveyorSpec.from_config(station_config["layout"]["conveyor"]["model"])
    xmin, xmax, ymin, ymax, zmin, zmax = build_merged(spec, scale=1.0).bounds
    assert ymax - ymin == pytest.approx(spec.length_total, abs=0.5)
    assert zmin == pytest.approx(0.0, abs=1e-6)  # chân chạm mặt đặt
    assert zmax == pytest.approx(spec.belt_top_height, abs=0.5)  # mặt băng là điểm cao nhất
    assert xmax - xmin >= spec.belt_width


def test_layout_derives_from_conveyor_model(station_config):
    from sorting_station.cell_state import CellLayout

    layout = CellLayout.from_config(station_config["layout"])
    spec = layout.conveyor_spec
    assert layout.conveyor_end_y - layout.conveyor_start_y == pytest.approx(spec.length_total / 1000.0)
    assert layout.conveyor_width == pytest.approx(spec.belt_width / 1000.0)
    assert layout.conveyor_start_y < layout.camera_y < layout.pick_y < layout.conveyor_end_y
    # Kiện nằm đúng trên mặt băng
    parcel = layout.parcel_on_belt(layout.pick_y)
    assert parcel[2] - layout.parcel_size[2] / 2 == pytest.approx(layout.belt_top_z)


def test_conveyor_clear_of_robot_base(station_config, model):
    from sorting_station.cell_state import CellLayout

    layout = CellLayout.from_config(station_config["layout"])
    inner_edge = layout.conveyor_center_x - layout.conveyor_spec.outer_width / 2000.0
    assert inner_edge > 0.135  # bán kính đế robot đo trên mesh ≈ 131 mm


def test_pick_point_on_belt(station_config):
    from sorting_station.cell_state import CellLayout

    layout = CellLayout.from_config(station_config["layout"])
    pick = np.asarray(station_config["sorting"]["pick_xyz_mm"]) / 1000.0
    assert abs(pick[0] - layout.conveyor_center_x) < layout.conveyor_width / 2
    assert layout.conveyor_start_y < pick[1] < layout.conveyor_end_y
    assert pick[2] > layout.belt_top_z


def test_invalid_model_key_rejected():
    from sorting_station.conveyor_model import ConveyorSpec

    with pytest.raises(ValueError):
        ConveyorSpec.from_config({"motor_size": 30})
