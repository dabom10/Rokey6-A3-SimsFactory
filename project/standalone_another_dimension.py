#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import traceback

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import carb
import omni.timeline
from isaacsim.core.api import World
from isaacsim.core.utils.stage import open_stage


ENV_USD_PATH = "/home/kyb/Rokey6-A3-SimsFactory/project/environment.usd"


def _assert_usd(path: str) -> None:
    if not os.path.isfile(path):
        raise FileNotFoundError(f"USD 파일이 없습니다: {path}")


def main():
    _assert_usd(ENV_USD_PATH)

    carb.log_info(f"[STAGE] open_stage: {ENV_USD_PATH}")
    open_stage(ENV_USD_PATH)

    world = World(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0)
    world.reset()

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()

    while simulation_app.is_running():
        world.step(render=True)


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        carb.log_error(f"[FATAL] {type(e).__name__}: {e}")
        carb.log_error(traceback.format_exc())
    finally:
        simulation_app.close()