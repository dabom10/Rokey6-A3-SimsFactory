import os
from ament_index_python.packages import get_package_share_directory

# 1. Isaac Sim App 시작
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

# 2. ROS2 Bridge 익스텐션 활성화
from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.ros2_bridge")

import omni.graph.core as og
from omni.isaac.core import SimulationContext
from omni.isaac.core.utils.stage import open_stage, is_stage_loading

def setup_global_ros2_clock():
    """ROS2 use_sim_time을 위한 전역 Clock 퍼블리셔 생성"""
    # robot.usd 안에 이미 cmd_vel, odom, scan 등의 Action Graph가 내장되어 있으므로,
    # Python 스크립트에서는 Nav2 시간 동기화를 위한 /clock 노드만 추가합니다.
    graph_path = "/ActionGraph/ROS2_Clock"
    keys = og.Controller.Keys
    
    try:
        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
                ],
                keys.CONNECT: [
                    # 시간 동기화 (RViz2 및 Nav2의 use_sim_time: True 에러 방지)
                    ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                    ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ],
            },
        )
        print("[INFO] ROS2 Clock OmniGraph setup successful!")
    except Exception as e:
        print(f"[ERROR] Failed to setup Clock OmniGraph: {e}")

def main():
    # 3. USD 씬 로드 (파일 구조에 맞게 파일명 수정)
    pkg_dir = get_package_share_directory('environment')
    # 기존 ur10_factory.usd 대신 현재 폴더 구조에 있는 environment.usd를 로드합니다.
    usd_path = os.path.join(pkg_dir, 'usd', 'environment.usd')
    
    print(f"[INFO] Loading USD from: {usd_path}")
    open_stage(usd_path)
    
    while is_stage_loading():
        simulation_app.update()
        
    print("[INFO] Stage loaded successfully.")

    # 4. Simulation Context 초기화 및 시뮬레이션 Play
    sim_context = SimulationContext()
    sim_context.initialize_physics()
    
    # Clock 그래프 생성
    setup_global_ros2_clock()
    
    # 시뮬레이션 시작 (아이작 심의 Play 버튼을 누르는 것과 동일한 효과)
    sim_context.play()

    # 5. 메인 시뮬레이션 루프
    print("[INFO] Starting simulation loop... Waiting for ROS2 Nav2 commands.")
    while simulation_app.is_running():
        sim_context.step(render=True)
        
    # 창을 닫으면 종료 및 정리
    sim_context.stop()
    simulation_app.close()

if __name__ == '__main__':
    main()