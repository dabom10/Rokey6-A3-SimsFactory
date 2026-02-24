# 1. 가장 먼저 SimulationApp을 임포트하고 실행해야 합니다! (절대 규칙)
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False}) # 화면을 보려면 headless: False

# ------------------------------------------------------------------------
# SimulationApp이 켜진 이후에 나머지 Isaac Sim 모듈들을 임포트합니다.
# ------------------------------------------------------------------------
import numpy as np
import carb
import omni.kit.commands

from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.robot.manipulators import SingleManipulator
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.rotations import euler_angles_to_quat
import isaacsim.robot_motion.motion_generation as mg

# RMPFlow 제어기 클래스
class RMPFlowController(mg.MotionPolicyController):
    def __init__(self, name: str, robot_articulation: SingleArticulation, physics_dt: float = 1.0 / 60.0):
        self.rmp_flow_config = mg.interface_config_loader.load_supported_motion_policy_config("UR10", "RMPflow")
        self.rmp_flow = mg.lula.motion_policies.RmpFlow(**self.rmp_flow_config)
        self.articulation_rmp = mg.ArticulationMotionPolicy(robot_articulation, self.rmp_flow, physics_dt)
        mg.MotionPolicyController.__init__(self, name=name, articulation_motion_policy=self.articulation_rmp)

def execute_motion(robot, cspace_controller, target_position, target_orientation):
    """ RMPFlow를 이용해 목표 지점으로 이동하고 도달 여부(True/False)를 반환 """
    action = cspace_controller.forward(
        target_end_effector_position=target_position,
        target_end_effector_orientation=target_orientation
    )
    robot.apply_action(action)
    
    current_joint_positions = robot.get_joint_positions()
    if np.all(np.abs(current_joint_positions[:6] - action.joint_positions[:6]) < 0.01):
        return True
    return False

def main():
    # 2. World 생성 (물리 엔진 및 시뮬레이션 환경 관리)
    world = World(physics_dt=1.0/60.0, rendering_dt=1.0/60.0)

    # 3. Environment & Robot USD 로드
    env_usd_path = "/home/rokey/Rokey6-A3-SimsFactory/project/environment.usd"
    robot_usd_path = "/home/rokey/Rokey6-A3-SimsFactory/project/robot.usd"
    
    add_reference_to_stage(usd_path=env_usd_path, prim_path="/World/Environment")
    add_reference_to_stage(usd_path=robot_usd_path, prim_path="/World/Robot")

    # 4. 로봇 셋업 (SingleManipulator)
    robot_position = np.array([-9.610604115900275, 6.006036154997052, -0.1408392725055303])
    
    # 실제 ee_link의 전체 USD 경로를 적어주세요.
    ee_full_path = "/Root/robot/run_robot/ur10/ee_link" 

    ur10_robot = world.scene.add(
        SingleManipulator(
            prim_path="/World/Robot/Root/robot/run_robot",
            name="robot",
            end_effector_prim_path=ee_full_path, # <--- 이름 대신 '전체 경로' 파라미터 사용
            position=robot_position
        )
    )

    # 5. Red Book 큐브 생성 (Rigid Body + Collider)
    cube_position = np.array([-11.894412279327886, 5.214124975327733, 1.3503861474629597])
    red_book = world.scene.add(
        DynamicCuboid(
            prim_path="/World/red_book",
            name="red_book",
            position=cube_position,
            scale=np.array([0.11400706644784753, 0.15779789435407796, 0.040555917109024356]),
            color=np.array([1.0, 0.0, 0.0]), # Red
            mass=0.5
        )
    )

    # 6. World 초기화 (이 과정에서 물리 엔진 초기화 및 객체들이 씬에 등록됨)
    world.reset()

    # 제어기 생성 (반드시 world.reset() 이후에 해야 관절 정보를 정상적으로 읽어옵니다)
    cspace_controller = RMPFlowController(name="ur10_controller", robot_articulation=ur10_robot)

    # State Machine 정의
    STATES = {
        "IDLE": 0, "MOVE_TO_PREGRASP": 1, "LOWER": 2, "ATTACH": 3, 
        "LIFT": 4, "MOVE_TO_PLACE": 5, "DETACH": 6, "RETREAT": 7, "DONE": 8
    }
    current_state = STATES["IDLE"]
    
    # 박스 놓는 위치 (로봇 베이스 좌표계 기준)
    place_locations = {
        "red": np.array([-21.351209889691667, 40.533305206280325, 0.39674743077175845]),
        "blue": np.array([-21.111031364438425, 40.533305206280325, 0.39674743077175845]),
        "yellow": np.array([-20.870852839185186, 40.533305206280325, 0.3846864660083471])
    }
    detected_color = "red" # 임시

    carb.log_info("시뮬레이션을 시작합니다...")

    # 7. 메인 시뮬레이션 루프
    while simulation_app.is_running():
        world.step(render=True) # 매 프레임마다 물리 엔진과 렌더링을 업데이트
        
        if not world.is_playing():
            continue

        cube_pos, _ = red_book.get_world_pose()

        if current_state == STATES["IDLE"]:
            # 컨베이어를 타고 이동한다고 가정. 특정 X축 좌표를 넘으면 동작 시작
            if cube_pos[0] > -11.0: 
                carb.log_info("[상태] 색상 인식 완료, PRE-GRASP로 이동")
                detected_color = "red"
                current_state = STATES["MOVE_TO_PREGRASP"]
                cspace_controller.reset()

        elif current_state == STATES["MOVE_TO_PREGRASP"]:
            target_pos = cube_pos - robot_position + np.array([0, 0, 0.20]) # 큐브 위 20cm
            target_ori = euler_angles_to_quat(np.array([0, np.pi, 0])) # 그리퍼가 아래를 향함

            if execute_motion(ur10_robot, cspace_controller, target_pos, target_ori):
                current_state = STATES["LOWER"]
                cspace_controller.reset()

        elif current_state == STATES["LOWER"]:
            target_pos = cube_pos - robot_position + np.array([0, 0, 0.02]) # 큐브 위 2cm
            target_ori = euler_angles_to_quat(np.array([0, np.pi, 0]))

            if execute_motion(ur10_robot, cspace_controller, target_pos, target_ori):
                current_state = STATES["ATTACH"]

        elif current_state == STATES["ATTACH"]:
            # 시스템 아키텍처에 명시된 Fake Grasp
            
            # [수정] 5.0.0에서는 end_effector 객체를 통해 경로를 가져옵니다.
            ee_prim_path = ur10_robot.end_effector.prim_path 
            carb.log_info(f"[상태] Fake Grasp 작동: 큐브 부착 완료 (End Effector: {ee_prim_path})")
            
            current_state = STATES["LIFT"]
            cspace_controller.reset()

        elif current_state == STATES["LIFT"]:
            target_pos = cube_pos - robot_position + np.array([0, 0, 0.30]) # 위로 들어올림
            target_ori = euler_angles_to_quat(np.array([0, np.pi, 0]))

            if execute_motion(ur10_robot, cspace_controller, target_pos, target_ori):
                current_state = STATES["MOVE_TO_PLACE"]
                cspace_controller.reset()

        elif current_state == STATES["MOVE_TO_PLACE"]:
            target_pos = place_locations[detected_color]
            target_pos[2] += 0.2 # 내려놓기 전 살짝 위
            target_ori = np.array([0.6709312701996814, 0, 0, 0.7415195416630921])

            if execute_motion(ur10_robot, cspace_controller, target_pos, target_ori):
                current_state = STATES["DETACH"]
                cspace_controller.reset()

        elif current_state == STATES["DETACH"]:
            carb.log_info("[상태] Fake Grasp 작동: 큐브 놓기 완료")
            current_state = STATES["RETREAT"]
            cspace_controller.reset()

        elif current_state == STATES["RETREAT"]:
            target_pos = place_locations[detected_color]
            target_pos[2] += 0.4 # 위로 빠짐
            target_ori = np.array([0.6709312701996814, 0, 0, 0.7415195416630921])

            if execute_motion(ur10_robot, cspace_controller, target_pos, target_ori):
                carb.log_info("[완료] Pick and Place Task가 성공적으로 종료되었습니다.")
                current_state = STATES["DONE"]

        elif current_state == STATES["DONE"]:
            # 모든 작업 완료 후 대기
            pass

    # 루프를 빠져나오면 앱 종료
    simulation_app.close()

if __name__ == "__main__":
    main()