"""
robot_waypoint_teleport.py  (v5 - 좌표/방향 이미지 기준 확정)
Home → A → B → C → Home  각 5초 대기 후 순간이동

실행:
    cd ~/Rokey6-A3-SimsFactory/project
    ~/isaacsim/python.sh robot_waypoint_teleport.py
"""

# ─────────────────────────────────────────────
#  설정  (이미지에서 확인된 실제값)
# ─────────────────────────────────────────────
USD_PATH        = "/home/rokey/Rokey6-A3-SimsFactory/project/environment.usd"
ROBOT_PRIM_PATH = "/Root/robot"          # Property 패널 Prim Path 기준

# (x, y, z,  orient_z_deg)
WAYPOINTS = {
    "Home": ( 0.38240,  0.69132, 0.09004, 0.0),
    "A":    ( 5.76333,  8.56107, 0.09004,   90.0),
    "B":    (-1.44080, 11.48552, 0.09004,   90.0),
    "C":    (-4.93302,  8.53896, 0.09004,   90.0),
}
SEQUENCE   = ["Home", "A", "B", "C", "Home"]
DWELL_TIME = 5.0
# ─────────────────────────────────────────────

import math
from isaacsim import SimulationApp
app = SimulationApp({"headless": False})

import time
import omni.usd
from isaacsim.core.api import SimulationContext
from isaacsim.core.utils.stage import open_stage
from pxr import UsdGeom, Gf

# ── RPY(deg) → Gf.Quatd ──────────────────────
def rpy_deg_to_quatd(roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0):
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)
    cr, sr = math.cos(r/2), math.sin(r/2)
    cp, sp = math.cos(p/2), math.sin(p/2)
    cy, sy = math.cos(y/2), math.sin(y/2)
    w  =  cr*cp*cy + sr*sp*sy
    ix =  sr*cp*cy - cr*sp*sy
    iy =  cr*sp*cy + sr*cp*sy
    iz =  cr*cp*sy - sr*sp*cy
    return Gf.Quatd(w, ix, iy, iz)

# ── 환경 로드 ─────────────────────────────────
print(f"\n[INFO] USD 로드: {USD_PATH}")
open_stage(usd_path=USD_PATH)
for _ in range(10):
    app.update()

# ── SimulationContext ─────────────────────────
sim = SimulationContext(stage_units_in_meters=1.0)
sim.play()
for _ in range(30):
    sim.step(render=True)

# ── 로봇 Prim 확인 ────────────────────────────
stage      = omni.usd.get_context().get_stage()
robot_prim = stage.GetPrimAtPath(ROBOT_PRIM_PATH)

if not robot_prim.IsValid():
    print(f"\n[ERROR] prim 없음: {ROBOT_PRIM_PATH}")
    for p in stage.Traverse():
        if p.GetPath().pathString.count("/") <= 3:
            print(f"  {p.GetPath()}")
    app.close()
    raise SystemExit(1)

print(f"[INFO] 로봇 prim OK: {ROBOT_PRIM_PATH}")

# ── 순간이동 함수 ─────────────────────────────
def teleport(prim, x, y, z, orient_z_deg):
    """
    기존 xformOp 타입을 유지하면서 값만 덮어씌움.
    - translate : Vec3d
    - orient    : Quatd (기존 타입 유지, 없으면 PrecisionDouble로 생성)
    """
    xformable    = UsdGeom.Xformable(prim)
    existing_ops = {op.GetOpName(): op for op in xformable.GetOrderedXformOps()}
    quat         = rpy_deg_to_quatd(yaw_deg=orient_z_deg)

    # ── translate ────────────────────────────
    if "xformOp:translate" in existing_ops:
        existing_ops["xformOp:translate"].Set(Gf.Vec3d(x, y, z))
    else:
        xformable.AddTranslateOp().Set(Gf.Vec3d(x, y, z))

    # ── orient (기존 타입 유지) ───────────────
    if "xformOp:orient" in existing_ops:
        op        = existing_ops["xformOp:orient"]
        type_name = op.GetAttr().GetTypeName().type.typeName.lower()
        if "quatd" in type_name:
            op.Set(Gf.Quatd(quat.GetReal(), quat.GetImaginary()))
        else:
            op.Set(Gf.Quatf(
                float(quat.GetReal()),
                float(quat.GetImaginary()[0]),
                float(quat.GetImaginary()[1]),
                float(quat.GetImaginary()[2]),
            ))
    else:
        xformable.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(quat)

    # ── 속도 초기화 ───────────────────────────
    for attr_name in ("physics:velocity", "physics:angularVelocity"):
        attr = prim.GetAttribute(attr_name)
        if attr:
            attr.Set(Gf.Vec3f(0, 0, 0))

    sim.step(render=True)

# ── 웨이포인트 순회 ───────────────────────────
print("\n" + "="*58)
print("  순회 시작:  Home → A → B → C → Home")
print("="*58)

for idx, wp_name in enumerate(SEQUENCE):
    x, y, z, oz = WAYPOINTS[wp_name]
    print(f"\n[{idx+1}/{len(SEQUENCE)}] ▶ {wp_name}")
    print(f"  translate : ({x}, {y}, {z})")
    print(f"  orient Z  : {oz}°")

    teleport(robot_prim, x, y, z, oz)

    print(f"  ⏱  {DWELL_TIME}초 대기...")
    t0 = time.time()
    while time.time() - t0 < DWELL_TIME:
        sim.step(render=True)

print("\n" + "="*58)
print("  순회 완료!  창을 닫으면 종료됩니다.")
print("="*58)

while app.is_running():
    sim.step(render=True)

app.close()