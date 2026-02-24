from pxr import UsdGeom
import omni.usd

def get_world_xform(prim):
    xcache = UsdGeom.XformCache()  # time=default(0)
    return xcache.GetLocalToWorldTransform(prim)

def get_relative_xform(from_path, to_path):
    stage = omni.usd.get_context().get_stage()

    from_prim = stage.GetPrimAtPath(from_path)
    to_prim   = stage.GetPrimAtPath(to_path)

    if not from_prim.IsValid():
        raise RuntimeError(f"Invalid from prim path: {from_path}")
    if not to_prim.IsValid():
        raise RuntimeError(f"Invalid to prim path: {to_path}")

    T_W_from = get_world_xform(from_prim)
    T_W_to   = get_world_xform(to_prim)

    T_rel = T_W_from.GetInverse() * T_W_to

    # 상대 위치
    rel_pos = T_rel.ExtractTranslation()

    # 상대 회전 (Quaternion)
    rel_rot = T_rel.ExtractRotation()   # Gf.Rotation
    rel_quat = rel_rot.GetQuat()        # Gf.Quatd (w + xyz)

    return rel_pos, rel_quat


# ===== 사용 예시 =====
rel_pos, rel_quat = get_relative_xform("/Root/ur10","/Root/run_robot/small_KLT_02")

print("Relative position:", rel_pos)
print("Relative quaternion (w, x, y, z):",
      rel_quat.GetReal(),
      rel_quat.GetImaginary())
