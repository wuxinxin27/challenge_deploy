"""Supervised slow joint positioning. Requires an operator at the emergency stop."""
import argparse
import json
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile
import time

import numpy as np
import yaml

from hardware.piper import SinglePiperArm
from hardware.conversions import joints_rad_to_sdk, opening_to_sdk_gripper
from .core import write_json
from .set_reference_grippers import targets as gripper_targets

LOW = np.array([-2.6179, 0, -2.967, -1.745, -1.22, -2.09439])
HIGH = np.array([2.6179, 3.14, 0, 1.745, 1.22, 2.09439])


def checked_state(arm, *, gripper=False):
    s = arm.read_state()
    stamp = arm.interface.GetArmJointMsgs().time_stamp
    if stamp <= 0 or not 0 <= time.time() - stamp < .3:
        raise ValueError(f"{arm.name}: stale joint feedback")
    if not arm.is_enabled() or not np.isfinite(s.qpos).all():
        raise ValueError(f"{arm.name}: disabled or invalid state")
    if gripper:
        stamp = arm.interface.GetArmGripperMsgs().time_stamp
        if stamp <= 0 or not 0 <= time.time() - stamp < .3:
            raise ValueError(f"{arm.name}: stale gripper feedback")
    return s


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--reference", type=Path, help="Defaults to calibration.reference in config")
    p.add_argument("--output", type=Path, help="Defaults to a unique directory under /tmp")
    p.add_argument("--config", default="configs/dual_piper_example.yaml")
    p.add_argument("--execute", action="store_true")
    p.add_argument("--arm", choices=("left", "right", "both"), default="both")
    p.add_argument("--max-joint-speed", type=float, help="Peak interpolation speed, rad/s (0.02 to 0.30)")
    p.add_argument("--countdown", type=float, help="Seconds before movement (1 to 30)")
    p.add_argument("--grippers", action=argparse.BooleanOptionalAction, default=None)
    p.add_argument("--raw-to-meters", type=float, help="Historical gripper scale; measured references use 1")
    p.add_argument("--open-overlay", action=argparse.BooleanOptionalAction, default=None)
    p.add_argument("--camera", choices=("cam_high", "cam_left_wrist", "cam_right_wrist"), default="cam_high")
    args = p.parse_args()
    cfg = yaml.safe_load(Path(args.config).read_text())
    workflow = cfg.get("calibration", {})
    args.max_joint_speed = args.max_joint_speed if args.max_joint_speed is not None else workflow.get("max_joint_speed", .15)
    args.countdown = args.countdown if args.countdown is not None else workflow.get("countdown_s", 5.)
    args.grippers = args.grippers if args.grippers is not None else workflow.get("set_grippers", False)
    args.open_overlay = args.open_overlay if args.open_overlay is not None else workflow.get("open_overlay", False)
    speed_percent = workflow.get("speed_percent", 15)
    if not .02 <= args.max_joint_speed <= .3:
        raise ValueError("Joint speed must be between 0.02 and 0.30 rad/s")
    if not 1 <= args.countdown <= 30 or not isinstance(speed_percent, int) or not 1 <= speed_percent <= 100:
        raise ValueError("Invalid countdown or controller speed percentage")
    if args.reference is None:
        if not workflow.get("reference"):
            p.error("--reference or calibration.reference is required")
        args.reference = Path(workflow["reference"])
    ref = json.loads((args.reference / "reference.json").read_text())
    names = ("left", "right") if args.arm == "both" else (args.arm,)
    target = {n: np.asarray(ref["joint_angles"][n], dtype=float) for n in names}
    for q in target.values():
        if q.shape != (6,) or not np.isfinite(q).all() or np.any(q < LOW) or np.any(q > HIGH):
            raise ValueError("Target outside SDK joint limits")
    grip_target = {}
    if args.grippers:
        scale = args.raw_to_meters if args.raw_to_meters is not None else ref.get("raw_to_meters", workflow.get("raw_to_meters"))
        if scale is None:
            p.error("Combined gripper positioning requires --raw-to-meters or a configured scale")
        if ref.get("gripper_unit") == "meters" and scale != 1:
            raise ValueError("Measured reference grippers are meters; use scale 1")
        grip_target = {n:v for n,v in gripper_targets(ref, scale).items() if n in names}
    overlay_reference = Path(workflow.get("overlay_reference", str(args.reference)))
    if args.execute and args.open_overlay:
        for filename in ("reference.json", args.camera + ".png"):
            if not (overlay_reference / filename).is_file():
                raise ValueError(f"Missing overlay reference: {overlay_reference / filename}")
    if args.output is None:
        args.output = Path(tempfile.mkdtemp(prefix="platform2_alignment_", dir="/tmp")) / "motion"
    args.output.mkdir(parents=True, exist_ok=False)
    print(f"OUTPUT {args.output}", flush=True)
    arms = {}
    started = False
    report = {"reference": ref, "execute": args.execute, "arms": list(names),
              "max_joint_speed_rad_s": args.max_joint_speed, "speed_percent": speed_percent,
              "countdown_s": args.countdown, "grippers": grip_target or "unchanged", "samples": []}
    def read(arm):
        return checked_state(arm, gripper=args.grippers)
    try:
        for name in target:
            arms[name] = SinglePiperArm(name=name, can_name=cfg["robot"][name]["can_name"], commands_enabled=args.execute)
            arms[name].connect(read_only=True)
        time.sleep(1)
        states = {n: read(a) for n, a in arms.items()}
        start = {n: s.qpos[:6].copy() for n, s in states.items()}
        grip_start = {n: float(s.qpos[6]) for n,s in states.items()} if args.grippers else {}
        if any(np.any(q < LOW-.02) or np.any(q > HIGH+.02) for q in start.values()):
            raise ValueError("Start outside joint-limit tolerance")
        duration = max(1., max(np.max(abs(target[n]-start[n])) for n in start) * 1.5 / args.max_joint_speed)
        if args.grippers:
            # Keep gripper peak speed at 20 mm/s while sharing the joint trajectory.
            duration = max(duration, max(abs(grip_target[n]-grip_start[n]) for n in names) * 1.5 / .02)
        report.update(start={n:q.tolist() for n,q in start.items()}, duration_s=duration)
        write_json(args.output / "preflight.json", report)
        print(json.dumps({"duration_s": duration, "start": report["start"], "target": {n:q.tolist() for n,q in target.items()}}), flush=True)
        if not args.execute:
            return
        print(f"MOVEMENT IN {args.countdown:g} SECONDS. Keep clear; emergency stop ready.", flush=True)
        time.sleep(args.countdown)
        for n,a in arms.items():
            state = read(a)
            if np.max(abs(state.qpos[:6] - start[n])) > .03:
                raise ValueError("Arm moved during countdown; refusing changed start")
            if args.grippers and abs(state.qpos[6] - grip_start[n]) > .003:
                raise ValueError("Gripper moved during countdown")
        started = True
        for a in arms.values():
            a.set_joint_mode(speed_percent=speed_percent)
        previous = start
        begin = time.monotonic()
        for step in range(int(duration * 25) + 51):
            u = min(1., step / (duration * 25))
            easing = u*u*(3-2*u)
            states = {n: read(a) for n,a in arms.items()}
            for n,s in states.items():
                if np.max(abs(s.qpos[:6] - previous[n])) > .15:
                    raise ValueError(f"{n}: tracking error exceeds 0.15 rad")
            command = {n: np.clip(start[n] + easing*(target[n]-start[n]), LOW, HIGH) for n in start}
            for n,a in arms.items():
                a.interface.JointCtrl(*[int(x) for x in joints_rad_to_sdk(command[n])])
                if args.grippers:
                    width = grip_start[n] + easing*(grip_target[n]-grip_start[n])
                    a.interface.GripperCtrl(abs(opening_to_sdk_gripper(width)), 500, 0x01, 0)
            previous = command
            if step % 25 == 0:
                report["samples"].append({"time_s": time.monotonic()-begin, "qpos": {n:s.qpos.tolist() for n,s in states.items()}})
                print(f"Moving {min(100,100*u):.0f}%", flush=True)
            time.sleep(max(0, begin + (step+1)/25 - time.monotonic()))
        final = {n: read(a).qpos.tolist() for n,a in arms.items()}
        error = {n: float(np.max(abs(np.asarray(final[n])[:6]-target[n]))) for n in final}
        report.update(final=final, max_joint_error_rad=error)
        if max(error.values()) > .025:
            raise ValueError("Final joint tolerance not reached")
        if args.grippers:
            grip_error = {n: abs(final[n][6]-grip_target[n])*1000 for n in names}
            report["gripper_error_mm"] = grip_error
            if max(grip_error.values()) > 2:
                raise ValueError("Final gripper tolerance not reached (2 mm)")
        report["status"] = "reached"
        print("REACHED " + json.dumps(error), flush=True)
    except BaseException as exc:
        report.update(status="aborted", error=str(exc))
        if started:
            for arm in arms.values():
                try:
                    arm.interface.MotionCtrl_1(emergency_stop=0x01)
                except Exception:
                    pass
        raise
    finally:
        write_json(args.output / "motion_report.json", report)
        for arm in arms.values():
            arm.disconnect()
    if args.execute and args.open_overlay:
        # Launch only after successful arrival and after releasing the CAN connections.
        command = [sys.executable, "-m", "calibration.live_overlay", "--config", args.config,
                   "--reference", str(overlay_reference), "--camera", args.camera,
                   "--output", str(args.output / "overlay")]
        result = subprocess.run(command, check=False)
        report["overlay_returncode"] = result.returncode
        write_json(args.output / "motion_report.json", report)
        if result.returncode:
            command[-1] = str(args.output / f"overlay_retry_{time.time_ns()}")
            print("机械臂已到位；叠图启动/运行失败。无需重新执行运动，仅重开窗口：\n"
                  + "PYTHONDONTWRITEBYTECODE=1 " + shlex.join(command), flush=True)
            raise SystemExit(result.returncode)


if __name__ == "__main__":
    main()
