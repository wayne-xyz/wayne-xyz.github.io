---
layout: post
title: "IK Solver for Xlerobot in Issac sim"
date: 2025-10-01 16:00:00 +0000
categories: robot
tags: [robot, ik, xlerobot, issacsim,simulation,chanllenging]
---

## Intro
Recently I am involved in a robotic hackathon, and we plan to using the xlerobot(https://github.com/Vector-Wangel/XLeRobot) in the sim to train some policy, this blog will introduce the ik for control arms and introduce some chanllenging I faced and how to deal with them.

Full video intro : https://www.youtube.com/watch?v=B3NwrR5iIoA&t=159s


## Lula Ik
Lula ik(https://docs.isaacsim.omniverse.nvidia.com/5.0.0/manipulators/manipulators_lula_kinematics.html) requrie prepare the URDF and Yaml config for the ik solver. 
- Attention1:
Yaml file can be generate by the Lula robot description  editor, and still be attention is root_link is requried for the c-space define. 

``` Yaml
api_version: 1.0
cspace:
    - Rotation_2
    - Pitch_2
    - Elbow_2
    - Wrist_Pitch_2
    - Wrist_Roll_2
root_link: Base_2

```

- Attention2
Velocity in the URDF should all over 0 


Challenging in LulaIK- Rare are move due to the low tolerance in LulaIK
Lula ik is based on the CCD (Cyclic Coordinate Descent) and BFGS(quasi-Newton optimization algorithm) 

the tolenracen setting is too low to avoid any movement ,  is that when Lula IK fails it does not output a guaranteed valid joint update, so if you don’t apply its “near-solution” anyway, the articulation state never changes—leaving the seed unchanged and the arm frozen.

My solution is applies the near-solution from a failed IK attempt, keeping the seed close to the target so the solver can converge in the next step instead of restarting far away.

``` Python
# Apply near-solution anyway so the seed can converge across steps
if self._apply_action_on_fail and action_r is not None:
    try:
        self._articulation.apply_action(action_r)
        if (self._step_count % self._debug_every_n) == 0:
            print(f"[XleRobot][Right] applying action despite IK failure")
    except Exception:
        if (self._step_count % self._debug_every_n) == 0:
            print("[XleRobot][Right] IK failed and action could not be applied")
if (self._step_count % self._debug_every_n) == 0:
    try:
        ee_pos_r_b, ee_rot_r_b = self._right_articulation_ik.compute_end_effector_pose()
        pos_err_b = float(np.linalg.norm(tgt_pos_r_b - np.array(ee_pos_r_b)))
        print(f"[XleRobot][Right] IK failed. world_pos_err={pos_err_r:.4f} base_pos_err={pos_err_b:.4f} tgt_b={tgt_pos_r_b.tolist()}")
    except Exception:
        print("[XleRobot][Right] IK failed target=", trg_pos)
```


## Differential IK

Since my teammate create edited USD file, I switch to the differential ik without the urdf and yaml. 

Chanllgening 1:
This is the floating base robotic, the ik solver will have diffeertn Phyx data size from the the fixed base. Without the offset, the dof indice and jacobian will gain the wrong columns. In the float base, there will have a base_dof_offset be added into the Phyx. 

```Python 
# float base  physx dofs is longer than the robot dofs so we need to offset the joint indices
total_dofs = jacobians.shape[-1]
base_dof_offset = max(total_dofs - len(self.robot.data.joint_names), 0)
left_dof_indices = self._left_joint_ids + base_dof_offset if base_dof_offset else self._left_joint_ids
left_jacobian = jacobians[:, left_ee_jacobi_idx, :, left_dof_indices].clone()
left_joint_pos = self.robot.data.joint_pos[:, self._left_joint_ids]

```


Chanllenging 2:
Differential Ik is plain ik, which requried more stratgies to optimize the move and paramters adjust. 

Optimize 1 : 
Adaptive damping avoid jitter at low lambda, and latency at high value.

```Python
   def _adaptive_lambda(self, J: torch.Tensor,pos_err_mag: torch.Tensor,*,
        base_lambda: float = 0.1,   # small, always-on damping
        lam_max: float = 0.8,       # cap (adjust after you see variation)
        k_manip: float = 0.10,       # manipulability sensitivity
        k_err: float = 0.20,         # error sensitivity
        sigma_ref: float = 0.20,     # "healthy" σ_min
        manip_slope: float = 0.08,   # softness of manipulability curve
        err_cap: float = 0.30,       # 50 cm → avoids instant saturation
        use_position_rows: bool = True,
        log_debug: bool = False
    ) -> torch.Tensor:
        """
        Adaptive DLS damping using a smooth manipulability term (via softplus) and a capped
        error term. Accepts J of shape [6,dof]/[B,6,dof] or [3,dof]/[B,3,dof].
        Returns scalar (unbatched) or [B] (batched).
        """
        # ---- normalize shapes ----
        batched = (J.dim() == 3)
        if not batched:
            J = J.unsqueeze(0)  # [1, r, dof]

        # ---- choose rows (prefer 3xDOF for 5-DOF arm) ----
        if use_position_rows and J.shape[1] >= 6:
            Jt = J[:, 0:3, :]
        else:
            Jt = J

        device, dtype = Jt.device, Jt.dtype

        # ---- singular values and σ_min ----
        svals = torch.linalg.svdvals(Jt)          # [B, min(r,dof)]
        sigma_min = svals.min(dim=1).values       # [B]

        # ---- constants as tensors (fixes the softplus float error) ----
        sigma_ref_t   = torch.as_tensor(sigma_ref,   device=device, dtype=dtype)
        manip_slope_t = torch.as_tensor(manip_slope, device=device, dtype=dtype)
        err_cap_t     = torch.as_tensor(err_cap,     device=device, dtype=dtype)
        base_lambda_t = torch.as_tensor(base_lambda, device=device, dtype=dtype)

        # ---- manipulability term (smooth, bounded, ~0 when σ>=sigma_ref) ----
        # x grows as sigma_min drops below sigma_ref; softplus keeps it gentle
        x = (sigma_ref_t - sigma_min) / manip_slope_t
        num   = torch.nn.functional.softplus(x)
        denom = torch.nn.functional.softplus(sigma_ref_t / manip_slope_t) + 1e-6
        manip_term = (num / denom).clamp(0.0, 1.0)    # [B]

        # ---- error term (0→1 over 0→err_cap, then capped) ----
        if pos_err_mag.dim() == 0:
            pos_err_mag = pos_err_mag.expand(Jt.shape[0])
        elif pos_err_mag.dim() > 1:
            pos_err_mag = pos_err_mag.reshape(-1)
        err_term = (pos_err_mag / err_cap_t).clamp(0.0, 1.0)  # [B]

        # ---- assemble λ and clamp ----
        lam = base_lambda_t + k_manip * manip_term + k_err * err_term
        lam = lam.clamp(max=lam_max)                           # [B]

        if log_debug:
            print(f"[AdaptiveLambdaDBG] "
                f"sigma_min_mean={sigma_min.mean().item():.4f} "
                f"manip_term_mean={manip_term.mean().item():.3f} "
                f"err_term_mean={err_term.mean().item():.3f} "
                f"lambda_mean={lam.mean().item():.3f}")

        return lam if batched else lam.squeeze(0)

```


I also compare witht the low pass filter ,which did not gain the good result , i did not apply it. 

Optimize 2:
When we move the base, the arm's unwanted move will be affect the interial mounmanta due to no-sync with the target .
So I applied the sync strategies and the cool-down method to avoid the arm's dragging back movement. 


## Perf Compare 

| IK Method                              | Avg. Time per Frame |
|-----------------------------------------|---------------------|
| Differential IK (2 arms)                | ~3.1 ms             |
| Differential IK (2 arms, Adaptive Damping) | ~5-6 ms             |
| Lula IK (2 arms)                        | ~12 ms              |
