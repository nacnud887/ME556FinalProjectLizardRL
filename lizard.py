# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for LIBR Lizard"""

from __future__ import annotations

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

##
# Configuration
##

LIZARD_CFG = ArticulationCfg(
    prim_path="{ENV_REGEX_NS}/Robot",
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"C:/Users/dja5722/Documents/DJAIsaacLab/LizardFiles/V1TailAssemblyUSD/V1TailAssembly.usd",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=1.0,
            enable_gyroscopic_forces=True,
            max_angular_velocity = 5.0, #57 rev/min is max unloaded motor speed - 5.96 rad/s *@11.1 V
            max_linear_velocity = 5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
            sleep_threshold=0.01,
            stabilization_threshold=0.001,
        ),
        #copy_from_source=False,
        activate_contact_sensors=True,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.1),
        joint_pos={
            #".*_leg": 0.0,
            "Front": 0,
            "Back": 0,
            "Tail": 0,
            "FR": -0.785398,
            "FL": -0.785398,
            "HR": -0.785398,
            "HL": -0.785398,
        },
    ),
    actuators={
        "body": ImplicitActuatorCfg(
            joint_names_expr=[".*"],
            stiffness=80.0,
            damping=4.0,
        ),
    },
)
"""Configuration for LIBR Lizard"""