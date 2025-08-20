import isaaclab.sim as sim_utils
from isaaclab.actuators import DCMotorCfg,ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
import os
dirname, filename = os.path.split(os.path.abspath(__file__))

# Actuator Configs 

HAA_ACTUATOR_CFG = ImplicitActuatorCfg(
    joint_names_expr=["HAA.*"],
    effort_limit_sim=1,
    velocity_limit_sim=1,
    stiffness={".*": 10.0},
    damping={".*": 1.0},
    # each of these are enabled for each test
    friction={".*": 0.5},
    # viscous_friction={".*": 1}
    # dynamic_friction={".*": 10}
)

KFE_ACTUATOR_CFG = ImplicitActuatorCfg(
    joint_names_expr=["KFE.*"],
    effort_limit_sim=1*1.5,
    velocity_limit_sim=1,
    stiffness={".*": 10.0},
    damping={".*": 1.0},
    # friction={".*": 10},
    # dynamic_friction={".*": 10}
)

HFE_ACTUATOR_CFG = ImplicitActuatorCfg(
    joint_names_expr=["HFE.*"],
    effort_limit_sim=1,
    velocity_limit_sim=1,
    stiffness={".*": 10.0},
    damping={".*": 1.0},
    # friction={".*": 10},
    # dynamic_friction={".*": 10}
)

#Articulation Configs

MISTLETOE_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        # assume submodule is installed in the root of the repo
        usd_path= dirname + '/../../../../Mistletoe-Hardware/v4/sim_files/mistletoe-v4-fixed.usd',

        # Adapted from Anymal Velocity Tracking Example's Config
        activate_contact_sensors=True,
        
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),

        # TODO: self-collisions could cause self destruction not relaly sure
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, solver_position_iteration_count=4, solver_velocity_iteration_count=0
        ),


    ),

    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0),
        joint_pos={
            "HAA.*": 0.0,  # all HAA
            "HFE.*": 0.0,  # all HAA
            "KFE.*": 0.0,  # all HAA

            # # positive on left, negative on right for both HFE and KFE
            # "HFE_[13]": 0.523599,  
            # "HFE_[24]": -0.523599,  
            # "KFE_[24]": -0.872665,  # right
            # "KFE_[13]": 0.872665,  # left
        },
        joint_vel={".*": 0.0},
    ),

    actuators={
        "HAA": HAA_ACTUATOR_CFG,
        "HFE": HFE_ACTUATOR_CFG,
        "KFE": KFE_ACTUATOR_CFG
    },

    # soft_joint_pos_limit_factor=0.9
)