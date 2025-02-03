#imports
import numpy as np
import os
import pydrake

from pydrake.all import *


# Check where we are
#print("Current directory:", os.getcwd())

#fixing timing 
simulation_time = 8.0   # Desired duration of the simulation [s].
realtime_rate = 1.0     # Desired real time rate.
time_step = 1e-2        # Discrete time step for the system [s]. Must be positive.
render_flag = True

# Column and joint position (everything in meters)
column_height = 0.20
joint_height = 0.08
joint_radius = 0.055
leg_height = 0.675

# Initiliazing the model
builder = DiagramBuilder()           #creating plant

plant_config = MultibodyPlantConfig()
plant_config.time_step = time_step

plant, scene_graph = AddMultibodyPlant(plant_config, builder)

#Add a renderer to render the inside dot pattern of the bubble gripper.
#Currently (April 2024), deformable rendering is only supported by
#RenderEngineGl.
if render_flag:
    scene_graph.AddRenderer("gl_renderer", MakeRenderEngineGl(RenderEngineGlParams()))

#Minimum required proximity properties for rigid bodies to interact with
#deformable bodies.
#1. A valid Coulomb friction coefficient, and
#2. A resolution hint. (Rigid bodies need to be tessellated so that collision
#queries can be performed against deformable geometries.) 


rigid_proximity_props = ProximityProperties()
surface_friction = CoulombFriction(1.15, 1.15)
resolution_hint = 0.01;
AddContactMaterial(friction = surface_friction,  properties = rigid_proximity_props)
rigid_proximity_props.AddProperty("hydroelastic", "resolution_hint", resolution_hint)


#Rigid parts
parser = Parser(plant)              # Parser for URDF files

# Adjust paths to correctly point to URDF files
base_column_path = "oneleg_column.urdf"
leg_path = "oneleg_leg.urdf"

# Verify file paths before loading URDF files
if not os.path.exists(base_column_path):
    raise FileNotFoundError(f"File not found: {base_column_path}")

if not os.path.exists(leg_path):
    raise FileNotFoundError(f"File not found: {leg_path}")


column_models = parser.AddModels(base_column_path)  # Returns a list
leg_models = parser.AddModels(leg_path)

# Get the column instance and its frame
colonna_model_instance = column_models[0]  # Ensure you're using the correct instance
body_base = plant.GetBodyByName("Link_colonna", colonna_model_instance)
column_frame = body_base.body_frame()

# Weld the column to the world frame
plant.WeldFrames(
    plant.world_frame(), 
    column_frame, 
    RigidTransform([0, 0, 0])
)

#print(plant.GetDefaultFreeBodyPose(body_base))          #check position



## ---------------------------------------------------------------------------- DEFORMABLE PART (JOINT) ---------------------------------------------------------------------------------------

# Material properties for the elastic joint (steel)
density_ms = 7800          # kg/m³
youngs_modulus_ms = 200e9  # Young's Modulus for Maraging steel (Pa)
poisson_ratio_ms = 0.3     # Poisson's ratio for steel
beta = 0.01                # Stiffness damping coefficient for the deformable body [1/s].

# Assign the  material model with linear corotated material model
#documentation: https://drake.mit.edu/pydrake/pydrake.multibody.fem.html?highlight=deformablebodyconfig#pydrake.multibody.fem.MaterialModel
material_model_joint = MaterialModel.kLinearCorotated

#Geometry
joint_path = "volumetric_vtk_joint_no_triangles.vtk"  # Update with the actual mesh file path
if not os.path.exists(joint_path):
    raise FileNotFoundError(f"Mesh file not found: {joint_path}")

#pose the joint in world frame so that it is place over the column for 0.01
X_WB = RigidTransform(RotationMatrix(), [0, 0,0.23])

# geometry instance for the joint: define mesh and place it
joint_geometry = GeometryInstance(X_WB,  Mesh("volumetric_vtk_joint_no_triangles.vtk"),  "deformable_joint" )

#jont properties
joint_deformable_model = DeformableModel(plant)
config = DeformableBodyConfig()
config.set_youngs_modulus(youngs_modulus=youngs_modulus_ms)
config.set_poissons_ratio(poissons_ratio=poisson_ratio_ms)
config.set_stiffness_damping_coefficient(beta)
config.set_mass_density(mass_density=density_ms)
config.set_material_model(material_model_joint)


# Minimumly required proximity properties for deformable bodies:
# A valid Coulomb friction coefficient.
deformable_proximity_props = ProximityProperties()
AddContactMaterial(friction=surface_friction, properties=deformable_proximity_props)
joint_geometry.set_proximity_properties(deformable_proximity_props)

fem_resolution_hint = 0.05  # Controls FEM discretization
joint_deformable_model = plant.mutable_deformable_model()

#register the deformbale model
deformable_body_id = joint_deformable_model.RegisterDeformableBody(
    geometry_instance=joint_geometry,
    config=config,
    resolution_hint=fem_resolution_hint
)
deformable_model = joint_deformable_model

#Now we attach the bubble to the WSG finger using a fixed constraint. To do
#that, we specify a box geometry and put all vertices of the bubble geometry
#under fixed constraint with the rigid finger if they fall inside the box.
#Refer to DeformableModel::AddFixedConstraint for details. 


#joint in the column frame
X_JC = RigidTransform(RotationMatrix(), np.array([0,0, 0.13]))
#All vertices of the deformable bubble mesh inside this box will be subject 
box_column = Box(0.06, 0.06, 0.47)  

#attach to column (see attched sheet for the reasoning)
deformable_model.AddFixedConstraint(deformable_body_id, body_base, X_JC, box_column, RigidTransform(RotationMatrix(), [0, 0, 0.095]) ) 


#get leg frame and put it in initial pose sligtly above the joint
leg_model_instance = plant.GetModelInstanceByName("PIPPA_oneleg_leg")
leg_base = plant.GetBodyByName("Gamba_Link", leg_model_instance)
leg_heigth_center = 0.26 +leg_height/2
leg_initial_pose = RigidTransform([0, 0, leg_heigth_center])  
plant.SetDefaultFreeBodyPose(leg_base, leg_initial_pose)    

X_JL = RigidTransform(RotationMatrix(), [0,0, -leg_height/2])

box_leg = Box(0.06, 0.06, 0.8)  
#attach the leg
deformable_model.AddFixedConstraint(deformable_body_id, leg_base, X_JL, box_leg, RigidTransform(RotationMatrix(), [0,0, 0.265])) 

plant.Finalize()    #finalize plant
# Check the number of DOF and velocities for the deformable body
num_dof = plant.num_velocities()
print(f"Number of velocities: {num_dof}")


#create camera for rendering
if render_flag:
    camera_config = CameraConfig(
        width=1280,
        height=720,
        focal=CameraConfig.FovDegrees(y=90),
        clipping_near=0.001,
        renderer_name="gl_renderer",
        show_rgb=True
    )
    ApplyCameraConfig(camera_config, builder)

diagram = builder.Build()
diagram_context = diagram.CreateDefaultContext()

simulator = Simulator(diagram, diagram_context)
mutable_root_context = simulator.get_mutable_context()
plant_context =diagram.GetMutableSubsystemContext(plant, mutable_root_context)
simulator.Initialize()
simulator.set_target_realtime_rate(realtime_rate)
simulator.AdvanceTo(simulation_time)