import argparse
import os

def generate_urdf(num_links, file_name="robot_arm.urdf", link_length=0.2, 
                 link_radius=0.05, mass_base=1.0, mass_reduction_factor=0.1,
                 joint_effort=100, joint_velocity=1.0, colorful=True):
    if num_links < 1:
        print("Error: Number of links must be at least 1")
        return
    colors = [
        {"name": "blue", "rgba": "0.0 0.0 0.8 1.0"},
        {"name": "red", "rgba": "0.8 0.0 0.0 1.0"},
        {"name": "green", "rgba": "0.0 0.8 0.0 1.0"},
        {"name": "yellow", "rgba": "0.8 0.8 0.0 1.0"},
        {"name": "purple", "rgba": "0.8 0.0 0.8 1.0"},
        {"name": "cyan", "rgba": "0.0 0.8 0.8 1.0"},
        {"name": "orange", "rgba": "1.0 0.5 0.0 1.0"},
        {"name": "grey", "rgba": "0.5 0.5 0.5 1.0"}
    ]
    
    urdf = f"""<?xml version="1.0"?>
<robot name="{num_links}_dof_arm">
    <!-- Materials -->
"""
    
    for color in colors:
        urdf += f"""    <material name="{color["name"]}">
        <color rgba="{color["rgba"]}"/>
    </material>
"""
    
    # Base link
    urdf += f"""
    <!-- Base Link -->
    <link name="base_link">
        <visual>
            <geometry>
                <cylinder radius="{link_radius*1.5}" length="0.05"/>
            </geometry>
            <material name="grey"/>
        </visual>
        <collision>
            <geometry>
                <cylinder radius="{link_radius*1.5}" length="0.05"/>
            </geometry>
        </collision>
        <inertial>
            <mass value="{mass_base*1.2}"/>
            <inertia ixx="{mass_base*1.2*0.01}" ixy="0" ixz="0" iyy="{mass_base*1.2*0.01}" iyz="0" izz="{mass_base*1.2*0.01}"/>
        </inertial>
    </link>
"""
    
    # Create links and joints
    for i in range(1, num_links + 1):
        current_mass = mass_base * (1.0 - (i-1) * mass_reduction_factor / num_links)
        if current_mass < 0.1:
            current_mass = 0.1
            
        if colorful:
            color_index = (i-1) % len(colors)
            color = colors[color_index]["name"]
        else:
            color = "grey"
        
        shape = "box" if i % 2 == 1 else "cylinder"
        
        if i == 1:
            axis = "0 0 1"  # z-axis 
            joint_limits = "-3.14 3.14"
        else:
            axis = "0 1 0"  # y-axis 
            joint_limits = "-1.57 1.57"
            
        inertia_xx = current_mass * 0.005
        inertia_yy = current_mass * 0.005
        inertia_zz = current_mass * 0.005
        
        # Joint connecting previous link to this link
        previous_link = "base_link" if i == 1 else f"link{i-1}"
        joint_z_offset = 0.025 if i == 1 else link_length  # Special case for first joint
        
        urdf += f"""
    <!-- Joint {i}: {previous_link} to link{i} (Revolute) -->
    <joint name="joint{i}" type="revolute">
        <parent link="{previous_link}"/>
        <child link="link{i}"/>
        <origin xyz="0 0 {joint_z_offset}" rpy="0 0 0"/>
        <axis xyz="{axis}"/>
        <limit lower="{joint_limits.split()[0]}" upper="{joint_limits.split()[1]}" effort="{joint_effort/(i/2) if i > 1 else joint_effort}" velocity="{joint_velocity * (1 + i/10)}"/>
    </joint>
"""
        
        if shape == "box":
            geometry = f"""<box size="{link_radius*2} {link_radius*1.5} {link_length}"/>"""
            origin = f"""<origin xyz="0 0 {link_length/2}" rpy="0 0 0"/>"""
        else:
            geometry = f"""<cylinder radius="{link_radius * (1.0 - i * 0.03)}" length="{link_length}"/>"""
            origin = f"""<origin xyz="0 0 {link_length/2}" rpy="0 0 0"/>"""
        
        urdf += f"""
    <!-- Link {i} -->
    <link name="link{i}">
        <visual>
            <geometry>
                {geometry}
            </geometry>
            {origin}
            <material name="{color}"/>
        </visual>
        <collision>
            <geometry>
                {geometry}
            </geometry>
            {origin}
        </collision>
        <inertial>
            <mass value="{current_mass:.3f}"/>
            {origin}
            <inertia ixx="{inertia_xx:.6f}" ixy="0" ixz="0" iyy="{inertia_yy:.6f}" iyz="0" izz="{inertia_zz:.6f}"/>
        </inertial>
    </link>
"""
    
    # Add end effector
    urdf += f"""
    <!-- Joint {num_links+1}: link{num_links} to End Effector (Fixed) -->
    <joint name="joint{num_links+1}" type="revolute">
        <parent link="link{num_links}"/>
        <child link="end_effector"/>
        <origin xyz="0 0 {link_length}" rpy="0 0 0"/>
        <axis xyz="0 1 0"/>
        <limit lower="-1.57" upper="1.57" effort="{joint_effort/3}" velocity="{joint_velocity*2}"/>
    </joint>

    <!-- End Effector -->
    <link name="end_effector">
        <visual>
            <geometry>
                <sphere radius="{link_radius * 0.8}"/>
            </geometry>
            <material name="grey"/>
        </visual>
        <collision>
            <geometry>
                <sphere radius="{link_radius * 0.8}"/>
            </geometry>
        </collision>
        <inertial>
            <mass value="0.1"/>
            <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
        </inertial>
    </link>
</robot>
"""
    if "urdf" not in os.listdir("Snake"):
        os.mkdir("urdf")

    with open("Snake/urdf/"+file_name, 'w') as f:
        f.write(urdf)
    
    print(f"Successfully generated URDF file '{file_name}' with {num_links} links and {num_links + 1} DOF")
    print(f"Total height of robot: ~{0.05 + (num_links * link_length) + link_radius:.2f}m")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Generate a URDF file for a robot arm with n links')
    parser.add_argument('num_links', type=int, help='Number of max links for the robot arm')
    parser.add_argument('--output', '-o', default='robot_arm', help='Output file name')
    parser.add_argument('--length', '-l', type=float, default=0.2, help='Length of each link (meters)')
    parser.add_argument('--radius', '-r', type=float, default=0.05, help='Radius of each link (meters)')
    parser.add_argument('--base-mass', '-m', type=float, default=1.0, help='Mass of the base link (kg)')
    parser.add_argument('--plain', action='store_false', dest='colorful', help='Use only grey color for links')
    
    args = parser.parse_args()
    for i in range(10, args.num_links, 10):
        generate_urdf(i, f"{args.output}_{i}.urdf", args.length, args.radius, args.base_mass, colorful=args.colorful)