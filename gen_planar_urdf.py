import argparse


def generate_planar_robot_urdf(num_links, file_name="planar_robot.urdf", link_length=0.2, 
                         link_width=0.05, link_height=0.025, base_size=0.1, 
                         mass_base=1.0, mass_reduction_factor=0.1,
                         joint_effort=100, joint_velocity=1.0, colorful=True):
    
    if num_links < 1:
        print("Error: Number of links must be at least 1")
        return
    
    # Colors for links
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
    
    # Start the URDF file
    urdf = f"""<?xml version="1.0"?>
<robot name="planar_{num_links}_dof_robot">
    <!-- Materials -->
"""
    
    # Add materials
    for color in colors:
        urdf += f"""    <material name="{color["name"]}">
        <color rgba="{color["rgba"]}"/>
    </material>
"""
    
    # Base link - make it flat and wider
    urdf += f"""
    <!-- Base Link - flat on the floor -->
    <link name="base_link">
        <visual>
            <geometry>
                <box size="{base_size} {base_size} {link_height}"/>
            </geometry>
            <origin xyz="0 0 {link_height/2}" rpy="0 0 0"/>
            <material name="grey"/>
        </visual>
        <collision>
            <geometry>
                <box size="{base_size} {base_size} {link_height}"/>
            </geometry>
            <origin xyz="0 0 {link_height/2}" rpy="0 0 0"/>
        </collision>
        <inertial>
            <mass value="{mass_base}"/>
            <origin xyz="0 0 {link_height/2}" rpy="0 0 0"/>
            <inertia 
                ixx="{mass_base * (base_size**2 + link_height**2) / 12}" 
                ixy="0" ixz="0" 
                iyy="{mass_base * (base_size**2 + link_height**2) / 12}" 
                iyz="0" 
                izz="{mass_base * (base_size**2 + base_size**2) / 12}"/>
        </inertial>
    </link>
"""
    
    # Create links and joints
    for i in range(1, num_links + 1):
        # Calculate current link properties
        current_mass = mass_base * (1.0 - (i-1) * mass_reduction_factor / num_links)
        if current_mass < 0.1:
            current_mass = 0.1
            
        # Choose color
        if colorful:
            color_index = (i-1) % len(colors)
            color = colors[color_index]["name"]
        else:
            color = "grey"
        
        # Calculate inertia for a box
        ixx = current_mass * (link_width**2 + link_height**2) / 12
        iyy = current_mass * (link_length**2 + link_height**2) / 12
        izz = current_mass * (link_length**2 + link_width**2) / 12
        
        # Joint connecting previous link to this link
        previous_link = "base_link" if i == 1 else f"link{i-1}"
        
        # For planar robot, each joint is at the end of the previous link
        if i == 1:
            # First joint is at the edge of the base
            joint_x_offset = base_size/2
            joint_y_offset = 0
        else:
            # Other joints are at the end of the previous link
            joint_x_offset = link_length
            joint_y_offset = 0
        
        urdf += f"""
    <!-- Joint {i}: {previous_link} to link{i} (Revolute around Z-axis) -->
    <joint name="joint{i}" type="revolute">
        <parent link="{previous_link}"/>
        <child link="link{i}"/>
        <origin xyz="{joint_x_offset} {joint_y_offset} 0" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
        <limit lower="-3.14" upper="3.14" effort="{joint_effort/(i/2) if i > 1 else joint_effort}" velocity="{joint_velocity}"/>
    </joint>
"""
        
        # Link - rectangular prism oriented along the x-axis
        urdf += f"""
    <!-- Link {i} - flat on the floor -->
    <link name="link{i}">
        <visual>
            <geometry>
                <box size="{link_length} {link_width} {link_height}"/>
            </geometry>
            <origin xyz="{link_length/2} 0 {link_height/2}" rpy="0 0 0"/>
            <material name="{color}"/>
        </visual>
        <collision>
            <geometry>
                <box size="{link_length} {link_width} {link_height}"/>
            </geometry>
            <origin xyz="{link_length/2} 0 {link_height/2}" rpy="0 0 0"/>
        </collision>
        <inertial>
            <mass value="{current_mass:.3f}"/>
            <origin xyz="{link_length/2} 0 {link_height/2}" rpy="0 0 0"/>
            <inertia ixx="{ixx:.6f}" ixy="0" ixz="0" iyy="{iyy:.6f}" iyz="0" izz="{izz:.6f}"/>
        </inertial>
    </link>
"""
    
    # Add end effector (optional - a small cylinder at the end)
    urdf += f"""
    <!-- End Effector -->
    <joint name="joint{num_links+1}" type="fixed">
        <parent link="link{num_links}"/>
        <child link="end_effector"/>
        <origin xyz="{link_length} 0 {link_height/2}" rpy="0 0 0"/>
    </joint>

    <link name="end_effector">
        <visual>
            <geometry>
                <cylinder radius="{link_width/2}" length="{link_height}"/>
            </geometry>
            <material name="grey"/>
        </visual>
        <collision>
            <geometry>
                <cylinder radius="{link_width/2}" length="{link_height}"/>
            </geometry>
        </collision>
        <inertial>
            <mass value="0.1"/>
            <inertia ixx="0.00001" ixy="0" ixz="0" iyy="0.00001" iyz="0" izz="0.00001"/>
        </inertial>
    </link>
</robot>
"""
    
    # Write to file
    with open(f"Snake/urdf/{file_name}", 'w') as f:
        f.write(urdf)
    
    print(f"Successfully generated URDF file '{file_name}' with {num_links} links")
    print(f"Total length of robot: ~{base_size/2 + (num_links * link_length)}m")
    print(f"Robot height: {link_height}m")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Generate a URDF file for a planar robot with Z-axis revolute joints')
    parser.add_argument('num_links', type=int, help='Number of links for the robot')
    parser.add_argument('--output', '-o', default='planar_robot.urdf', help='Output file name')
    parser.add_argument('--length', '-l', type=float, default=0.2, help='Length of each link (meters)')
    parser.add_argument('--width', '-w', type=float, default=0.05, help='Width of each link (meters)')
    parser.add_argument('--height', type=float, default=0.025, help='Height of each link (meters)')
    parser.add_argument('--base', '-b', type=float, default=0.1, help='Size of the base (meters)')
    parser.add_argument('--mass', '-m', type=float, default=1.0, help='Mass of the base link (kg)')
    parser.add_argument('--plain', action='store_false', dest='colorful', help='Use only grey color for links')
    
    args = parser.parse_args()
    for i in range(10, args.num_links+1, 10):
        generate_planar_robot_urdf(i, f"planar_robot_{i}.urdf", 1/i, 0.4 * (1/i), 
                        args.height, args.base, args.mass, colorful=args.colorful)
        
