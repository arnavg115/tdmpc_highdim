#!/usr/bin/env python3

import argparse
import os
import xml.dom.minidom
import xml.etree.ElementTree as ET
from xml.dom import minidom

def generate_srdf(urdf_file, output_file="robot.srdf", disable_collisions_pairs=None,
                 create_planning_groups=True, add_end_effector=True, add_default_poses=True):
    """
    Generate an SRDF file for a robot based on a URDF file.
    
    Args:
        urdf_file (str): Path to the input URDF file
        output_file (str): Path to the output SRDF file
        disable_collisions_pairs (list): List of tuples of link pairs to disable collisions between
        create_planning_groups (bool): Whether to create planning groups
        add_end_effector (bool): Whether to add end effector information
        add_default_poses (bool): Whether to add default poses
    """
    try:
        # Parse the URDF to extract robot name and links
        urdf_tree = ET.parse(urdf_file)
        urdf_root = urdf_tree.getroot()
        robot_name = urdf_root.get('name')
        
        # Get all links and joints
        links = [link.get('name') for link in urdf_root.findall('.//link')]
        joints = {joint.get('name'): {
            'parent': joint.find('parent').get('link'),
            'child': joint.find('child').get('link'),
            'type': joint.get('type')
        } for joint in urdf_root.findall('.//joint')}
        
        # Create SRDF structure
        srdf_root = ET.Element('robot', name=robot_name)
        
        # Add groups if requested
        if create_planning_groups:
            add_planning_groups(srdf_root, links, joints)
        
        # Add end effector if requested
        if add_end_effector and 'end_effector' in links:
            add_end_effector_group(srdf_root, links, joints)
        
        # Add default poses if requested
        if add_default_poses:
            add_robot_poses(srdf_root, joints)
        
        # Add disabled collisions
        add_disabled_collisions(srdf_root, links, joints, disable_collisions_pairs)
        
        # Write to file with pretty formatting
        xmlstr = minidom.parseString(ET.tostring(srdf_root)).toprettyxml(indent="  ")
        with open(output_file, "w") as f:
            f.write(xmlstr)
            
        print(f"Successfully generated SRDF file: {output_file}")
        
    except Exception as e:
        print(f"Error generating SRDF: {e}")

def add_planning_groups(srdf_root, links, joints):
    """Add planning groups to the SRDF."""
    # Determine if there are enough links to create multiple groups
    revolute_joints = [name for name, j in joints.items() if j['type'] == 'revolute']
    
    # Create arm group
    arm_group = ET.SubElement(srdf_root, 'group', name="arm")
    
    # Add all revolute joints to the arm group
    for joint_name in revolute_joints:
        ET.SubElement(arm_group, 'joint', name=joint_name)
    
    # If we have enough links, create an upper_arm and forearm group
    if len(revolute_joints) >= 6:
        mid_point = len(revolute_joints) // 2
        
        # Create upper arm group (first half of joints)
        upper_arm = ET.SubElement(srdf_root, 'group', name="upper_arm")
        for joint_name in revolute_joints[:mid_point]:
            ET.SubElement(upper_arm, 'joint', name=joint_name)
        
        # Create forearm group (second half of joints)
        forearm = ET.SubElement(srdf_root, 'group', name="forearm")
        for joint_name in revolute_joints[mid_point:]:
            ET.SubElement(forearm, 'joint', name=joint_name)

def add_end_effector_group(srdf_root, links, joints):
    """Add end effector information to the SRDF."""
    # Find the parent of the end_effector link
    end_effector_parent = None
    for joint_name, joint_info in joints.items():
        if joint_info['child'] == 'end_effector':
            end_effector_parent = joint_info['parent']
            end_effector_joint = joint_name
            break
    
    if end_effector_parent:
        # Create end effector definition
        ee = ET.SubElement(srdf_root, 'end_effector', 
                           name="end_effector", 
                           parent_link=end_effector_parent,
                           group="arm",
                           parent_group="arm")

def add_robot_poses(srdf_root, joints):
    """Add default poses to the SRDF."""
    # Create home pose (all joints at 0)
    home_pose = ET.SubElement(srdf_root, 'group_state', name="home", group="arm")
    for joint_name, joint_info in joints.items():
        if joint_info['type'] == 'revolute':
            ET.SubElement(home_pose, 'joint', name=joint_name, value="0")
    
    # Create tuck pose (all joints at half their range, alternating positive/negative)
    tuck_pose = ET.SubElement(srdf_root, 'group_state', name="tuck", group="arm")
    for i, (joint_name, joint_info) in enumerate(joints.items()):
        if joint_info['type'] == 'revolute':
            # Alternate between positive and negative values
            value = "0.5" if i % 2 == 0 else "-0.5"
            ET.SubElement(tuck_pose, 'joint', name=joint_name, value=value)
    
    # Create extended pose (arm stretched out)
    extended_pose = ET.SubElement(srdf_root, 'group_state', name="extended", group="arm")
    for joint_name, joint_info in joints.items():
        if joint_info['type'] == 'revolute':
            # First joint (base) at 0, all others at 0 (stretched out)
            value = "0"
            ET.SubElement(extended_pose, 'joint', name=joint_name, value=value)

def add_disabled_collisions(srdf_root, links, joints, additional_pairs=None):
    """Add disabled collision pairs to the SRDF."""
    # Disable collisions between adjacent links
    for joint_name, joint_info in joints.items():
        parent = joint_info['parent']
        child = joint_info['child']
        ET.SubElement(srdf_root, 'disable_collisions', 
                      link1=parent, link2=child, 
                      reason="Adjacent")
    
    # Disable collisions for links that are far apart in the kinematic chain
    chain_positions = {}
    
    # Calculate position in the chain for each link
    def get_chain_position(link_name, position=0):
        if link_name in chain_positions:
            return
        chain_positions[link_name] = position
        # Find children
        for _, joint_info in joints.items():
            if joint_info['parent'] == link_name:
                get_chain_position(joint_info['child'], position + 1)
    
    # Start with the base link
    base_link = next((link for link in links if link == 'base_link'), links[0])
    get_chain_position(base_link)
    
    # Disable collisions between links that are far apart
    for link1 in links:
        for link2 in links:
            if link1 != link2 and link1 in chain_positions and link2 in chain_positions:
                # If links are more than 3 steps apart, disable collision
                if abs(chain_positions[link1] - chain_positions[link2]) > 3:
                    ET.SubElement(srdf_root, 'disable_collisions', 
                                  link1=link1, link2=link2, 
                                  reason="Never")
    
    # Add additional disabled collision pairs if specified
    if additional_pairs:
        for link1, link2 in additional_pairs:
            ET.SubElement(srdf_root, 'disable_collisions', 
                          link1=link1, link2=link2, 
                          reason="User defined")

def find_urdf_files(directory="."):
    """Find all URDF files in the given directory."""
    urdf_files = []
    for file in os.listdir(directory):
        if file.endswith(".urdf"):
            urdf_files.append(os.path.join(directory, file))
    return urdf_files

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Generate an SRDF file from a URDF file')
    parser.add_argument('urdf_file', nargs='?', help='Input URDF file')
    parser.add_argument('--output', '-o', help='Output SRDF file')
    parser.add_argument('--no-planning-groups', action='store_false', dest='planning_groups', 
                        help='Do not create planning groups')
    parser.add_argument('--no-end-effector', action='store_false', dest='end_effector',
                        help='Do not add end effector information')
    parser.add_argument('--no-default-poses', action='store_false', dest='default_poses',
                        help='Do not add default poses')
    args = parser.parse_args()
    
    # If no URDF file is specified, look for URDF files in the current directory
    if not args.urdf_file:
        urdf_files = find_urdf_files()
        if not urdf_files:
            print("No URDF files found in the current directory.")
            exit(1)
        elif len(urdf_files) == 1:
            args.urdf_file = urdf_files[0]
        else:
            print("Multiple URDF files found. Please specify which one to use:")
            for i, file in enumerate(urdf_files):
                print(f"{i+1}. {file}")
            choice = input("Enter the number of the file to use: ")
            try:
                choice = int(choice) - 1
                if 0 <= choice < len(urdf_files):
                    args.urdf_file = urdf_files[choice]
                else:
                    print("Invalid choice. Exiting.")
                    exit(1)
            except ValueError:
                print("Invalid input. Exiting.")
                exit(1)
    
    # If no output file is specified, create one based on the input file
    if not args.output:
        base_name = os.path.splitext(args.urdf_file)[0]
        args.output = f"{base_name}.srdf"
    
    # Generate the SRDF
    generate_srdf(args.urdf_file, args.output, 
                 create_planning_groups=args.planning_groups,
                 add_end_effector=args.end_effector,
                 add_default_poses=args.default_poses)