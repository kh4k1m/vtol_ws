import os
import yaml
import xml.etree.ElementTree as ET

def main():
    config_path = os.path.expanduser('~/vtol_ws/config/marker_positions.yaml')
    input_sdf_path = os.path.expanduser('~/SITL_Models/Gazebo/worlds/alti_transition_runway_cam.sdf')
    output_sdf_path = os.path.expanduser('~/SITL_Models/Gazebo/worlds/alti_transition_runway_python.sdf')

    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    tree = ET.parse(input_sdf_path)
    root = tree.getroot()
    world = root.find('world')

    # Remove old apriltag_unified_pad include
    for include in world.findall('include'):
        uri = include.find('uri')
        if uri is not None and uri.text == 'model://apriltag_unified_pad':
            world.remove(include)

    # Create new model for markers
    model = ET.Element('model', name='custom_apriltag_pad')
    static = ET.SubElement(model, 'static')
    static.text = 'true'
    
    # We will place the model at the center of the world
    pose = ET.SubElement(model, 'pose')
    pose.text = '0 0 0 0 0 0'

    link = ET.SubElement(model, 'link', name='pad_link')

    # Background
    bg_visual = ET.SubElement(link, 'visual', name='background')
    bg_pose = ET.SubElement(bg_visual, 'pose')
    bg_pose.text = '0 0 0.0002 0 0 0'
    bg_cast = ET.SubElement(bg_visual, 'cast_shadows')
    bg_cast.text = 'false'
    bg_geom = ET.SubElement(bg_visual, 'geometry')
    bg_plane = ET.SubElement(bg_geom, 'plane')
    bg_normal = ET.SubElement(bg_plane, 'normal')
    bg_normal.text = '0 0 1'
    bg_size = ET.SubElement(bg_plane, 'size')
    bg_size.text = '10.0 10.0'
    bg_mat = ET.SubElement(bg_visual, 'material')
    bg_light = ET.SubElement(bg_mat, 'lighting')
    bg_light.text = 'false'
    bg_amb = ET.SubElement(bg_mat, 'ambient')
    bg_amb.text = '1 1 1 1'
    bg_diff = ET.SubElement(bg_mat, 'diffuse')
    bg_diff.text = '1 1 1 1'
    bg_spec = ET.SubElement(bg_mat, 'specular')
    bg_spec.text = '0 0 0 1'

    # Add tags
    for category, data in config['markers'].items():
        size_m = data['size'] / 1000.0
        for tag_name, pos_mm in data['positions'].items():
            x_m = pos_mm[0] / 1000.0
            y_m = pos_mm[1] / 1000.0
            
            visual = ET.SubElement(link, 'visual', name=tag_name)
            v_pose = ET.SubElement(visual, 'pose')
            v_pose.text = f'{x_m} {y_m} 0.0004 0 0 0'
            v_cast = ET.SubElement(visual, 'cast_shadows')
            v_cast.text = 'false'
            
            geom = ET.SubElement(visual, 'geometry')
            plane = ET.SubElement(geom, 'plane')
            normal = ET.SubElement(plane, 'normal')
            normal.text = '0 0 1'
            size = ET.SubElement(plane, 'size')
            size.text = f'{size_m} {size_m}'
            
            mat = ET.SubElement(visual, 'material')
            light = ET.SubElement(mat, 'lighting')
            light.text = 'false'
            amb = ET.SubElement(mat, 'ambient')
            amb.text = '1 1 1 1'
            diff = ET.SubElement(mat, 'diffuse')
            diff.text = '1 1 1 1'
            spec = ET.SubElement(mat, 'specular')
            spec.text = '0 0 0 1'
            
            pbr = ET.SubElement(mat, 'pbr')
            metal = ET.SubElement(pbr, 'metal')
            albedo = ET.SubElement(metal, 'albedo_map')
            albedo.text = f'model://apriltag_unified_pad/materials/textures/{tag_name}.png'
            rough = ET.SubElement(metal, 'roughness')
            rough.text = '0.8'
            metalness = ET.SubElement(metal, 'metalness')
            metalness.text = '0.1'

    world.append(model)
    
    # Prettify and write
    ET.indent(tree, space="  ", level=0)
    tree.write(output_sdf_path, encoding='utf-8', xml_declaration=True)
    print(f"Successfully generated {output_sdf_path}")

if __name__ == '__main__':
    main()
