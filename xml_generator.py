# Build an xml file for the scene of exercise 3 

def xml_file():
    import xml.etree.ElementTree as ET
    import xml.dom.minidom

    root = ET.Element("mujoco", model="scene")

    # Header elements
    ET.SubElement(root, "compiler", angle="radian", autolimits="true")
    ET.SubElement(root, "option", timestep="0.002", integrator="implicitfast", solver="Newton", gravity="0 0 -9.82", cone="elliptic")

    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", azimuth="120", elevation="-20")
    ET.SubElement(visual, "rgba", haze="0.15 0.25 0.35 1")

    ET.SubElement(root, "statistic", meansize="0.1", extent="1", center="4 0 0")
    ET.SubElement(root, "include", file="assets/UR5eGripper/ur5e2f85.xml")

    # Asset section
    asset = ET.SubElement(root, "asset")
    ET.SubElement(asset, "mesh", name="table_mesh", file="assets/table/sigmund_table.stl")
    ET.SubElement(asset, "texture", type="skybox", builtin="gradient", rgb1="0.3 0.5 0.7", rgb2="0 0 0", width="512", height="3072")
    ET.SubElement(asset, "texture", type="2d", name="groundplane", builtin="checker", mark="edge", rgb1="0.2 0.3 0.4", rgb2="0.1 0.2 0.3", markrgb="0.8 0.8 0.8", width="300", height="300")
    ET.SubElement(asset, "material", name="groundplane", texture="groundplane", texuniform="true", texrepeat="5 5", reflectance="0.2")

    # Worldbody section
    worldbody = ET.SubElement(root, "worldbody")
    # Floor and light
    ET.SubElement(worldbody, "geom", name="floor", pos="0 0 -0.74", size="0 0 0.05", type="plane", material="groundplane")
    ET.SubElement(worldbody, "light", pos="0 0 0.5", dir="0 0 -1", type="directional", range="100", bulbradius="0.1")

    # Tables
    ET.SubElement(worldbody, "geom", name="table_1", pos="0.801 -0.6 0", type="mesh", rgba="0.1 0.1 0.1 1", mesh="table_mesh", condim="6", solref="0.001 2", solimp="0.99 0.999 0.001")
    ET.SubElement(worldbody, "geom", name="table_2", pos="0 -0.6 0", type="mesh", rgba="0.1 0.1 0.1 1", mesh="table_mesh", condim="6", solref="0.001 2", solimp="0.99 0.999 0.001")

    # Cylinders (2 rows x 5 columns)
    base_xs = [0.45, 0.65]
    base_ys = [-0.4, -0.2, 0, 0.2, 0.4]
    for i, x in enumerate(base_xs):
        for j, y in enumerate(base_ys):
            idx = i * 5 + j + 1
            body = ET.SubElement(worldbody, "body", name=f"cylinder{idx}", pos=f"{x} {y} 0.1", quat="1 0 0 0")
            ET.SubElement(body, "geom", name=f"cylinder{idx}", type="cylinder", size="0.035 0.1", rgba="1 1 0 1", contype="0", conaffinity="0")

    # Zones
    zone_pickup = ET.SubElement(worldbody, "body", name="zone_pickup", pos="0.6 0 -0.01", quat="1 0 0 0")
    ET.SubElement(zone_pickup, "geom", name="zone_pickup", type="box", size="0.2 0.6 0.01", rgba="0 1 0 0.1", contype="0", conaffinity="0")
    zone_drop = ET.SubElement(worldbody, "body", name="zone_drop", pos="-0.6 0 -0.01", quat="1 0 0 0")
    ET.SubElement(zone_drop, "geom", name="zone_drop", type="box", size="0.2 0.6 0.01", rgba="1 0 0 0.1", contype="0", conaffinity="0")

    # Save to file with pretty print
    xml_str = ET.tostring(root, encoding='utf-8')
    parsed = xml.dom.minidom.parseString(xml_str)
    name = "scene_gen.xml"
    with open(name, "w", encoding="utf-8") as f:
        f.write(parsed.toprettyxml(indent="    "))
    return name