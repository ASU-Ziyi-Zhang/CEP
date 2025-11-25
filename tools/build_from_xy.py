#######################################
### Tyler Ard                       ###
### Vehicle Mobility Systems Group  ###
### tard(at)anl(dot)gov             ###
#######################################

import xml.etree.ElementTree as ET
import os

def create_node_file(nodes, filename="myroad.nod.xml"):
    root = ET.Element("nodes")
    
    for i, (x, y) in enumerate(nodes):
        node = ET.SubElement(
            root, "node", id=f"n{i+1}", 
            x=str(x), y=str(y)
            )
    
    tree = ET.ElementTree(root)
    tree.write(filename)

    print(f"Node file '{filename}' generated!")

def create_edge_file(nodes, filename="myroad.edg.xml"):
    root = ET.Element("edges")
    
    for i in range(0, len(nodes) - 1):
        edge = ET.SubElement(
            root, "edge", id=f"e{i+1}", 
            from=f"n{i+1}", to=f"n{i+2}", 
            numLanes="2", speed="50"
            )
    
    tree = ET.ElementTree(root)
    tree.write(filename)
    print(f"Edge file '{filename}' generated!")

def generate_sumo_network():
    os.system("netconvert --node-files=myroad.nod.xml --edge-files=myroad.edg.xml --output-file=myroad.net.xml")
    print("SUMO network file 'myroad.net.xml' generated!")

if __name__ == '__main__':
    # Sample coordinates
    coordinates = [(10, 20), (50, 100), (80, 150)]  # Example road points

    # Run the conversion process
    create_node_file(coordinates)
    create_edge_file(coordinates)
    generate_sumo_network()