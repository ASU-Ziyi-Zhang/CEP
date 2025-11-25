import xml.etree.ElementTree as ET
import math
from typing import List, Dict, Tuple

def set_edge_lengths_and_shift_lights(
    net_file: str,
    output_file: str,
    edge_new_lengths: Dict[str, float]
) -> None:
    """
    Adjusts the length of specified edges to the user-defined values and repositions
    any traffic-light junctions that lie at the end of those edges (the 'to' junction).

    This script assumes each edge is (mostly) straight, so that modifying the final
    point of the lane shape is enough to match the new length. For multi-segment or
    curved edges, you need a more advanced geometry re-scaling routine.

    :param net_file: Path to the SUMO .net.xml file.
    :param output_file: Path to write the updated .net.xml file.
    :param edge_new_lengths: Dictionary mapping edge_id -> new_length (float).
    """

    tree = ET.parse(net_file)
    root = tree.getroot()

    # -------------------------------------------------------------------------
    # 1. Build quick references for edges and junctions
    # -------------------------------------------------------------------------
    edges_by_id = {}
    junctions_by_id = {}
    for edge in root.findall('edge'):
        e_id = edge.get('id')
        if e_id is not None:
            edges_by_id[e_id] = edge

    for jun in root.findall('junction'):
        j_id = jun.get('id')
        if j_id is not None:
            junctions_by_id[j_id] = jun

    # -------------------------------------------------------------------------
    # 2. Function to parse a lane shape string into a list of (x,y) floats
    # -------------------------------------------------------------------------
    def parse_shape(shape_str: str) -> List[Tuple[float, float]]:
        pts = []
        if not shape_str:
            return pts
        for coordpair in shape_str.strip().split():
            px, py = coordpair.split(',')
            pts.append((float(px), float(py)))
        return pts

    # 3. A helper to convert a list of (x,y) floats back to shape="x1,y1 x2,y2 ..."
    def shape_to_str(pts: List[Tuple[float, float]]) -> str:
        return " ".join(f"{p[0]:.2f},{p[1]:.2f}" for p in pts)

    # -------------------------------------------------------------------------
    # 4. For each edge in edge_new_lengths: adjust geometry to the new length
    # -------------------------------------------------------------------------
    for e_id, new_len in edge_new_lengths.items():
        edge_elem = edges_by_id.get(e_id)
        if edge_elem is None:
            continue  # skip if not found in the file

        old_len_str = edge_elem.get('length')
        if not old_len_str:
            # If no length attribute is present, skip or handle as needed
            continue

        old_len = float(old_len_str)

        # For simplicity, we do a *straight-line approach*:
        #   - parse the shape of each lane, assume it is 2 points or (start, end)
        #   - set the end point so that the total length is new_len
        #   - update <edge length="..."> to new_len

        # Also, we likely need to shift the 'to' junction if the geometry end changes
        to_junction_id = edge_elem.get('to')
        from_junction_id = edge_elem.get('from')

        # We'll store how much we shift the final point
        # (dx, dy) = shift needed for that edge's endpoint
        # We'll compute it once from lane0 and reuse for other lanes & for the junction
        shift_dx = 0.0
        shift_dy = 0.0

        # 4a. Adjust <edge length="...">
        edge_elem.set('length', f"{new_len:.2f}")

        # 4b. For each lane, parse shape and scale if it has a start & end
        for lane in edge_elem.findall('lane'):
            shape_attr = lane.get('shape')
            if not shape_attr:
                # If there's no shape, skip or you might want to create one
                continue

            pts = parse_shape(shape_attr)
            if len(pts) < 2:
                # Not enough points to recast a new length
                continue

            # We'll assume the first point is the start, last point is the end
            (x0, y0) = pts[0]
            (xN, yN) = pts[-1]
            old_dx = xN - x0
            old_dy = yN - y0
            old_dist = math.sqrt(old_dx**2 + old_dy**2)
            if old_dist < 1e-6:
                continue

            # scale factor to get from old_dist to new_len
            scale = new_len / old_dist
            new_dx = old_dx * scale
            new_dy = old_dy * scale

            # The shift from the old end to the new end
            lane_shift_dx = new_dx - old_dx
            lane_shift_dy = new_dy - old_dy

            # If we haven't computed shift_dx, shift_dy yet (we just pick from lane 0)
            if shift_dx == 0.0 and shift_dy == 0.0:
                shift_dx = lane_shift_dx
                shift_dy = lane_shift_dy

            # Reconstruct the shape:
            #   For a simple 2-point shape, we just adjust the last point
            #   For multi-point shape, you might want to scale all intermediate points
            #   but let's keep it simple and only shift the final point
            x1 = x0 + new_dx
            y1 = y0 + new_dy
            pts[-1] = (x1, y1)

            lane.set('shape', shape_to_str(pts))

        # 4c. If there's a 'to' junction, shift it by (shift_dx, shift_dy)
        #     This ensures the traffic light or the dead-end junction matches the new end
        if to_junction_id and (abs(shift_dx) > 1e-9 or abs(shift_dy) > 1e-9):
            jun = junctions_by_id.get(to_junction_id)
            if jun is not None:
                # shift the junction x, y
                old_jx = float(jun.get('x', "0.0"))
                old_jy = float(jun.get('y', "0.0"))
                new_jx = old_jx + shift_dx
                new_jy = old_jy + shift_dy
                jun.set('x', f"{new_jx:.2f}")
                jun.set('y', f"{new_jy:.2f}")

                # If it's a traffic light, we might also update the shape polygon
                shape_str = jun.get('shape')
                if shape_str:
                    shape_pts = parse_shape(shape_str)
                    # shift all points in shape
                    for i, (px, py) in enumerate(shape_pts):
                        shape_pts[i] = (px + shift_dx, py + shift_dy)
                    jun.set('shape', shape_to_str(shape_pts))

                # Optionally, do the same if there's an <intLanes> connection shape, etc.

    # -------------------------------------------------------------------------
    # 5. Write out the updated net file
    # -------------------------------------------------------------------------
    tree.write(output_file)
    print(f"Updated edges and traffic-light positions saved to: {output_file}")


if __name__ == "__main__":
    # Example usage:
    #
    # Suppose you want to set 'gneE0' to length=200.0, and 'gneE1' to 350.0
    # The dictionary is edge_id -> new_length

    edge_lengths_example = {
        "gneE0": 200.0,
        "gneE1": 350.0
    }
    
    set_edge_lengths_and_shift_lights(
        net_file="example.net.xml",
        output_file="example_modified.net.xml",
        edge_new_lengths=edge_lengths_example
    )

    # # For wchi_ave.net.xml, do the same but with whichever edges & new lengths you desire.
    # wchi_edge_lengths = {
    #     "261114939": 280.0,
    #     "261115163": 150.0,
    # }
    # set_edge_lengths_and_shift_lights(
    #     net_file="wchi_ave.net.xml",
    #     output_file="wchi_ave_modified.net.xml",
    #     edge_new_lengths=wchi_edge_lengths
    # )
