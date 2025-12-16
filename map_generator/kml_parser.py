import os

from lxml import etree as ET


def parse_kml_file(file_path):
    parser = ET.XMLParser(encoding="utf-8")
    try:
        tree = ET.parse(file_path, parser=parser)
    except ET.ParseError as e:
        raise Exception("Error parsing KML file") from e
    root = tree.getroot()
    return parse_kml_tree(root)


def parse_kml_tree(root):
    placemarks_data = {
        "NFZs": {"polygons": [], "paths": []},
        "ROIs": {"polygons": [], "paths": []},
        "EMZs": {"polygons": [], "paths": []},
        "home": {"lat": None, "lng": None},
    }

    # KML namespace
    ns = {"kml": "http://www.opengis.net/kml/2.2"}

    # Now, find all Folders
    folders = root.findall(".//kml:Folder", ns)
    for folder in folders:
        folder_name_elem = folder.find("kml:name", ns)
        if folder_name_elem is not None:
            folder_name = folder_name_elem.text.strip()
            placemarks = folder.findall(".//kml:Placemark", ns)
            if folder_name == "NFZs":
                placemarks_data["NFZs"] = extract_zones(placemarks, ns)
            elif folder_name == "ROIs":
                placemarks_data["ROIs"] = extract_zones(placemarks, ns)
            elif folder_name == "EMZs":
                placemarks_data["EMZs"] = extract_zones(placemarks, ns)
            elif folder_name == "Home":
                placemarks_data["home"] = extract_zones(placemarks, ns)
    return placemarks_data


def extract_zones(placemarks, ns):
    zones = {"polygons": [], "paths": []}
    for placemark in placemarks:
        name_elem = placemark.find("kml:name", ns)
        visibility_elem = placemark.find("kml:visibility", ns)
        visibility = "1"  # Default visibility
        if visibility_elem is not None:
            visibility = visibility_elem.text.strip()
        # Skip placemarks with visibility 0
        if visibility == "0":
            continue
        placemark_name = name_elem.text.strip() if name_elem is not None else ""
        # Get altitude from LookAt/altitude
        altitude = 0
        lookat_elem = placemark.find(".//kml:LookAt", ns)
        if lookat_elem is not None:
            altitude_elem = lookat_elem.find("kml:altitude", ns)
            if altitude_elem is not None:
                altitude = float(altitude_elem.text.strip())

        # Polygons
        polygons = placemark.findall(".//kml:Polygon", ns)
        for polygon in polygons:
            coords_elem = polygon.find(".//kml:coordinates", ns)
            if coords_elem is not None:
                coords_text = coords_elem.text.strip()
                points = coords_text.split()
                google_polygon_paths = []
                for point in points:
                    coord = point.split(",")
                    if len(coord) >= 2:
                        lng = float(coord[0])
                        lat = float(coord[1])
                        google_polygon_paths.append({"lat": lat, "lng": lng})
                zones["polygons"].append(
                    {
                        "name": placemark_name,
                        "altitude": altitude,
                        "coords": google_polygon_paths,
                    }
                )

        # Paths (LineString)
        linestrings = placemark.findall(".//kml:LineString", ns)
        for linestring in linestrings:
            coords_elem = linestring.find(".//kml:coordinates", ns)
            if coords_elem is not None:
                coords_text = coords_elem.text.strip()
                points = coords_text.split()
                google_linestring_paths = []
                for point in points:
                    coord = point.split(",")
                    if len(coord) >= 2:
                        lng = float(coord[0])
                        lat = float(coord[1])
                        google_linestring_paths.append({"lat": lat, "lng": lng})
                zones["paths"].append(
                    {
                        "name": placemark_name,
                        "altitude": altitude,
                        "coords": google_linestring_paths,
                    }
                )
    return zones


def extract_home_location(placemarks, ns):
    home = {"lat": None, "lng": None}
    for placemark in placemarks:
        name_elem = placemark.find("kml:name", ns)
        visibility_elem = placemark.find("kml:visibility", ns)
        visibility = "1"  # Default visibility
        if visibility_elem is not None:
            visibility = visibility_elem.text.strip()
        # Skip placemarks with visibility 0
        if visibility == "0":
            continue
        placemark_name = name_elem.text.strip() if name_elem is not None else ""
        # Get Point coordinates
        points = placemark.findall(".//kml:Point", ns)
        for point in points:
            coords_elem = point.find(".//kml:coordinates", ns)
            if coords_elem is not None:
                coords_text = coords_elem.text.strip()
                coord = coords_text.split(",")
                if len(coord) >= 2:
                    lng = float(coord[0])
                    lat = float(coord[1])
                    if "home" in placemark_name.lower():
                        home = {"lat": lat, "lng": lng}
    return home
