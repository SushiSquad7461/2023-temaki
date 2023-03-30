import json
import os

BASE_DIR = "src/main/deploy/pathplanner/"
colorToConvert = input("enter the color to convert files to (Red or Blue?): ").lower()
print(colorToConvert)
if (colorToConvert == "red"):
    print("converting to red")
else:
    print("converting to blue")

folder = os.fsencode(BASE_DIR)

def write_file(overrideFile):
    if overrideFile == ("y"):
        newPathFile = open(f"{BASE_DIR}{filename}", 'w')
        newPathFile.write(json.dumps(path))
    else:
        newPathFile = open(f"{BASE_DIR}{newPathName}", "x")
        newPathFile.write(json.dumps(path))

def convert(path):
    if (colorToConvert == "red"):
        converter = -1
    else: 
        converter = 1

    for waypoint in path["waypoints"]:
        waypoint["anchorPoint"]["x"] = 16.5 + waypoint["anchorPoint"]["x"] * converter
        if waypoint["prevControl"] != None:
            waypoint["prevControl"]["x"] = 16.5 + waypoint["prevControl"]["x"] * converter
        if waypoint["nextControl"] != None:
            waypoint["nextControl"]["x"] = 16.5 + waypoint["nextControl"]["x"] * converter
        waypoint["holonomicAngle"] = 180.0 + waypoint["holonomicAngle"] * converter
        if waypoint["holonomicAngle"] > 180.0:
            waypoint["holonomicAngle"] -= 360.0

for file in os.listdir(folder):
    filename = os.fsdecode(file)
    path = json.load(open(BASE_DIR+filename))
    if (colorToConvert == "red"):
        if not filename.startswith("Red_"):
            convert(path)
            newPathName = "Red_" + filename
            overrideFile = input("override file: " + filename + "(y/n)? ")
            write_file(overrideFile)

    else:
        if (filename.startswith("Red_")):
            convert(path)
            newPathName = filename[4:]
            overrideFile = input("override file: " + filename + "(y/n)? ")
            write_file(overrideFile)