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
        os.rename(BASE_DIR + filename, BASE_DIR + newPathName) #renames only if overriding file
    else:
        newPathFile = open(f"{BASE_DIR}{newPathName}", "x")
        newPathFile.write(json.dumps(path))

def convert(path):
    for waypoint in path["waypoints"]:
        waypoint["anchorPoint"]["x"] = 16.5 - waypoint["anchorPoint"]["x"]
        if waypoint["prevControl"] != None:
            waypoint["prevControl"]["x"] = 16.5 - waypoint["prevControl"]["x"] 
        if waypoint["nextControl"] != None:
            waypoint["nextControl"]["x"] = 16.5 - waypoint["nextControl"]["x"]
            
        waypoint["holonomicAngle"] = (180.0 + waypoint["holonomicAngle"])
        if waypoint["holonomicAngle"] > 180.0:
            waypoint["holonomicAngle"] -= 360.0
        waypoint["holonomicAngle"] = (-1 * waypoint["holonomicAngle"]) #had to adjust for subtracting 360

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