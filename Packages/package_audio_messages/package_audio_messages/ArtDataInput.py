from ament_index_python.packages import get_package_share_directory
import os
import ast

class ArtDataInput:

    def __init__(self):
        self.artNameList = []
        self.artLocsList = []
        self.adding = True

        self.collatedNames = ""
        self.collatedLocations = ""

    def main(self):
        print("Welcome to Data Input for the Art Names and Locations for the Gallery Greeter")
        
        while self.adding:
            print("Artwork Name: the name should be the same as the jpg file without .jpg")
            currentArtName = str(input("Artwork Name: "))

            print("Artwork Location")
            artLocX = float(input("X Coordinate: "))
            artLocY = float(input("Y Coordinate: "))
            artLocW = float(input("Orientation: "))

            self.artNameList.append(currentArtName)
            self.artLocsList.append([artLocX, artLocY, artLocW])

            newPainting = input("Do you want to add another painting (Y/N): ")
            if newPainting.lower() == 'n':
                self.adding = False

        for i, artName in enumerate(self.artNameList):
            # print(f"i is: {i}, length is {len(self.artNameList)}")
            if i >= len(self.artNameList)-1:
                self.collatedNames = self.collatedNames + artName
            else:
                self.collatedNames = self.collatedNames + artName + "/"
        
        for i, artLocation in enumerate(self.artLocsList):
            if i >= len(self.artLocsList)-1:
                self.collatedLocations = self.collatedLocations + f"[{artLocation[0]}, {artLocation[1]}, {artLocation[2]}]"
            else:
                self.collatedLocations = self.collatedLocations + f"[{artLocation[0]}, {artLocation[1]}, {artLocation[2]}]" + "/"

        print(self.collatedNames)
        print(self.collatedLocations)

        # package_path = get_package_share_directory('package_audio_messages')
        # resources_path = os.path.join(package_path, 'resources')
        # filepath = os.path.join(resources_path, 'ArtworkInfo', 'artworkInfo.txt')
        
        
        # Get absolute path to the directory of this script
        script_dir = os.path.dirname(os.path.realpath(__file__))

        # Build the path to artworkInfo.txt relative to the script
        filepath = os.path.join(script_dir, 'resources', 'ArtworkInfo', 'artworkinforealcopy.txt')
        print(filepath)

        overwriteResponse = input("Do you want to Overwrite the Existing Data Set (Y/N): ")

        if overwriteResponse.lower() == 'y':
            try:
                with open(filepath, 'r') as file:
                    lines = file.readlines()

                print(lines[0])
                lines[0] = self.collatedNames + '\n'

                print(lines[1])
                lines[1] = self.collatedLocations + '\n'

                with open(filepath, 'w') as file:
                    file.writelines(lines)
                print("Successfully Overwritten Art Data.")

                with open(filepath, 'r') as file:
                    print("File contents after write:")
                    print(file.read())
            except FileNotFoundError:
                print(f"Error: File not found at {filepath}")
            except Exception as e:
                print(f"An error occurred: {e}")
        else:
            try:
                with open(filepath, 'r') as file:
                    lines = file.readlines()

                currentArtNames = lines[0]
                lines[0] = currentArtNames + "/" + self.collatedNames + '\n'

                currentArtLocations = lines[1]
                lines[1] = currentArtLocations + "/" + self.collatedLocations + '\n'

                with open(filepath, 'w') as file:
                    file.writelines(lines)
                print("Successfully Overwritten Art Data.")

                with open(filepath, 'r') as file:
                    print("File contents after write:")
                    print(file.read())
            except FileNotFoundError:
                print(f"Error: File not found at {filepath}")
            except Exception as e:
                print(f"An error occurred: {e}")

def main():
    obj = ArtDataInput()
    obj.main()

if __name__ == "__main__":
    main()
