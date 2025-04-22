from table import Table
from PIL import Image, ImageDraw, ImageFont, ImageTk
import math

class TableDatabase:
    def __init__(self, tableInfoFile):
        self.tables = []
        with open(tableInfoFile, "r") as tableInfoFile:
            tablesInfo = tableInfoFile.readlines()
        
        i = 0
        for line in tablesInfo:
            info = line.split("/")
            self.tables.append(Table(int(info[0]), [float(info[1]), float(info[2])], int(info[3])))
            if len(info) == 5:
                self.tables[i].addOccupants(int(info[4]))
            i = i + 1
            
        # for table in self.tables:
        #     table.allTableInfo()
    
    def findTableLocation(self, num):
        for table in self.tables:
            if int(table.getTableNum()) == num:
                print(table.getLocation())
                return table.getLocation()
            else:
                continue
    
    def getClosestTable(self, location):
        minDistance = 10000000
        closestTable = 0
        for table in self.tables:
            tableLoc = table.getLocation()
            distance = math.sqrt(math.pow(location[0] - tableLoc[0], 2) + math.pow(location[1] - tableLoc[1], 2))
            if distance < minDistance:
                minDistance = distance
                closestTable = table.getTableNum()
        return closestTable
            

    def getNumofTables(self):
        return len(self.tables)
    
    def minOccupacy(self):
        min = 10000000
        emptiestTable = 0
        for table in self.tables:
            occup = table.getOccupants()
            if occup == 0:
                return table.getTableNum()
            elif occup <  min:
                min = occup
                emptiestTable = table.getTableNum()
        
        return emptiestTable
    
    def generateTableStatusImage(self, width, height):
        
        image = Image.new("RGB", (width, height), "white")
        draw = ImageDraw.Draw(image)

        # Optional: Load a font
        try:
            font = ImageFont.truetype("audio_messages/src/fonts/SpecialGothic-Regular.ttf", 30)
        except:
            font = ImageFont.load_default()
    
        radius = 75
        spacing_x = (127 + (2*radius))
        spacing_y = (120 + (2*radius))
        offset_x = (127 + radius)
        offset_y = (60 + radius)

        for i, table in enumerate(self.tables):
            status = table.getCurrentTableStatus()

            x = offset_x + (i % 3) * spacing_x
            y = offset_y + (i // 3) * spacing_y

            # Circle color
            fill_color = "red" if status[2] else "green"

            # Draw the circle
            draw.ellipse(
                [x - radius, y - radius, x + radius, y + radius],
                fill=fill_color,
                outline="black"
            )

            # Draw table name above
            # draw.text((x - radius, y - radius - 25), f"Table {status[0]}", fill="black", font=font)

            # Draw occupancy inside
            occ_text = f"Table {status[0]}\n   {status[1]}"
            text_size = draw.textsize(occ_text, font=font)
            draw.text(
                (x - text_size[0] // 2, y - text_size[1] // 2),
                occ_text,
                fill="white",
                font=font
            )

        return image

                
        
