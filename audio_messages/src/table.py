class Table:
    def __init__(self, number, location, maxCapacity=4):
        self.tableNum = number
        self.location = location
        self.maxCapacity = maxCapacity
        self.occupants = 0
        self.awaitingDrink = False

    def addOccupants(self, amount=1):
        self.occupants = self.occupants + amount

    def removeOccupants(self, amount=1):
        self.occupants = self.occupants - amount

    def clearTable(self):
        self.occupants = 0

    def setDrinkStatus(self, state):
        self.awaitingDrink = state
    
    def getCurrentTableStatus(self):
        return [self.tableNum, self.occupants, self.awaitingDrink]
    
    def allTableInfo(self):
        print(f"Table #{self.tableNum} at {self.location} has {self.occupants}/{self.maxCapacity} and waiting for drink is {self.awaitingDrink}")
        return [self.tableNum, self.location, self.maxCapacity, self.occupants, self.awaitingDrink]