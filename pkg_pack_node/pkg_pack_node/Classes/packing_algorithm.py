import time
import itertools
from Classes.genetic_algorithm import Genetic_Algorithm
from Classes.solution import Solution
import copy
import pandas as pd
from Classes.order_data_transfer import order_data_transfer

# Datenvorverarbeitung
def data_preparation (order_file_name, container_file_name):
    
    '''Importiere  Bestelldaten und Containerdaten'''
    
    # Lese Containerdaten ein
    # container_list = pd.read_excel(container_file_name, sheet_name = "Container", skiprows = 1)
    
    # Erzeuge eine Liste für die Containertypen und speichere die relevanten Informationen darin ab
    container_data = []
    # for i in range(len(container_list)):
    #     container_type = (container_list["Länge innen [mm]"][i],container_list["Breite innen [mm]"][i],container_list["Höhe innen [mm]"][i], 
    #         round(container_list["Fixkosten gesamt"][i],2), round(container_list["Max.Gewicht [kg]"][i],2))
    #     container_data.append(container_type)

    container_data = [(585,392,188,10,30)] # --> Containerdaten manuell eingegeben

    # Lese die Bestelldaten ein
    # order_list = pd.read_excel(order_file_name)
    
    # # Erzeuge eine Liste für die Packstücke der Bestellung und speichere die relevanten Informationen darin ab
    # start = 0 # Hilfsvariable für Tabellenindex
    # end = start + 1 # Hilfsvariable für Tabellenindex
    order_data = [] 
    # for i in range(0, len(order_list)):
    #     packaging_unit = (int(order_list[start:end]["Länge [mm]"]),int(order_list[start:end]["Breite [mm]"]), int(order_list[start:end]["Höhe [mm]"]), float(order_list[start:end]["Gewicht [kg]"]), 
    #                       str(order_list["Orientierungen"][start]))
    #     order_data.append(packaging_unit)
    #     start = start + 1
    #     end = end + 1

    order_data = order_data_transfer.get_order_data()
    '''Bestelldaten und Containerdaten importiert'''
    
    # Entferne Label aus Liste
    order_data = [item[1:] for item in order_data]

    '''Erzeuge die modifizierte Bestelldaten'''

    modified_order_data = [] # Liste zur Speicherung der modifizierten Bestelldaten
    ID = 0 # Fortlaufende Variable zur Zuweisung der Packstück ID
    
    # Durchlaufe die Packstücke aus den Bestelldaten
    for i in range(len(order_data)):
        
            # Rufe die Packstückdimensionen ab
            unit_dimensions = (order_data[i][0],order_data[i][1],order_data[i][2]) 
            # Sortiere die Packstückdimensionen absteigend
            sorted_unit_dimensions = sorted(unit_dimensions, reverse = True)
            # Erzeuge sämtliche Orientierungsmöglichkeiten
            all_orientations = list(itertools.permutations(sorted_unit_dimensions)) 
            
            # Berücksichtige nun die Orientierungsrestriktionen und entferne verbotene Orientierungen            
            # Rufe zunächst die zugelassenen Orientierungen ab
            orientations_permitted = list(order_data[i][4]) 
            # Erzeuge eine Liste, in der die Indizes der verbotenen Orientierungen abgespeichert werden
            index_to_drop = [] 
            for j in range(1, 7): # Suche nach Kennziffern 1 - 6 
                if str(j) not in orientations_permitted: # Wenn Kennziffer nicht erlaubt, dann füge den Index der Liste hinzu
                    index_to_drop.append((j-1)) #Kennziffer 1 steht an Position 0. Daher j - 1.
            # Sortiere jetzt die Indizes die zu entfernen sind
            index_to_drop = sorted(index_to_drop, reverse = True)
            # Entferne  nun die verbotenen Orientierungen
            for itd in index_to_drop:
                all_orientations.pop(itd) 
            
            # Erzeuge ein modifiziertes Packstücke MPS
            MPS = [] 
            
            ID = i + 1 # Gebe Packstücken eine eindeutige ID. Dies ermöglicht eine Unterscheidung von identischen Packstücken.
            for orientation in all_orientations:
                area = (orientation[0] * orientation[1]) / 100 # Errechne die Grundfläche einer Orientierung
                volume = (orientation[0] * orientation[1] * orientation [2]) / 1000 # Errechne das Packstückvolumen
                # Füge dem MPS für jede zugelassene Orientierung folgende Informationen hinzu: 
                # Grundfläche, Packstück ID, Länge, Breite, Höhe, Gewicht, Volumen
                MPS_position = (area, ID, orientation[0], orientation[1], orientation[2], order_data[i][3], round(volume, 2))
                MPS.append(MPS_position)

            # Füge das MPS den modfizierten Bestelldaten hinzu
            modified_order_data.append(MPS)
            
    # Sortiere jedes MPS absteigend nach der Grundfläche der Orientierungsvarianten
    for i, MPS in enumerate(modified_order_data):
        modified_order_data[i] = sorted(MPS, key=lambda elem: elem[0], reverse = True) 
    
 
    # Sortiere die MPS nach Volumen
    sorted_modified_order_data = [] # Liste in der die sortierten MPS gespeichert werden
    volume_index_old = [] # Volumen und aktuellen Index aller MPS der unsortierten modifizierten Bestelldaten abspeichern
    index = 0
    for MPS in modified_order_data:
        volume_index = (MPS[0][6], index)
        volume_index_old.append(volume_index)
        index += 1
    
    # Sortiere die erzeugte Liste
    volume_index_new = sorted (volume_index_old, reverse = True) 
    
    # Nun ist eine Zuordnung zwischen den Indizes der unsortierten Bestelldaten und den nach Volumen sortierten modifzierten Bestelldaten möglich
    for index_to_change in volume_index_new:
        sorted_modified_order_data.append(modified_order_data[index_to_change[1]]) # Index steht an Position 1
        
    '''Modifizierte Bestelldaten erzeugt'''

    # Gebe Liste mit Instanze aus modifizierte Bestelldaten mit zufälliger Reihenfolge und Containerdaten zurück
    return sorted_modified_order_data, container_data

class Packing_Algorithm:

    def __init__(self, order_instance, population_size, nr_of_iterations, choosen_parent_pairing, list_of_choosen_crossovers, nr_of_parents_to_keep):
        self.start_time = time.time() # Anfangszeit
        self.end_time = 0 # Schlusszeit
        self.time_needed = 0 # benötigte Zeit

        # Dateinamen der Bestellung und der Containerliste festlegen
        self.order_file_name = order_instance + ".xlsx"
        self.container_file_name = "Containerliste" + ".xlsx"    

        # Initialisiere Parameter für den Evolutionären Algorithmus
        self.population_size = population_size # Startpopulationsgröße
        self.nr_of_iterations = nr_of_iterations # Anzahl an Folgegenerationen die der evolutionöre Algorithmus erzeugen soll
        self.choosen_parent_pairing = choosen_parent_pairing # Gewählte Elternselektion
        self.list_of_choosen_crossovers = list_of_choosen_crossovers # Gewähltes Rekombinationsverfahren
        self.nr_of_parents_to_keep = nr_of_parents_to_keep # Anzahl der besten Individuen, die in die Folgegeneration übernommen werden sollen. Hat nur Auswirkungen, wenn die lokale Suche in der Funktion run_all_offspring_generations der Klasse Populations nicht durchgeführt wird.
        
        # Rufe Datenvorverarbeitung auf und erhalte die initialen modifizierten Bestelldaten sortiert nach Packstückvolumen und Grundfläche der Orientierungen sowie die Containerdaten
        self.initial_modified_order_data, self.container_data = data_preparation(self.order_file_name, self.container_file_name)

        # Initialisiere die durch die Basisheuristik mit der initialen Packreihenfolge berechnete Lösung
        self.best_solution_base = Solution(copy.deepcopy(self.initial_modified_order_data), self.container_data) # erzeugte einer leere Lösung und Initailisiere sie mit der initialen MPS Packreihenfolge & den Containerdaten
        self.best_solution_base.calculate_solution() # Berechne die Lösung
        

        
        # Initialisiere Variable zum speichern der durch die Verbesserungsheuristik erzeugten besten Lösung
        self.best_solution_improve = 0

        # Initialisiere Listen zum speichern der Simulationsdaten durch die Grid_Search
        self.best_volume_usage_values = [] # Initialisiere eine Liste, welche die Volumenauslastung der besten Lösung jeder Generation speichert
        self.index_best_solution = [] # Initialisiere eine Liste, welche den Index des besten Individuums einer Population trägt

    
    # Berechne die benötigte Zeit
    def set_needed_time(self):
        self.end_time = time.time() # Schlusszeit
        self.time_needed = self.end_time - self.start_time # Berechne die benötigte Zeit


    # Funktion, die Eigenschaften über die übergebene Lösung ausgibt
    def print_solution_propertiers(self, best_solution):
        print("\nDie gesamten Versandkosten liegen bei " +  str(best_solution.costs) + " €")
        print("Die durchschnittliche Containerauslastung liegt bei " + str(best_solution.usage_average) + " %")
        print("Es wurden " + str(best_solution.container_number) + " Container benötigt")


    # Funktion, die den gesamten Algorithmus ausführt
    def run_algorithm(self):
            
        # Gebe die Ergebnisse der initialen Lösung aus
        # print(" ")
        # print("Ergebnis der Basisheuristik mit der sortierten Packreihenfolge nach Volumen und größter Grundfläche:")
        # self.print_solution_propertiers(self.best_solution_base)

        # Visualisere gepackte Container der Lösung der Basisheuristik
        # self.best_solution_base.visualize()
        # Gebe Packplan der Lösung der Basisheuristik aus
        # print(" ")
        # print("Packplan Basisheuristik")
        # print(self.best_solution_base.containers)

        # Erzeuge eine Instanz des genetischen Algorithmus
        genetic_algorithm = Genetic_Algorithm(self.initial_modified_order_data, self.container_data, self.population_size, self.nr_of_iterations, self.choosen_parent_pairing, self.list_of_choosen_crossovers, self.nr_of_parents_to_keep)
        # Führe den genetischen Algorithmus aus
        self.best_solution_improve, self.best_volume_usage_values, self.index_best_solution = genetic_algorithm.run_genetic_algorithm()

        # Berechne die benötigte Zeit & gebe sie aus
        self.set_needed_time()
        print("\nDie Rechenzeit liegt bei " + str(round(self.time_needed, 2)) + " Sekunden\n")

        # Gebe die Ergebnisse der finalen Lösung mit der niedrigesten Volumenauslastung aus
        print("Ergebnisse der finalen Lösung:")
        self.print_solution_propertiers(self.best_solution_improve)

        print("\nRelative Verbesserung zum Ergebnis der Basisheuristik:")
        print(str((self.best_solution_improve.usage_average/self.best_solution_base.usage_average - 1) * 100) + "%\n")
        
        # Visualisere gepackte Container der besten Lösung
        self.best_solution_improve.print_packplan()
        self.best_solution_improve.visualize() #--> aktivieren um die Visualisierung in Fenster zu sehen
        # Gebe Packplan der besten Lösung aus
        # print("Packplan der besten Lösung")
        # print(self.best_solution_improve.containers)
    
        return self.best_volume_usage_values, self.index_best_solution, self.time_needed, self.best_solution_improve