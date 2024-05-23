from datetime import datetime
from Classes.packing_algorithm import Packing_Algorithm
from Classes.container import Container
from Classes.packplan import Packplan

class Grid_Search:
    
    def __init__(self):
        # Definiere die zu simulierenden Benchmarkinstanzen
        self.list_of_order_instances = ["01_Datensatz_Beispiel_4"]
        # Definiere die Anzahl der Iteration
        self.grid_nr_of_iterations = 10
        # Definiere die Populationsgrößen
        self.list_of_population_sizes = [10]
        # Initialisiere Variable zur Auswahl der Verfahrens für die Bildung von Elternpaaren für den Crossover (0-5)
        self.list_of_choosen_parent_pairing = [0]
        # Initialisiere Liste zu Auswahl der durchzuführenden Crossovervarianten & befülle sie mit den folgenden Zahlen, um festzulegen mit welchen Crossovervarianten Nachkommen erzeugt werden sollen
        # 0 = erste bzw. zweite Hälfte der Packreihenfolge beider Elternteile zuerst
        # 1 = Container mit Auslastung > dem Auslastungsdurchschnitt zuerst
        # 2 = erste Hälfte der Packreihenfolge zuerst. Von A bzw. B zuerst
        self.list_of_lists_of_choosen_crossovers = [[1]]
        
        # Initialisiere Listen in welche die Ergebnisse aller Simulationen gespeichert werden
        self.list_volume_usage_values = []
        self. list_index_best_solution = []
        self.list_packplan = []

        # Initialisiere die aktuelle Zeit für den Dateinamen
        self.timestamp = datetime.now().strftime("%Y-%m-%d %Hh%Mm%Ss")

    # Ruft den gesamten Algorithmus für jede in der Gittersuche definierte Parameterkombination auf.
    def run_grid_search(self):

        # Führe Simulationen für die verschiedenen Parameterausprägungen durch
        for grid_order_instance in self.list_of_order_instances:
            for grid_population_size in self.list_of_population_sizes:
                for grid_list_of_choosen_crossovers in self.list_of_lists_of_choosen_crossovers:
                    for grid_choosen_parent_pairing in self.list_of_choosen_parent_pairing:
                        
                        # Definiere die Anzahl an Eltern, die für die Bildung der neuen Population berücksichtigt werden sollen
                        nr_of_parents_to_keep = 1 # Berücksichtige die beste Lösung der Vorgängerpopulation
                        # nr_of_parents_to_keep = round(grid_population_size/2) #/2 = Berücksichtige Hälfte der Vorgängerpopulationslösunegn

                        packing_algorithm = Packing_Algorithm(grid_order_instance, grid_population_size, self.grid_nr_of_iterations, grid_choosen_parent_pairing, grid_list_of_choosen_crossovers, nr_of_parents_to_keep) # Erzeuge eine Insatnz des Algorithmus mit den jeweiligen Parametern
                        best_volume_usage_values, index_best_solution, time_needed, best_solution_improve = packing_algorithm.run_algorithm() # Führe den Algorithmus aus und erhalte eine Liste mit der Volumenauslastung bzw. Index der besten Lösung je Population
                        properties_simulation = 'Time needed: ' + str(round(time_needed,2)) + ' - PopSize: ' + str(grid_population_size) + ' - Pairing: ' + str(grid_choosen_parent_pairing) + ' - Crossover: ' + ' '.join(str(element) for element in grid_list_of_choosen_crossovers) + ' - BI: ' + grid_order_instance #Speichere Eigenschaften der Simulation in einer Variable

                        # Füge die Eigenschaften der Simulation als erstes Elemnet den Listen mit den Simulationsergebnissen hinzu
                        best_volume_usage_values.insert(0, properties_simulation) 
                        index_best_solution.insert(0, properties_simulation)

                        # Füge die Listen mit den Simulationsergebnissen den Listen mit den Ergebnissen aller Simulationen hinzu
                        self.list_volume_usage_values.append(best_volume_usage_values)
                        self.list_index_best_solution.append(index_best_solution)
                        temp_packplan = Packplan.get_packplan()
                        for i in range(len(temp_packplan)):
                            self.list_packplan.append((temp_packplan[i][0], temp_packplan[i][1], temp_packplan[i][2], temp_packplan[i][3], temp_packplan[i][4], temp_packplan[i][5], temp_packplan[i][6], temp_packplan[i][7], temp_packplan[i][8], temp_packplan[i][9]))

        
            # Speichere die Ergebnisse der Simulation in eine CSV Datei
            #self.save_to_csv(grid_order_instance) 
'''
    # Funktion, die alle übergebenen Ergebnisse einer Simulation in eine CSV Datei speichert
    def save_to_csv(self, grid_order_instance):
       
        # Initialisiere die Dateipfade in  welche die Ergebnisse gespeichert werden sollen
        file_path_volume = 'Simulationen//' + str(self.timestamp) + ' BI' + grid_order_instance[:2] + ' Simulation Volumenauslastung mit Mutation.csv'
        file_path_index = 'Simulationen//' + str(self.timestamp) + ' BI' + grid_order_instance[:2] + ' Simulation bester Index mit Mutation.csv'
        file_path_packplan = 'Simulationen//' + str(self.timestamp) + ' BI' + grid_order_instance[:2] + ' Simulation Packplan.csv'

        # Erzeuge Dateframes mit den Simulationsergebnissen
        results_volume_usage = pd.DataFrame(self.list_volume_usage_values)
        self.list_volume_usage_values = []
        results_volume_usage = results_volume_usage.T
        result_index_best_solutions = pd.DataFrame(self.list_index_best_solution)
        self.list_index_best_solution = []
        result_index_best_solutions = result_index_best_solutions.T
        results_packplan = pd.DataFrame(self.list_packplan)
        self.list_packplan = []

        # Speichere die Dataframes in CSV Dateien
        # results_volume_usage.to_csv(path_or_buf=file_path_volume, header=False, index=False)
        # result_index_best_solutions.to_csv(path_or_buf=file_path_index, header=False, index=False)
        results_packplan.to_csv(path_or_buf=file_path_packplan, header=False, index=False)

'''