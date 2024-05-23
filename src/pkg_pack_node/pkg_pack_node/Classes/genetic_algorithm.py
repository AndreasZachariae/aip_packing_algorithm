import itertools
from Classes.population import Population
import copy
import matplotlib.pyplot as plt

class Genetic_Algorithm:
    def __init__(self, initial_modified_order_data, container_data, population_size, nr_of_iterations, choosen_parent_pairing, list_of_choosen_crossovers, nr_of_parents_to_keep):
        self.initial_modified_order_data = initial_modified_order_data
        self.container_data = container_data

        self.population_size = population_size # Initialisiere die Populationsgröße
        self.number_of_parents = population_size # Initialisiere die Anzahl an berücksichtigten Eltern
        self.nr_of_iterations = nr_of_iterations # Initialisiere die Anzahl der Iterationen
        self.choosen_parent_pairing = choosen_parent_pairing # Initialisiere Variable zur Festlegung der Paarbildungsvariante
        self.list_of_choosen_crossovers = list_of_choosen_crossovers # Initialisiere Liste zur Festlegung der Crossovervariante
        self.nr_of_parents_to_keep = nr_of_parents_to_keep # Initialisiere Variable zur Festlegung der Anzahl an Eltern, die bei der Populationsbildung aus der Vorgängerpopulation mitberücksichtigt werden sollen

        self.iteration_numbers = [] # Initialisiere eine Liste, welche die Iterationsnummer nacheinander speichert (dient ausschließlich der Visualisierung)
        self.best_volume_usage_values = [] # Initialisiere eine Liste, welche die Volumenauslastung der besten Lösung jeder Generation speichert (dient ausschließlich der Visualisierung)
        self.index_best_solution = list() # Initialisiere eine Liste, welche den Index des besten Individuums einer Population trägt (dient ausschließlich der Visualisierung)

        self.tabu_list = [] # Initialisiere eine Liste in der alle bereits verwendeten Packreihenfolgen gespeichert werden
        
    def add_iteration_number(self, iteration_number):
        self.iteration_numbers.append(iteration_number)

    def add_best_volume_usage(self, best_volume_usage):
        self.best_volume_usage_values.append(best_volume_usage)

    # Fügt eine übergebene Mange an Packreihenfolgen der Tabu-Liste hinzu fügt
    def add_pack_orders_to_tabu_list(self, pack_orders_current_population):
        for pack_order in pack_orders_current_population: # Durchlaufe die übergebenen Packreihenfolge
            self.tabu_list.append(copy.deepcopy(pack_order)) # Füge jede Packreihenfolge der Tabu Liste hinzu


    # Erzeugen & berechnen der ersten Population
    def run_initial_population(self):
            # Erzeuge eine leere Population
            initial_population = Population(self.number_of_parents, self.population_size-1, [], self.choosen_parent_pairing) # -1 bei Populationsgröße weil das durch die Basisheuristik erzeugte Individuum in die Startpopulation ebenfalls miteinfließt (wird später hinzugefügt)
            # Initialisiere die Anzahl der Packstücke
            article_count = len(self.initial_modified_order_data)

            # Erzuege die Packreihenfolgen für die erste Population
            initial_population.create_start_population_pack_orders(article_count) 
            
            # Füge die Packreihenfolgen der Liste mit allen bereits verwendeten Packreihenfolgen hinzu
            self.add_pack_orders_to_tabu_list(initial_population.pack_orders)

            # Erzeuge die MPS Datensätze für die erste Population
            initial_population.create_start_population_mps(self.initial_modified_order_data)


            # Berechne die Lösungen für jede MPS der ersten Population und sortiere sie nach Volumenauslastung (größte Auslastung am Anfang der Liste)
            # Füge den IIndex der besten erzeugten Lösung in die entsprechende Visualisierungsliste ein
            self.index_best_solution = initial_population.get_population_solutions(self.container_data, self.index_best_solution) 
     
            # Sortiere die Lösungen nach Volumenauslastung (größte Auslastung am Anfang der Liste)
            initial_population.sort_population_solutions()

            # Füge die Iteratiosnnummer und die beste Auslastung der ersten Generation in die entsprechenden Visualisierungslisten ein
            self.add_iteration_number(-self.nr_of_iterations)
            self.add_best_volume_usage(initial_population.population_solutions[0].usage_average)
            return initial_population


    # Erzeugen & berechnen aller Folgegenerationen
    def run_all_offspring_generations(self, initial_population):
        # Initialisiere die Vorgängerpopulation mit der initialen Population
        previous_population = initial_population

        '''Durchlaufe die definierte Anzahl an Generationen'''

        while (self.nr_of_iterations > 0):
            # Erzeuge eine leere Population
            current_population = Population(self.number_of_parents, self.population_size, previous_population.population_solutions, self.choosen_parent_pairing)
            '''Code für die Durchführung des GAs nur mit Mutation'''
            # # Weiße Packreihenfolge der vorherigen Generation der Packreihenfolge der aktuellen Generation zu
            # current_population.pack_orders = previous_population.pack_orders
            # # Führe nur Mutation durch
            # current_population.mutation(self.tabu_list) # Mutiere alle Packreihenfolgen und weiße sie der Populationsinstanz zu
            '''Ende des Codes für die Durchführung des GAs nur mit Mutation'''
            
            '''Abschnitt auskommentieren für nur Mutation'''
            # Zusätzlich muss der Funktionsaufruf für die Mutation innerhalb der Funktion create_packing_orders_for_current_offspring_generation in der Klasse Population auskommentiert werden
            # Erzeuge Packreihenfolgen für die neue Population
            current_population.create_packing_orders_for_current_offspring_generation(self.tabu_list, self.list_of_choosen_crossovers) # Erzuege die Packreihenfolgen für die neue Generation
            '''Ende des auskommentieren Abschnitts für nur Mutation'''

            '''Lokale Suche für die beste Lösung (Abschnitt auskommentieren für Berechnungen ohne lokale Suche)'''
            # Erzeuge Population nur mit bester Lösung der Vorgängerpopulation und mutiere sie für eine lokale Suche
            dummy_population = copy.deepcopy(previous_population) # Erzeuge Kopie der Vorgängerpopulation
            dummy_population.pack_orders = [dummy_population.pack_orders[0]] # Entferne alle Packreihenfolgen der Vorgängergeneration bis auf die beste
            dummy_population.mutation(self.tabu_list) # Mutiere die beste Packreohenfolge der Vorgängergeneration
            current_population.add_pack_order(dummy_population.pack_orders[0]) #Füge die mutierte Packreohenfolge den Packreihenfolgen der aktuellen Generation hinzu
            '''Ende der lokalen Suche für die beste Lösung'''

            # Füge die Packreihenfolgen der Liste mit allen bereits verwendeten Packreihenfolgen hinzu
            self.add_pack_orders_to_tabu_list(current_population.pack_orders)
            
            # Erzeuge die MPS Datensätze für die aktuelle Population
            current_population.create_offspring_population_mps(self.initial_modified_order_data)
            
            # Berechne die Lösung für jeden Nachkommen der aktuellen Population und sortiere die Lösungen nach der durchschnittlichen Volumenauslastung (größte Auslastung am Anfang der Liste)
            # Füge den Index der besten erzeugten Lösung in die entsprechende Visualisierungsliste ein
            self.index_best_solution = current_population.get_population_solutions(self.container_data, self.index_best_solution)
            
            '''Fortsetzung Lokale Suche für die beste Lösung (Abschnitt auskommentieren für Berechnungen ohne lokale Suche)'''
            # Prüfe ob der durch die lokale Suche erzeugte Mutant schlechter ist als die nicht mutierte beste Lösung der Vorgängergeneration.
            if current_population.population_solutions[-1].usage_average <= previous_population.population_solutions[0].usage_average: 
                current_population.population_solutions[-1] = previous_population.population_solutions[0] # Ersetze den Mutanten durch die beste Lösung der Vorgängergeneration
                current_population.pack_orders[-1] = previous_population.pack_orders[0] # Ersetze die Packreihenfolge des Mutanten durch die der besten Lösung der Vorgängergeneration
            '''Ende Fortsetzung der lokalen Suche für die beste Lösung'''
            
            '''Ohne lokale Suche (Abschnitt muss ausgeführt werden)'''
            # # Ergänze beste x Lösung der Eltern in der Liste
            # for parent_solution in itertools.islice(previous_population.population_solutions, 0, self.nr_of_parents_to_keep):
            #     current_population.add_population_solution(parent_solution)
            '''Ende ohne lokale Suche'''

            # Sortiere die Lösungen nach Volumenauslastung (größte Auslastung am Anfang der Liste)
            current_population.sort_population_solutions()
            self.nr_of_iterations -= 1 # Verringere die Zählervariable für die Anzahl an Generationen/ Iterationen, die der genetischen Algorithmus noch durchlaufen soll
            
            # Speichere die erzeugte Population als Vorgängerpopulation für die nächste zu erzeugende Population
            previous_population = current_population

            # Füge die Iteratiosnnummer und die beste Auslastung der ersten Generation in die entsprechenden Visualisierungslisten ein
            self.add_iteration_number(-self.nr_of_iterations)
            self.add_best_volume_usage(current_population.population_solutions[0].usage_average)

        '''Die definierte Anzahl an Generationen wurde durchlaufen'''
        return current_population.population_solutions[0]


    # Funktion die die Volumenauslastung über die Generationen & die beste erzeugte Lösung pro Population visualisiert
    def show_line_chart_with_best_solutions_of_each_generation(self):
        # Initialisiere die subplot Funktion
        figure, axis = plt.subplots(2, 1, figsize=(25,10))

        axis[0].plot(self.iteration_numbers, self.best_volume_usage_values)
        axis[0].set_title('Population size: ' + str(self.population_size) + ', Iterations: ' + str(self.nr_of_iterations) + ', Crossover: ' + str(self.list_of_choosen_crossovers).strip('[]') + ', Pairing Variant: ' + str(self.choosen_parent_pairing))
        axis[0].set_ylabel('Auslastung')

        axis[1].plot(self.iteration_numbers, self.index_best_solution)
        axis[1].set_xlabel('Populationsnummer')
        axis[1].set_ylabel('Index beste neue Lösung')

        plt.show()
    


    # Führe alle Iterationen des evolutionären Algorithmus durch
    def run_genetic_algorithm(self):     
        # Erzeuge und berechne die erste Population
        initial_population = self.run_initial_population()

        # Erzeuge und berechne die Populationen aller Folgegenerationen
        best_solution = self.run_all_offspring_generations(initial_population)


        # Erezuge einen line plot, der die Auslastung pro Generation anzeigt
        # self.show_line_chart_with_best_solutions_of_each_generation()

        # Gebe die finale Lösung zurück
        return best_solution, self.best_volume_usage_values, self.index_best_solution