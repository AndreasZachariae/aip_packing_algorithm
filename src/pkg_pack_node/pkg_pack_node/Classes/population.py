import random
from Classes.solution import Solution
import copy

class Population():

    def __init__(self, number_of_parents, population_size, parent_solutions, choosen_parent_pairing):
        self.pack_orders = [] # Liste in die alle Genome einer Population als Liste gespeichert werden
        self.mps_datasets = [] # Liste in der alle modifizierten Bestelldatensätze einer Population gespeichert werden
        self.parent_solutions = parent_solutions # Liste mit allen Lösungen der Populationseltern
        self.population_solutions = [] # Liste mit allen Lösungen der Population
        self.list_of_parent_pairs_for_crossover = [] # Liste mit Tupel für jedes Elternpaar, Tupel enthält nur Index des Elternteils
        self.number_of_parents = number_of_parents # Anzahl an Elternindividuen, die für den Crossover zu berücksichtigen sind
        self.population_size = population_size # Startpopulationsgröße
        self.choosen_parent_pairing = choosen_parent_pairing # Index für gewähltes Elternselektionsverfahren

    def add_pack_order(self, pack_order):
        self.pack_orders.append(pack_order)

    def add_mps_dataset(self, mps_dataset):
        self.mps_datasets.append(mps_dataset)

    def add_population_solution(self, solution):
        self.population_solutions.append(solution)

    def sort_population_solutions(self):
        self.population_solutions.sort(key = lambda solution: solution.usage_average, reverse = True) # Sortiere die Lösungen nach Volumenauslastung (größte Auslastung am Anfang der Liste)

    # Entferne alle Eltern, die nicht für die Rekombination berücksichtigt werden sollen
    def ceep_parents_for_pairing(self):
        self.parent_solutions = self.parent_solutions[:self.number_of_parents]


    '''Funktionen für die Berechnung der Elternpaarzuweisung'''
    # Bilde die Elternpaare für den Crossover anhand des ausgewählten Verfahrens
    def create_parent_pairs(self):
        if (self.choosen_parent_pairing == 0):
            self.get_parent_pairs_first_with_last()
        elif (self.choosen_parent_pairing == 1):
            self.get_parent_pairs_first_with_last_step_2()
        elif (self.choosen_parent_pairing == 2):
            self.get_parent_pairs_last_with_first()
        elif (self.choosen_parent_pairing == 3):
            self.get_parent_pairs_first_with_second()
        elif (self.choosen_parent_pairing == 4):
            self.get_parent_pairs_first_with_better_half()
        elif (self.choosen_parent_pairing == 5):
            self.get_parent_pairs_better_half_with_first()


    # Funktion zur Erzeugungvon Elternpaaren (nFWL) für den Crossover: Bester mit Schlechtestem, Zweitbester mit Zweitschlechtestem...
    def get_parent_pairs_first_with_last(self):

        # Initialisiere Zählervariable für die Auswahl des Elternteils A bzw. B
        parent_A = 0
        parent_B = 0
        # Initialisiere Zählervariable für die Auswahl des nächsten noch nicht berücksichtigten Elternteils
        parent_counter = 0

        while (parent_counter < self.number_of_parents/2): # Überprüfe, ob bereits alle Eltern berücksichtigt wurden
            parent_A = parent_counter # Weise Index für Elternteil A zu (Starte am Anfang der Liste mit allen Individuen)
            if ((self.number_of_parents%2 == 1) and ((self.number_of_parents-1)/2 == parent_counter)): # Überprüfe ob die Populationsgröße ungerade ist und alle Elternteile bis auf einen (den Mittleren) bereits berücksichtigt wurden
                parent_B = 0 # Weiße Index für Elternteil B zu: erstes Elternteil der Population
            else:
                parent_B = (self.number_of_parents - 1 - parent_counter) # Weise Index für Elternteil B zu (Starte am Ende der Liste mit allen Individuen)
            
            
            self.list_of_parent_pairs_for_crossover.append((parent_A, parent_B)) # Füger das erzeugte Elternpaar der Liste hinzu, welche alle Elternpaare enthält die miteinander rekombiniert werden sollen
            parent_counter += 1 # Erhöhe Zählervariable für die Auswahl des nächsten noch nicht berücksichtigten Elternteils


    # Funktion zur Erzeugungvon Elternpaaren (nFWLS2) für den Crossover: Bester mit Schlechtestem, Drittbester mit Drittschlechtestem...
    def get_parent_pairs_first_with_last_step_2(self):

        # Initialisiere Zählervariable für die Auswahl des Elternteils A bzw. B
        parent_A = 0
        parent_B = 0
        # Initialisiere Zählervariable für die Auswahl des nächsten noch nicht berücksichtigten Elternteils
        parent_counter = 0

        while (parent_counter < self.number_of_parents): # Überprüfe, ob bereits alle Eltern berücksichtigt wurden

            parent_A = parent_counter # Weise Index für Elternteil A zu (Starte am Anfang der Liste mit allen Individuen)

            if ((self.number_of_parents%2 == 1) and ((self.number_of_parents-1)/2 == parent_counter)): # Überprüfe ob die Populationsgröße ungerade ist und alle Elternteile bis auf einen (den Mittleren) bereits berücksichtigt wurden
                parent_B = 0 # Weiße Index für Elternteil B zu: erstes Elternteil der Population
            else:
                parent_B = (self.number_of_parents - 1 - parent_counter) # Weise Index für Elternteil B zu (Starte am Ende der Liste mit allen Individuen)
            
            
            self.list_of_parent_pairs_for_crossover.append((parent_A, parent_B)) # Füger das erzeugte Elternpaar der Liste hinzu, welche alle Elternpaare enthält die miteinander rekombiniert werden sollen
            parent_counter += 2 # Erhöhe Zählervariable für die Auswahl des nächsten noch nicht berücksichtigten Elternteils

    # Funktion zur Erzeugungvon Elternpaaren (LWF) für den Crossover: Schlechtester mit Bestem, Zweitschlechtester mit Zweitbestem...
    def get_parent_pairs_last_with_first(self):

        # Initialisiere Zählervariable für die Auswahl des Elternteils A bzw. B
        parent_A = 0
        parent_B = 0
        # Initialisiere Zählervariable für die Auswahl des nächsten noch nicht berücksichtigten Elternteils
        parent_counter = 0

        while (parent_counter < self.number_of_parents/2): # Überprüfe, ob bereits alle Eltern berücksichtigt wurden

            parent_A = parent_counter # Weise Index für Elternteil A zu (Starte am Anfang der Liste mit allen Individuen)

            if ((self.number_of_parents%2 == 1) and ((self.number_of_parents-1)/2 == parent_counter)): # Überprüfe ob die Populationsgröße ungerade ist und alle Elternteile bis auf einen (den Mittleren) bereits berücksichtigt wurden
                parent_B = 0 # Weiße Index für Elternteil B zu: erstes Elternteil der Population
            else:
                parent_B = (self.number_of_parents - 1 - parent_counter) # Weise Index für Elternteil B zu (Starte am Ende der Liste mit allen Individuen)
            
            
            self.list_of_parent_pairs_for_crossover.append((parent_B, parent_A)) # Füger das erzeugte Elternpaar der Liste hinzu, welche alle Elternpaare enthält die miteinander rekombiniert werden sollen
            parent_counter += 1 # Erhöhe Zählervariable für die Auswahl des nächsten noch nicht berücksichtigten Elternteils
        

    # Funktion zur Erzeugungvon Elternpaaren (TNB) für den Crossover: Bester mit Zweitbestem, Drittbester mit Viertbestem...
    def get_parent_pairs_first_with_second(self):

        # Initialisiere Zählervariable für die Auswahl des Elternteils A bzw. B
        parent_A = 0
        parent_B = 0
        # Initialisiere Zählervariable für die Auswahl des nächsten noch nicht berücksichtigten Elternteils
        parent_counter = 0

        while (parent_counter < self.number_of_parents): # Überprüfe, ob bereits alle Eltern berücksichtigt wurden

            parent_A = parent_counter # Weise Index für Elternteil A zu (Starte am Anfang der Liste mit allen Individuen)

            if ((self.number_of_parents%2 == 1) and (parent_counter == self.number_of_parents - 1)): # Überprüfe ob die Populationsgröße ungerade ist und alle Elternteile bis auf einen (den Letzten) bereits berücksichtigt wurden
                parent_B = 0 # Weiße Index für Elternteil B zu: erstes Elternteil der Population
            else:
                parent_B = (parent_counter + 1) # Weise Index für Elternteil B zu
            
            
            self.list_of_parent_pairs_for_crossover.append((parent_A, parent_B)) # Füger das erzeugte Elternpaar der Liste hinzu, welche alle Elternpaare enthält die miteinander rekombiniert werden sollen
            parent_counter += 2 # Erhöhe Zählervariable für die Auswahl des nächsten noch nicht berücksichtigten Elternteils


    # Funktion zur Erzeugungvon Elternpaaren (FWBH) für den Crossover: Bester mit besserer Häfte
    def get_parent_pairs_first_with_better_half(self):

        # Initialisiere Zählervariable für die Auswahl des Elternteils A bzw. B
        parent_A = 0
        parent_B = 0
        # Initialisiere Zählervariable für die Auswahl des nächsten noch nicht berücksichtigten Elternteils
        parent_counter = 0

        while (parent_counter < self.number_of_parents/2): # Überprüfe, ob bereits alle Eltern berücksichtigt wurden
            
            parent_A = 0 # Weise Index für Elternteil A zu (Betse Lösung)
            parent_B = parent_counter + 1     
            
            self.list_of_parent_pairs_for_crossover.append((parent_A, parent_B)) # Füger das erzeugte Elternpaar der Liste hinzu, welche alle Elternpaare enthält die miteinander rekombiniert werden sollen
            parent_counter += 1 # Erhöhe Zählervariable für die Auswahl des nächsten noch nicht berücksichtigten Elternteils


    # Funktion zur Erzeugungvon Elternpaaren (BHWF) für den Crossover: besere Hälfte mit Bestem
    def get_parent_pairs_better_half_with_first(self):

        # Initialisiere Zählervariable für die Auswahl des Elternteils A bzw. B
        parent_A = 0
        parent_B = 0
        # Initialisiere Zählervariable für die Auswahl des nächsten noch nicht berücksichtigten Elternteils
        parent_counter = 0

        while (parent_counter < self.number_of_parents/2): # Überprüfe, ob bereits alle Eltern berücksichtigt wurden
            
            parent_A = parent_counter + 1 # Weise Index für Elternteil A zu 
            parent_B = 0 # Weise Index für Elternteil B zu (Betse Lösung)
            
            
            self.list_of_parent_pairs_for_crossover.append((parent_A, parent_B)) # Füger das erzeugte Elternpaar der Liste hinzu, welche alle Elternpaare enthält die miteinander rekombiniert werden sollen
            parent_counter += 1 # Erhöhe Zählervariable für die Auswahl des nächsten noch nicht berücksichtigten Elternteils
    '''Ende der Funktionen für die Berechnung der Elternpaarzuweisung'''



    '''Funktionen für die Verschiedenen Crossovervarianten'''
    # Crossovervariante 0: Erzeuge 2 Nachkommen aus 2 Eltern
    # 1. Nachkommen: Erste gepackte Containerhälfte beider Eltern zuerst
    # 2. Nachkommen: Zweite gepackte Containerhälfte beider Eltern zuerst
    def create_two_offsprings_by_first_second_package_half_first(self, parent_A, parent_B, pack_order_offspring_A, pack_order_offspring_B):
        
        # Erzeuge Nachkommen A

        # wähle die als erstes gepackte Hälfte der Container in Elternteil A und dann B aus und Füge deren Packreihenfolge in die Packreihenfolge des Nachkommens ein 
        # (ist die gesamte Containerzahl ungerade wird aufgerundet z.B. Containeranzahl = 7, berücksichtige erste 4 Conatiner)
        pack_order_offspring_A = self.pick_first_half_of_parent_pack_order(self.parent_solutions[parent_A], pack_order_offspring_A)
        pack_order_offspring_A = self.pick_first_half_of_parent_pack_order(self.parent_solutions[parent_B], pack_order_offspring_A)

        # wähle die zweite Hälfte der gepackten Container in Elternteil A und dann B aus und Füge deren Packreihenfolge in die Packreihenfolge des Nachkommens ein 
        # (ist die gesamte Containerzahl ungerade wird aufgerundet z.B. Containeranzahl = 7, berücksichtige ab Conatiner 5)
        pack_order_offspring_A = self.pick_second_half_of_parent_pack_order(self.parent_solutions[parent_A], pack_order_offspring_A)
        pack_order_offspring_A = self.pick_second_half_of_parent_pack_order(self.parent_solutions[parent_B], pack_order_offspring_A)

        # Erzeuge Nachkommen B

        # wähle die zweite Hälfte der gepackten Container in Elternteil A und dann B aus und Füge deren Packreihenfolge in die Packreihenfolge des Nachkommens ein 
        pack_order_offspring_B = self.pick_second_half_of_parent_pack_order(self.parent_solutions[parent_A], pack_order_offspring_B)
        pack_order_offspring_B = self.pick_second_half_of_parent_pack_order(self.parent_solutions[parent_B], pack_order_offspring_B)

        # wähle die als erstes gepackte Hälfte der Container in Elternteil A und dann B aus und Füge deren Packreihenfolge in die Packreihenfolge des Nachkommens ein 
        pack_order_offspring_B = self.pick_first_half_of_parent_pack_order(self.parent_solutions[parent_A], pack_order_offspring_B)
        pack_order_offspring_B = self.pick_first_half_of_parent_pack_order(self.parent_solutions[parent_B], pack_order_offspring_B)

        return pack_order_offspring_A, pack_order_offspring_B

    # Crossovervariante 1: Erzeuge 2 Nachkommen aus 2 Eltern
    # Gepackte Container deren Auslastung > der durchschnittlichen Auslastung der Lösung sind zuerst
    def create_two_offsprings_by_package_half_with_highest_volume_usage_first(self, parent_A, parent_B, pack_order_offspring_A, pack_order_offspring_B):
        
        # Erzeuge Nachkommen A

        # wähle die die Container in Elternteil A bzw. B aus, deren Auslastung über der durchschnittlichen Auslastung des Elternteils liegt aus und Füge deren Packreihenfolge in die Packreihenfolge des Nachkommens ein
        pack_order_offspring_A = self.pick_containers_above_average_volume_usage_of_parent_pack_order(self.parent_solutions[parent_A], pack_order_offspring_A)
        pack_order_offspring_A = self.pick_containers_above_average_volume_usage_of_parent_pack_order(self.parent_solutions[parent_B], pack_order_offspring_A)

        # wähle die die Container in Elternteil A bzw. B aus, deren Auslastung unterhalb der durchschnittlichen Auslastung des Elternteils liegt aus und Füge deren Packreihenfolge in die Packreihenfolge des Nachkommens ein
        pack_order_offspring_A = self.pick_containers_below_average_volume_usage_of_parent_pack_order(self.parent_solutions[parent_A], pack_order_offspring_A)
        pack_order_offspring_A = self.pick_containers_below_average_volume_usage_of_parent_pack_order(self.parent_solutions[parent_B], pack_order_offspring_A)

        # Erzeuge Nachkommen B

        # wähle die die Container in Elternteil B bzw. A aus, deren Auslastung über der durchschnittlichen Auslastung des Elternteils liegt aus und Füge deren Packreihenfolge in die Packreihenfolge des Nachkommens ein
        pack_order_offspring_B = self.pick_containers_above_average_volume_usage_of_parent_pack_order(self.parent_solutions[parent_B], pack_order_offspring_B)
        pack_order_offspring_B = self.pick_containers_above_average_volume_usage_of_parent_pack_order(self.parent_solutions[parent_A], pack_order_offspring_B)

        # wähle die die Container in Elternteil B bzw. A aus, deren Auslastung unterhalb der durchschnittlichen Auslastung des Elternteils liegt aus und Füge deren Packreihenfolge in die Packreihenfolge des Nachkommens ein
        pack_order_offspring_B = self.pick_containers_below_average_volume_usage_of_parent_pack_order(self.parent_solutions[parent_B], pack_order_offspring_B)
        pack_order_offspring_B = self.pick_containers_below_average_volume_usage_of_parent_pack_order(self.parent_solutions[parent_A], pack_order_offspring_B)

        return pack_order_offspring_A, pack_order_offspring_B

    # Crossovervariante 2: Erzeuge 2 Nachkommen aus 2 Eltern
    # 1. Nachkommen: Erste gepackte Containerhälfte beider Elternteile zuerst
    def create_two_offsprings_by_first_package_half_first(self, parent_A, parent_B, pack_order_offspring_A, pack_order_offspring_B):
        
        # Erzeuge Nachkommen A

        # wähle die als erstes gepackte Hälfte der Container in Elternteil A und dann B aus und Füge deren Packreihenfolge in die Packreihenfolge des Nachkommens ein 
        # (ist die gesamte Containerzahl ungerade wird aufgerundet z.B. Containeranzahl = 7, berücksichtige erste 4 Conatiner)
        pack_order_offspring_A = self.pick_first_half_of_parent_pack_order(self.parent_solutions[parent_A], pack_order_offspring_A)
        pack_order_offspring_A = self.pick_first_half_of_parent_pack_order(self.parent_solutions[parent_B], pack_order_offspring_A)

        # wähle die zweite Hälfte der gepackten Container in Elternteil A und dann B aus und Füge deren Packreihenfolge in die Packreihenfolge des Nachkommens ein 
        # (ist die gesamte Containerzahl ungerade wird aufgerundet z.B. Containeranzahl = 7, berücksichtige ab Conatiner 5)
        pack_order_offspring_A = self.pick_second_half_of_parent_pack_order(self.parent_solutions[parent_A], pack_order_offspring_A)
        pack_order_offspring_A = self.pick_second_half_of_parent_pack_order(self.parent_solutions[parent_B], pack_order_offspring_A)

        # Erzeuge Nachkommen B

        # wähle die als erstes gepackte Hälfte der Container in Elternteil B und dann A aus und Füge deren Packreihenfolge in die Packreihenfolge des Nachkommens ein (ist die gesamte Containerzahl ungerade wird aufgerundet z.B. Containeranzahl = 7, berücksichtige erste 4 Conatiner)
        pack_order_offspring_B = self.pick_first_half_of_parent_pack_order(self.parent_solutions[parent_B], pack_order_offspring_A)
        pack_order_offspring_B = self.pick_first_half_of_parent_pack_order(self.parent_solutions[parent_A], pack_order_offspring_A)

        # wähle die zweite Hälfte der gepackten Container in Elternteil B und dann A aus und Füge deren Packreihenfolge in die Packreihenfolge des Nachkommens ein (ist die gesamte Containerzahl ungerade wird aufgerundet z.B. Containeranzahl = 7, berücksichtige ab Conatiner 5)
        pack_order_offspring_B = self.pick_second_half_of_parent_pack_order(self.parent_solutions[parent_B], pack_order_offspring_A)
        pack_order_offspring_B = self.pick_second_half_of_parent_pack_order(self.parent_solutions[parent_A], pack_order_offspring_A)

        return pack_order_offspring_A, pack_order_offspring_B
    '''Ende Funktionen für die Verschiedenen Crossovervarianten'''


    '''Sub-Funktionen für die Verschiedenen Crossovervarianten'''
    '''Subfunktionen der Crossovervariante 0 & 2'''
    # Crossovervariante 0 & 2: Zieht die Packreihenfolge von der als erstes gepackten Conatinerhälfte heran und fügt sie der Packreihenfolge des Nachkommen hinzu
    def pick_first_half_of_parent_pack_order(self, parent, pack_order_offspring):
        
        # Initialisiere Anzahl Container des Elternteils
        conatiners_in_parent = parent.container_number
        # Initialisiere Zählervariable für die Auswahl eines spezifischen Containers des Elternteils (Starte mit erstem Container der ersten Hälfte)
        picked_container_in_parent = 0
        
        # wähle die als erstes gepackte Hälfte der Container im Elternteil aus und Füge deren Packreihenfolge in die neu zu erzeugende Packreihenfolge ein (ist die gesamte Containerzahl ungerade wird aufgerundet z.B. Containeranzahl = 7, berücksichtige erste 4 Conatiner)
        while (picked_container_in_parent < (conatiners_in_parent/2)): 
            # Durchlaufe alle Packstücke des gewählten Containers aus dem Elternteil
            for article in parent.containers[picked_container_in_parent].article_locations:
                if (article.package_ID not in pack_order_offspring): # Überprüfe ob der Artikel noch nicht in der Packreihenfolge dieses Nachkommen vorkommt
                    pack_order_offspring.append(article.package_ID) # und füge die Packstück ID aller Packstücke dieses Containers der Reihe nach der Packstückreihenfolge des Nachkommen hinzu

            picked_container_in_parent += 1 # Erhöhe Zählervariable für die Containerauswahl des Elternteils
        
        # Gebe die erzeugte (Teil-)Packreihenfolge des Nachkommens zurück
        return pack_order_offspring

    # Crossovervariante 0 & 2: Zieht die Packreihenfolge von der als letztes gepackten Conatinerhälfte heran und fügt sie der Packreihenfolge des Nachkommen hinzu
    def pick_second_half_of_parent_pack_order(self, parent, pack_order_offspring):
        
        # Initialisiere Anzahl Container des Elternteils
        conatiners_in_parent = parent.container_number
        # Initialisiere Zählervariable für die Auswahl eines spezifischen Containers des Elternteils (Starte mit erstem Container der zweiten Hälfte)
        picked_container_in_parent = round(conatiners_in_parent/2) # aufrunden für ungerade Containeranzahl in Elternteil (ist die gesamte Containerzahl ungerade wird aufgerundet z.B. Containeranzahl = 7, berücksichtige ab Conatiner 5)
        
        # wähle die zweite Hälfte der gepackten Container im Elternteil aus und Füge deren Packreihenfolge in die neu zu erzeugende Packreihenfolge ein 
        while (picked_container_in_parent < conatiners_in_parent): 
            # Durchlaufe alle Packstücke des gewählten Containers aus dem Elternteil
            for article in parent.containers[picked_container_in_parent].article_locations:
                if (article.package_ID not in pack_order_offspring): # Überprüfe ob der Artikel noch nicht in der Packreihenfolge dieses Nachkommen vorkommt
                    pack_order_offspring.append(article.package_ID) # und füge die Packstück ID aller Packstücke dieses Containers der Reihe nach der Packstückreihenfolge des Nachkommen hinzu

            picked_container_in_parent += 1 # Erhöhe Zählervariable für die Containerauswahl des Elternteils
        
        # Gebe die erzeugte (Teil-)Packreihenfolge des Nachkommens zurück
        return pack_order_offspring
    '''Ende der Subfunktionen der Crossovervariante 0 & 2'''

    '''Subfunktionen der Crossovervariante 1'''
    # Crossovariante 1: Zieht die Packreihenfolge der Conatiner heran, deren Auslastung höher als die durchschnittliche Auslastung des übergebenen Elterteils ist und fügt sie der Packreihenfolge des Nachkommen hinzu
    def pick_containers_above_average_volume_usage_of_parent_pack_order(self, parent, pack_order_offspring):
        
        # Initialisiere Anzahl Container des Elternteils
        conatiners_in_parent = parent.container_number
        # Initialisiere die durchschnittliche Auslastung des Elternteils
        average_usage_of_parent = parent.usage_average
        # Initialisiere Liste die die (restlichen) Container des übergebenen Elternteils enthält
        parent_containers = copy.deepcopy(parent)
        
        # Durchlaufe dafür alle Container so oft, wie es noch einen Container gibt, der noch nicht der Packreihenfolge des Nachkommens hinzugefügt wurde & dessen Auslastung höher ist als die durchschnittliche Auslastung des Elternteils
        for nr_of_container_iterations in range(conatiners_in_parent):

            # Initialisiere Variable zum Speichern der Auslastung des Containers mit der höchsten Auslastung, der noch nicht der Packreihenfolge des Nachkommens hinzugefügt wurde
            volume_usage_of_best_container = 0
            # Initialisiere Variable zum Speichern des Index des Containers mit der höchsten Auslastung, der noch nicht der Packreihenfolge des Nachkommens hinzugefügt wurde
            index_of_best_container = -1
            # Initialisiere Variable zum Speichern der Auslastung eines ausgewählten Containers
            volume_usage_of_selected_container = 0
            # Initialisiere die Anzhal an Conatainern des Elternteils, die noch nicht der Packreihenfolge des Nachkommens hinzugefügt wurden
            not_packed_conatiners_in_parent = len(parent_containers.containers)

            # Durchlaufe alle Container, die noch nicht der Packreihenfolge des Nachkommens hinzugefügt wurden und finde davon den Container mit der höchsten Auslastung
            for container_index in range(not_packed_conatiners_in_parent):
                volume_usage_of_selected_container = parent_containers.containers[container_index].volume_usage_rate # Speichere die Volumenauslastung des aktuell betrachteten Containers
                if (volume_usage_of_selected_container > volume_usage_of_best_container): # Überprüfe, ob die Volumenasulastung des aktuell betrachteten Containers der bisher größten erfassten Auslastung entspricht
                    volume_usage_of_best_container = volume_usage_of_selected_container # Speicher die Auslastung dieses Containers als die bisher größte erfasste Auslastung
                    index_of_best_container = container_index # Speicher den Index dieses Containers
                
                # Erhöhe den Container Index um eins, um die Schleife für den Nächsten Container erneut zu starten
                container_index += 1
            
            # Nachdem der Container mit der größten Auslastung identifirt ist, der noch nicht der Packreihenfolge des Nachkommens hinzugefügt wurde:
            if (volume_usage_of_best_container >= average_usage_of_parent): # Überprüfe, ob dessen Auslastung höher ist als die durchschnittliche Auslastung des Elternteils
                # Durchlaufe alle Packstücke des gewählten Containers aus dem Elternteil
                for article in parent_containers.containers[index_of_best_container].article_locations:
                    if (article.package_ID not in pack_order_offspring): # Überprüfe ob der Artikel noch nicht in der Packreihenfolge dieses Nachkommen vorkommt
                        pack_order_offspring.append(article.package_ID) # und füge die Packstück ID aller Packstücke dieses Containers der Reihe nach der Packstückreihenfolge des Nachkommen hinzu

                # Entferne den gewählten Container aus der Containermenge des Elternteils
                del parent_containers.containers[index_of_best_container]
            
            else: # Bereits alle Container, deren Auslastung größer als die durchschnittlichen Volumenauslatung des Elternteils ist wurden der Packreihenfolge des Nachkommens hinzugefügt
                break # Unterbreche die Suche nach weiteren Containern
            
            # Erhöhe die Anzahl der durchgeführten Durchsuchungen aller Conatiner nach einem Container, der noch nicht der Packreihenfolge des Nachkommens hinzugefügt wurde & dessen Auslastung höher ist als die durchschnittliche Auslastung des Elternteils
            nr_of_container_iterations += 1
        
        # Gebe die erzeugte (Teil-)Packreihenfolge des Nachkommens zurück
        return pack_order_offspring
        
    # Crossovervariante 1: Zieht die Packreihenfolge der Conatiner heran, deren Auslastung geringer als die durchschnittliche Auslastung des übergebenen Elterteils ist und fügt sie der Packreihenfolge des Nachkommen hinzu
    def pick_containers_below_average_volume_usage_of_parent_pack_order(self, parent, pack_order_offspring):
        
        # Initialisiere Anzahl Container des Elternteils
        conatiners_in_parent = parent.container_number
        # Initialisiere die durchschnittliche Auslastung des Elternteils
        average_usage_of_parent = parent.usage_average
        # Initialisiere Liste die die (restlichen) Container des übergebenen Elternteils enthält
        parent_containers = copy.deepcopy(parent)
        
        # Durchlaufe dafür alle Container so oft, wie es noch einen Container gibt, der noch nicht der Packreihenfolge des Nachkommens hinzugefügt wurde & dessen Auslastung höher ist als die durchschnittliche Auslastung des Elternteils
        for nr_of_container_iterations in range(conatiners_in_parent):

            # Initialisiere Variable zum Speichern der Auslastung des Containers mit der höchsten Auslastung unterhalb des Auslastungsdurchschnitts, der noch nicht der Packreihenfolge des Nachkommens hinzugefügt wurde
            volume_usage_of_best_container_below_average = 0
            # Initialisiere Variable zum Speichern des Index des Containers mit der höchsten Auslastung unterhalb des Auslastungsdurchschnitts, der noch nicht der Packreihenfolge des Nachkommens hinzugefügt wurde
            index_of_best_container_below_average = -1
            # Initialisiere Variable zum Speichern der Auslastung eines ausgewählten Containers
            volume_usage_of_selected_container = 0
            # Initialisiere die Anzhal an Conatainern des Elternteils, die noch nicht der Packreihenfolge des Nachkommens hinzugefügt wurden
            not_packed_conatiners_in_parent = len(parent_containers.containers)

            # Durchlaufe alle Container, die noch nicht der Packreihenfolge des Nachkommens hinzugefügt wurden und finde davon den Container mit der höchsten Auslastung unterhalb des Auslastungsdurchschnitts
            for container_index in range(not_packed_conatiners_in_parent):
                volume_usage_of_selected_container = parent_containers.containers[container_index].volume_usage_rate # Speichere die Volumenauslastung des aktuell betrachteten Containers
                if (volume_usage_of_selected_container > volume_usage_of_best_container_below_average and volume_usage_of_selected_container < average_usage_of_parent): # Überprüfe, ob die Volumenasulastung des aktuell betrachteten Containers der bisher größten erfassten Auslastung unterhalb des Auslastungsdurchschnitts entspricht
                    volume_usage_of_best_container_below_average = volume_usage_of_selected_container # Speicher die Auslastung dieses Containers als die bisher größte erfasste Auslastung unterhalb des Auslastungsdurchschnitts
                    index_of_best_container_below_average = container_index # Speicher den Index dieses Containers
                
                # Erhöhe den Container Index um eins, um die Schleife für den Nächsten Container erneut zu starten
                container_index += 1
            
            # Nachdem der Container mit der größten Auslastung unterhalb des Auslastungsdurchschnitts identifirt ist, der noch nicht der Packreihenfolge des Nachkommens hinzugefügt wurde:
            if (volume_usage_of_best_container_below_average > 0): # Überprüfe, ob es überhaupt noch solch einen Conatiner gab
                # Durchlaufe alle Packstücke des gewählten Containers aus dem Elternteil
                for article in parent_containers.containers[index_of_best_container_below_average].article_locations:
                    if (article.package_ID not in pack_order_offspring): # Überprüfe ob der Artikel noch nicht in der Packreihenfolge dieses Nachkommen vorkommt
                        pack_order_offspring.append(article.package_ID) # und füge die Packstück ID aller Packstücke dieses Containers der Reihe nach der Packstückreihenfolge des Nachkommen hinzu

                # Entferne den gewählten Container aus der Containermenge des Elternteils
                del parent_containers.containers[index_of_best_container_below_average]
            
            else: # Bereits alle Container, deren Auslastung kleiner als die durchschnittlichen Volumenauslatung des Elternteils ist wurden der Packreihenfolge des Nachkommens hinzugefügt
                break # Unterbreche die Suche nach weiteren Containern
            
            # Erhöhe die Anzahl der durchgeführten Durchsuchungen aller Conatiner nach einem Container, der noch nicht der Packreihenfolge des Nachkommens hinzugefügt wurde & dessen Auslastung geringer ist als die durchschnittliche Auslastung des Elternteils
            nr_of_container_iterations += 1
        
        # Gebe die erzeugte (Teil-)Packreihenfolge des Nachkommens zurück
        return pack_order_offspring
    '''Ende der Subfunktionen der Crossovervariante 1'''
    '''Ende der Sub-Funktionen für die Verschiedenen Crossovervarianten'''



    '''Funktionen für die Mutation'''
    # Bekommt die Packreihenfolge aller Nachkommen übergeben und mutierte diese solange bis eine noch nicht vorgekommene Packreihenfolge entsteht
    def mutation(self, tabu_list):
        for offspring_index in range(len(self.pack_orders)): # Durchlaufe die Packreihenfolge aller Nachkommen

            self.pack_orders[offspring_index] = self.change_2_random_articles(self.pack_orders[offspring_index]) # Führe Funktion für Mutation auf: tauschen 2er zufällig gewählter Artikel aus
            
            if (self.check_if_pack_order_already_used(self.pack_orders[offspring_index], tabu_list) == True): # Mutiere so lange weiter, bis ein noch nicht verwendete Packreihenfolge entsteht 
                
                self.pack_orders[offspring_index] = self.change_2_random_articles(self.pack_orders[offspring_index])
    # Tausche zwei zufällig gewählte Artikel in der übergebenen Packreihenfolge
    def change_2_random_articles(self, pack_order):
        # Initialisiere Variable zum Zwischenspeichern der Artikel ID, des zu tauschenden Artikels
        article_ID_of_article_1 = 0

        # Initialisiere Index Variable für die beiden zu tauschenden Artikel. Wähle dabei den Index zwischen 0 und der Anzahl an Artikeln in dieser Packreihenfolge
        index_articel_1 = random.randrange(0,len(pack_order))
        index_articel_2 = random.randrange(0,len(pack_order))
        article_ID_of_article_1 = pack_order[index_articel_1] # Speichere die Artikel ID des ersten Artikels zwischen
        pack_order[index_articel_1] = pack_order[index_articel_2] # Überschreibe die Position des ersten Artikels mit der Artikel ID des zweiten Artikels
        pack_order[index_articel_2] = article_ID_of_article_1 # Überschreibe die Position des zweiten Artikels mit der Artikel ID des ersten Artikels
    
        return pack_order

    # Funktion die überprüft, ob die übergebene PAckreihenfolge bereits verwendet wurden. Durch Abgleich mit der Tabu-Liste
    def check_if_pack_order_already_used(self, pack_order, tabu_list):
        pack_order_already_used = pack_order in tabu_list # Überprüfe, ob die übergeben Packreihenfolge bereits berechnet wurde, falls ja: True
        return pack_order_already_used
    '''Ende der Funktionen für die Mutation'''



    '''Funktionen für die Erzeugung von Packreihenfolgen'''
    # Erzeuge die Packreihenfolgen für die Startpopulation
    def create_start_population_pack_orders(self, article_count):   
        # Erzeuge eine Liste, welche die Artikel ID jedes Packstücks enthält, wie sie in den MPS enthalten ist. Also beginnend mit 1
        all_article_IDs = [article_ID for article_ID in range(1, (article_count + 1))]

        self.add_pack_order(all_article_IDs) # Füge die Packreihenfolge des initialen MPS Datensatzes hinzu
        
        for i in range(self.population_size):
            random.seed(article_count+i) # Setze einen Seed für die Reproduzierbarkeit der erzeugten Reihenfolge, Seed muss sich pro Durchgang ädnern, damit unterschiedliche Reihenfolgen erzeugt werden
            self.add_pack_order(random.sample(all_article_IDs, len(all_article_IDs))) # Füge erzeugte gemischte Packreihenfolge der Liste für alle Reihenfolgen hinzu
    
    # Funktion zum Erzeugen der Packreihenfolge einer Nachkommen-Population
    def create_packing_orders_for_current_offspring_generation(self, tabu_list, list_of_choosen_crossovers):
        '''Starte Selektion'''

        # Selektiere Eltern die für die Rekombination berücksichtigt werden sollen
        self.ceep_parents_for_pairing()

        '''Ende Selektion'''


        '''Starte Crossover'''

        # Bilde die Elternpaare für den Crossover anhand des ausgewählten Verfahrens
        self.create_parent_pairs()

        # Initialisiere Index zum iterieren der Liste für die durchzuführenden Crossovervarianten
        list_index = 0

        # Führe einen Crossover für jedes Elternpaar für jede festgelegte Crossovervariante durch:
        while(list_index < len(list_of_choosen_crossovers)): # Jedes Elternpaar wird mit in der Liste definierten Crossover Varianten rekombiniert
            
            nr_of_crossover_variant = list_of_choosen_crossovers[list_index] # Weise Crossovervariante zu, welche diese Schleifeniteration durchgeführt werden soll

            # Führe für jedes Elternpaar die definierte Corssovervariante durch
            for parent_pair in self.list_of_parent_pairs_for_crossover:
                parent_A = parent_pair[0] # Weise den Index für Elternteil A zu
                parent_B = parent_pair[1] # Weise den Index für Elternteil B zu

                # Initialisiere Listen, in welche die Packreihenfolgen der beiden Nachkommen gespeichert werden
                pack_order_offspring_A = []
                pack_order_offspring_B = []

                if (nr_of_crossover_variant == 0): # 0. Crossovervariante

                    pack_order_offspring_A, pack_order_offspring_B = self.create_two_offsprings_by_first_second_package_half_first(parent_A, parent_B, pack_order_offspring_A, pack_order_offspring_B)

                elif (nr_of_crossover_variant == 1): # 1. Crossovervariante
                    
                    pack_order_offspring_A, pack_order_offspring_B = self.create_two_offsprings_by_package_half_with_highest_volume_usage_first(parent_A, parent_B, pack_order_offspring_A, pack_order_offspring_B)

                elif (nr_of_crossover_variant == 2): # 2. Crossovervariante

                    pack_order_offspring_A, pack_order_offspring_B = self.create_two_offsprings_by_first_package_half_first(parent_A, parent_B, pack_order_offspring_A, pack_order_offspring_B)

                else: # In der Liste für die durchzuführenden Crossovervarianten wurde eine Zahl aufgenommen, für die keine Crossovervariante definiert ist
                    print("!!!!!!! Die Liste für Crossovervarianten enthält einen nicht definierten Wert. !!!!!!!")
                    break
                self.add_pack_order(pack_order_offspring_A) # Füge die neue erzeugte Packreihenfolge des Nachkommens A der Liste aller Nachkommen hinzu
                self.add_pack_order(pack_order_offspring_B) # Füge die neue erzeugte Packreihenfolge des Nachkommens B der Liste aller Nachkommen hinzu

            list_index += 1 # Erhöhe Index zum iterieren der Liste für die durchzuführenden Crossovervarianten

        '''Ende Crossover'''


        '''Start Mutation'''

        self.mutation(tabu_list) # Mutiere alle Packreihenfolgen und weiße sie der Populationsinstanz zu
           
        '''Ende Mutation'''
    '''Ende Funktionen für die Erzeugung von Packreihenfolgen'''



    '''Funktionen für die Erzeugung von MPS Datensätzen'''
    # Erzeuge die MPS Datensätze für alle Packreihenfolgen der Startpopulation
    def create_start_population_mps(self, initial_modified_order_data):
        for pack_order in self.pack_orders: # Durchlaufe alle erzeugten Packreihenfolgen der ersten Population
            # Rufe Funktion auf, um MPS für die entsprechende Packreihenfolge zu erzeugen, übergebe initialen MPS Datensatz, Packreihenfolge des Individuums
            self.create_MPS_for_individual(initial_modified_order_data, pack_order)
       
    # Erzeuge die MPS Datensätze für alle Packreihenfolgen einer Nachkommen-Population
    def create_offspring_population_mps(self, initial_modified_order_data):
        for pack_order in self.pack_orders: # Durchlaufe alle Packreihenfolgen der Population
            # Rufe Funktion auf, um MPS für die entsprechende Packreihenfolge zu erzeugen, übergebe initialen MPS Datensatz, Packreihenfolge des Individuums
            self.create_MPS_for_individual(initial_modified_order_data, pack_order)

    # Erzeuge den MPS Datensatz für eine übergebene Packreihenfolge (Genotyp)
    def create_MPS_for_individual(self, initial_modified_order_data, pack_order_offspring):
        # Initialisiere eine leere Liste um den MPS Datensatz eines Nachkommens darin zu speichern
        mps_for_offspring = []
        for package_index in range(len(pack_order_offspring)): # Durchlaufe die Packstückreihenfolge des Nachkommen
            package_ID = pack_order_offspring[package_index] # Speichere die Packstück ID für das aktuell betrachtete Packstück in der Reihenfolge
            mapping_package_ID_to_list_index = package_ID -1 # Ziehe von der Packstück ID eins um den korrekten Listenindex zu haben
            mps_for_offspring.append(initial_modified_order_data[mapping_package_ID_to_list_index][:]) # Wähle durch den korrekten Listenindex die MPS Daten für das betrachtete Packstück aus dem initialen MPS Datensatz aus und Füge es der Liste für den MPS Datensatz des Nachkommens hinzu
        
        self.add_mps_dataset(mps_for_offspring)
        # return mps_for_offspring # Gebe den erzeugten MPS Datensatz für den Nachkommen zurück
    '''Ende Funktionen für die Erzeugung von MPS Datensätzen'''
    


    '''Funktionen für die Berechnung der Fitness'''
    # Liefere die Lösung für jede Populationsinstanz sortiert nach durchschnittlicher Containerauslastung (größte Auslatsung zuerst)
    def get_population_solutions(self, container_data, index_best_solution):
        # Initialisiere Liste, um die Volumenauslastung jeder Lösung zu speichern
        volume_usage_each_solution = []

        for modified_order_data in self.mps_datasets:
            solution = Solution(modified_order_data, container_data) # erzeugte einer leere Lösung und Initailisiere sie mit der MPS Packreihenfolge & den Containerdaten
            solution.calculate_solution() # Berechne die Lösung
            volume_usage_each_solution.append(solution.usage_average) # Speichere die Volumenauslastung in der Liste mit den Ausalstungen aller Lösungen
            
            self.add_population_solution(solution) # Speichere die Lösung mit ihrer Volumenauslastung in einer Liste

        # Berechen den Index der Lösung mit der höchsten Auslastung
        max_value = max(volume_usage_each_solution)
        max_index = volume_usage_each_solution.index(max_value)

        index_best_solution.append(max_index) # Füge den Index der Liste mit den Indizes der besten Lösung jeder Population hinzu  
        return index_best_solution
    '''Ende Funktionen für die Berechnung der Fitness'''