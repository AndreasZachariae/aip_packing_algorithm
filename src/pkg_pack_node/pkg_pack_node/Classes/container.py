import collections
from cython_files.cython_part import Check_Placement
from cython_files.cython_part import Check_Placement_Modified_Stacking
from cython_files.cython_part import Calculate_Overlapping_Areas
from Classes.article import Article
from datetime import datetime
import pandas as pd
import open3d as o3d
import numpy as np
import random
from Classes.packplan import Packplan
from Classes.container_transfer import container_transfer


box = collections.namedtuple('box', 'width height length')

class Container:
    


    def __init__(self, container_width, container_height, container_length, container_costs, max_weight, rest_packages):
        self.container = box(container_width, container_height, container_length) # Container Dimensionen
        self.costs = container_costs # Containerkosten
        self.max_weight = max_weight # Maximaltraglast des Containers
        self.carried_weight = 0 # Initialisiere Variable für das gepackte Gewicht des Containers
        self.article_locations = [] # Initialisiere Liste, in welche alle Packstücke gespeichert werden, die der Container enthält
        self.volume_used = 0 # Initialisiere Variable für das genutzte Volumen des Containers
        self.volume_usage_rate = 0 # Initialisiere Variable für das Containerauslastung in Prozent

        '''Variablen die der SKA für die Containerbefüllung benötigt'''
        self.rest_packages = rest_packages # Initialisiere Liste mit allen noch nicht gepackten Packstücken
        self.packages_packed = [] # Speichere platzierte Packstücke separat ab. Hierüber werden später die platzierten Packstücke aus den modifizierten Bestelldaten entfernt.
        self.package_sequence_nr = 1 # Initialisierung der Variable für die Packreihenfolge, mit der nachvollzogen werden kann, in welcher Reihenfolge die Packstücke in den Container gepackt werden
        self.call_from_stack_building = False # Initialisiere Variable um zu definieren, ob das die Add_One_Raw Funktion durch eine Stack Building Funktion aufgerufen wird

        self.factor_overfilling = 1.05 # Parameter, welcher die maximale Überschreitung von der Reihenlänge vorgibt. 1,05 entspricht einer Überschreitung von 5 % der Reihenlänge.

        self.articles_from_width_extension = [] # Initialisiere Liste, welche durch die width_extension mit Tupeln befüllt wird, im Falle von call_from_stack_building = False bleibt diese Liste leer
        self.articles_from_length_extension = [] # Initialisiere Liste, welche durch die length_extension mit Tupeln befüllt wird, im Falle von call_from_stack_building = False bleibt diese Liste leer

        # Initialisiere Startkoordinaten für die Platzierung von Packstücken in dem Container
        self.main_box_width = 0 
        self.main_box_height = 0
        self.main_box_length = 0

        # Initialisiere verfügbare Länge auf dem Boden des Containers (genannt Schichtlänge)
        # Die Schichtlänge wird nach dem Platzieren einer Hauptreihe um die Länge der Hauptreihe reduziert
        # Sie gibt damit die initiale Leerraumlänge vor dem Platzieren eine Hauptreihe vor
        self.main_layer_length = container_length # Initialisierung mit gesamter Containerlänge des Containertyps

        # Für das Wall-Buikding:
        # Anfangs steht der gesamte Container zur Bildung einer Reihe zur Verfügung
        self.main_raw_width = container_width # Breite des Leerraums
        self.main_raw_length = container_length # Länge des Leerraums

        # Für das Raw & Layer Building:
        self.articles_in_raw = [] # Liste in welche alle Artikel einer Raw/Layer gespeichert werden

        
        '''Ende Variablen die der SKA für die Containerbefüllung benötigt'''

    def add_article(self, width, height, length, x, y, z, is_stack, package_ID, package_sequence_nr, package_weight):
        self.article_locations.append(Article(width, height, length, x, y, z, is_stack, package_ID, package_sequence_nr, package_weight)) # Füge Packstück hinzu
        self.volume_used += (width * height * length)/1000000 # Erhöhe das genutze Volumen des Containers um das Volumen des hinzugefügten Packstücks
        self.carried_weight += package_weight # Erhöhe das Containergewicht um das Gewicht des hinzugefügten Packstücks
        

    def remove_article(self, article):
        self.article_locations.remove(article)
        self.volume_used -= (article.width * article.height * article.length)/1000000 # Verringere das genutze Volumen des Containers um das Volumen des entfernten Artikels
        self.carried_weight -= article.package_weight # Verringere das Containergewicht um das Gewicht des entfernten Packstücks
    
    def set_volume_usage(self, container_volume_usage):
        self.volume_used = container_volume_usage

    # Berechne Containerauslastung in Prozent
    def set_volume_usage_rate(self):
        self.volume_usage_rate = (self.volume_used/((self.container.width*self.container.height*self.container.length)/1000000))*100 

    def Check_Overload_Of_Artcicles (self, selected_orientation):
    
        # Initialisiere Variable, die bestimmt, ob die Platzierung des übergebenen Packstücks zulässig ist
        placement_allowed = True

        # Erzeuge eine Listen mit den Informationen welche Packstücke der Stapelgrundfläche mit welcher Fläche das zu stapelnde Packstück tragen. In Form von Tupeln: (Packstück ID, tragende Fläche)
        # 1. Funktionsaufruf; Übergebe: zu packendes Packstück; Packstück welches die Basis der Stapelfläche bildet + alle Packstücke durch die die Stapelgrundfläche der Breite nach erweitert wurde
        # 2. Funktionsaufruf; Übergebe: zu packendes Packstück; alle Packstücke durch die die Stapelgrundfläche der Länge nach erweitert wurde
        list_with_covered_areas = Calculate_Overlapping_Areas(selected_orientation, self.articles_from_width_extension) + Calculate_Overlapping_Areas(selected_orientation, self.articles_from_length_extension)
        
        # Initialisiere die Packstück ID des zu packenden Packstücks
        package_ID_article_to_stack = selected_orientation[7]
        # Initialisiere das Gewicht des zu packenden Packstücks
        weight_article_to_stack = selected_orientation[8]
        
        # Initialisiere eine Variable, welche die gesamte Fläche auf der das zu packende Packstück aufliegt speichert
        area_used_of_base = 0
        # Berechne die gesamte Fläche auf der das zu packende Packstück aufliegt
        for article_base in list_with_covered_areas:
            area_used_of_base += article_base[1]

        # Weise das Gewicht des zu stapelnden Packstücks anteilig auf die Packstücke der Stapelgrundfläche zu, auf denen es aufliegt & überprüfe ob dadurch eine Überladung zustande kommt
        for article_base in list_with_covered_areas:
            area_article_base = article_base[1] # Initialisiere die Größe der tragenden Fläche des Packstücks, welches beladen wird
            package_ID_article_base = article_base[0] # Initialisiere die Packstück ID des Packstücks, welches beladen wird
            covered_weight = 0 # Initialisiere das Gewicht, welches das Packstück, welches beladen wird, zu tragen hat
                    
            if(len(list_with_covered_areas) == 1): # Überprüfe, ob ein Packstück der Grundfläche das gesamte zu stapelnde Packstück trägt
                covered_weight = weight_article_to_stack # Weise das Gemsatgewicht des zu stapelnden Packstücks dem einen Packstück zu
            
            else: # Mehrere Packstücke targen das zu packende Packstück
                covered_weight = area_article_base/area_used_of_base * weight_article_to_stack # Weise das Gewicht des zu stapelnden Packstücks dem Packstück anteilig zu (tragende Fläche des Packstücks / Gesamte tragende Fläche * Gewicht)
            
            # Überprüfe für das zu beladende Packstück, ob es überladen ist, falls das zu stapelnde Packstück (anteilig) darauf gestapelt wird
            for article in self.article_locations:
                if (article.package_ID == package_ID_article_base and article.check_if_article_has_overload(covered_weight)):
                    placement_allowed = False # Falls es überladen ist, erlaube die Platzierung des zu stapelnden Packstücks nicht

        # Überprüfe abschließend, ob das zu stapelnde Packstück platziert werden darf & füge das (anteilig) getragene Gewicht des zu stapelnden Packstücks allen als Stapelfläche dienene tragenden Packstücken hinzu    
        if (placement_allowed == True):
            for article_base in list_with_covered_areas:
                package_ID_article_base = article_base[0] # Initialisiere die Packstück ID des Packstücks, welches beladen wird
                for article in self.article_locations:
                    if (article.package_ID == package_ID_article_base):
                        article.add_supported_article(package_ID_article_to_stack, covered_weight) # Füge das getragene Gewicht der Liste aller getragenene Gewichte des Packstücks hinzu      
        
        return placement_allowed

    '''Funktionen des Single Knapsack Algortihmus'''
    # Heuristik zur Befüllung eines Containers
    def pack_container (self):
    
        # Führe Wallbuilding durch
        raw_length_list = self.run_Wall_Building()
        
        # Führe Postoptmierung durch 
        self.run_Post_Optimization(raw_length_list)
    
        return self.packages_packed

    def run_Wall_Building (self):

        # Solange Restartikel bzw. Restpackstücke übrig sind, bilde Reihen
        while len(self.rest_packages) > 0:
            rest_packages_old = len(self.rest_packages) # Variable, welche der Anzahl an verbleibenden Packstücke entspricht. Diese wird als Abbruchkriterium verwendet.
            
            self.factor_overfilling = 1.05  # Parameter, welcher die maximale Überschreitung von der Reihenlänge vorgibt. 1,05 entspricht einer Überschreitung von 5 % der Reihenlänge.
            
            self.call_from_stack_building = False # Setzte Variable auf False, die bestimmt ob die Add_One_Raw Funktion durch eine Stack Building Funktion aufgerufen wird

            # Rufe Methode zur Bildung einer Reihe auf
            # Erhalte: Liste mit Packstücklängen
            raw_length_list = self.Add_One_Raw (self.main_raw_width, self.main_raw_length, self.main_box_width, self.main_box_length, self.main_box_height)
            
            # Terminiere Wall-Building, sofern keine weiteren Packstücke und damit keine weitere Wall erzeugt werden konnte
            # In diesem Falle müssen genauso viele Restpackstücke existieren wie vor dem Raw-Building
            if len(self.rest_packages) == rest_packages_old: 
                break
            
            # Rufe Methode zur Bildung eines Stapels auf
            # Durchlaufe die bereits platzierten Packstücke. Jedes platzierte Packstück wird als Stapel angesehen.
            for article_packed in self.article_locations:
                self.Stack_Building (article_packed)
            # Nun ist eine Wall komplettiert

            # Setze Koordinate für neue Haptreihe
            self.main_box_length = self.main_box_length + max(raw_length_list) # Neue Längenposition entspricht alter Längenposition zuzüglich der maximalen Länge der platzierten Reihe
            self.main_box_width = 0 # Breitenposition wird zurückgesetzt
            self.main_box_height = 0 # Höhenposition wird zurückgesetzt
            
            # Errechne den neuen Leerraum für die nächste Hauptreihe auf dem Containerboden
            self.main_raw_length = self.main_layer_length - max(raw_length_list) # Verbleibender Leerraum entspricht der Schichtlänge abzüglich der maximalen Länge der platzierten Reihe
            self.main_raw_width = self.container.width # Verfügbare Reihenbreite wird auf Containebreite zurückgesetzt


        return raw_length_list


    def run_Post_Optimization(self, raw_length_list):
        
        # Bestimme minimale Packstückdimension
        # Die minimale Dimension eines Leerraums muss größer oder gleich der minimalen Packstückdimension sein --> Einsparung Rechenzeit
        min_dimension = 1000
        for package in self.rest_packages:
            dimensions = [package[0][2],package[0][3], package[0][4]]
            if min(dimensions) < min_dimension:
                min_dimension = min(dimensions)
                
        # Erzeuge Empty Spaces
        empty_spaces = self.Create_Empty_Spaces(min_dimension)

        # Befülle die einzelnen Leerräume
        for empty_space in empty_spaces:
            
            # Erzeuge den betrachteten Leerraum
            empty_space_width = empty_space[0]
            empty_space_length = empty_space[1]
            empty_space_height = empty_space[2]
            # Setze Leerraumkoordinaten
            empty_space_x = empty_space[3]
            empty_space_z = empty_space[5]
            empty_space_y = empty_space[4]
            
            # Eine Überschreitung der Reihenlänge ist nun nicht mehr erlaubt
            self.factor_overfilling = 1.0 
            
            # Speichere die Packstücke aus der Schicht in einer Liste ab
            articles_in_raw = []
            
            # Erzeuge eine Schicht auf dem Boden des Containers
            raw_length_list = self.Add_Single_Layer (raw_length_list, empty_space_width, empty_space_length, empty_space_x, empty_space_y, empty_space_z)
            
            # Entferne Packstücke aus der einzelnen Schicht von den verbleibenden Packstücken
            # Im Gegensatz zu den Modulen Stack-Building und Add_One_Raw, entfernt Add_Single_Layer die platzierten Packstücke nicht selbstständig          
            self.remove_packed_articles_from_rest_packages()  
            
            # Wiederhole Stapelvorgang
            for article_packed in self.article_locations:
                self.Stack_Building (article_packed)
                
        # Führe finale Stapelung mit zugelassenen Überhängen durch
        for article_packed in self.article_locations:

            self.Modified_Stack_Building (article_packed)


    def remove_packed_articles_from_rest_packages(self):
        # Entferne die platzierten Packstücke aus den verbleibenden Packstücken
        for package_to_remove in self.articles_in_raw:
            self.rest_packages.remove(package_to_remove)



    def Add_One_Raw (self, main_raw_width, main_raw_length, main_box_width, main_box_length, main_box_height):
    
        # Initialisiere Liste für Packstücke in der Hauptreihe
        self.articles_in_raw = []
        
        # Speichere die Längen-Dimensionen der Packstücke einer Reihe in einer Liste ab. 
        # Das Packstück mit der größten Länge gibt die Verschiebung der Längen-Koordinate (box_length) nach Erzeugung der Reihe vor.
        raw_length_list = []
        
        # Beginne mit der Reihenbildung
        for article in self.rest_packages: # Durchlaufe alle Packstücke
                    
            selected_orientation = () # Initialisiere ein Tupel, in welches die Orientierung des potenziell zu platzierenden Packstücks gespeichert wird

            for orientation in article: # Durchlaufe die möglichen Orientierungen eines Packstücks (sortiert nach größtmöglicher Grundfläche)
                                    
                # Überprüfe ob das betrachtete Packstück in der betrachteten Orientierung platziert werden kann
                if (Check_Placement(article, self.articles_in_raw, self.container, self.max_weight, self.carried_weight, orientation, main_raw_width, main_raw_length, main_box_length, main_box_height, self.factor_overfilling) == True):
                    
                    # Speichere die Daten über die Orientierung zwischen, welche potenziell platziert werden kann
                    possible_orientation = (orientation[2], orientation[4], orientation[3], main_box_width, main_box_height, main_box_length, False, orientation[1], orientation[5])
                    # Prüfe ob das Raw Building im Rahmen einer Stapelung durchgeführt wird
                    if (self.call_from_stack_building == True):

                        # Rufe Methode zur Prüfung der Überschreitung der maximalen Traglast auf
                        # Übergebe: Orientierung, Packstücke der Stapelgrundfläche aus width_extension, Packstücke der Stapelgrundfläche aus length_extension, welche Gewichtzuweisung aller gepackten Packstücke enthält
                        # Erhalte: Information, ob Platzierung zulässig
                        placement_allowed = self.Check_Overload_Of_Artcicles(possible_orientation)
                        
                        if (placement_allowed == True): # Prüfe ob die Orientierung platziert werden darf
                            selected_orientation = possible_orientation # Wähle Orientierung als zu platzierende Orientierung aus
                            break # Überprüfe keine weiteren Orientierungen

                    else:
                        # Wähle die Orientierung des potenziell zu platzierenden Packstücks als zu platzierendes Packstück
                        selected_orientation = possible_orientation
                        break # Überprüfe keine weiteren Orientierungen


            # Prüfe ob es eine zulässige Orientierung gibt                
            if len(selected_orientation) != 0:

                # Füge dem Container das Packstück in der selektierten Orientierung hinzu
                self.add_article(selected_orientation[0], selected_orientation[1], selected_orientation[2], selected_orientation[3], selected_orientation[4], selected_orientation[5], selected_orientation[6], selected_orientation[7], self.package_sequence_nr, selected_orientation[8])
                self.package_sequence_nr += 1 # Erhöhe den Zähler für die Packstückreihenfolge um 1
                self.packages_packed.append(article) # Erweitere die platzierten Packstücke
                self.articles_in_raw.append(article) # Erweitere die in der Reihe platzierten Packstücke

                raw_length_list.append(selected_orientation[2]) # Speichere die Packstücklänge in dem dafür vorgesehenen Array
                main_box_width = main_box_width + selected_orientation[0] # Erhöhe die Breitenkoordinate
                main_raw_width = main_raw_width - selected_orientation[0] # Verringere die verfügbare Reihenbreite
                
                # Aktualisiere die erlaubte Reihenlänge, falls noch kein Packstück in der Reihe platziert wurde
                if len(self.articles_in_raw) == 1: 
                    main_raw_length = selected_orientation[2]
                
                # Für alle anderen Packstücke wird der entstehende Leerräume innerhalb der Reihe berechnet und befüllt. Hierbei werden Nebenreihen erzeugt.
                elif len(self.articles_in_raw) > 1:
                    if (main_raw_length - selected_orientation[2]) > 0: # Wenn freie Länge größer Null, dann rufe die Methode zur Befüllung des Leeraums auf
                        
                        side_raw_length = main_raw_length - selected_orientation[2] # Berechne die Länge des entstehenden Nebenleerraums
                        side_raw_width = selected_orientation[0] # Die Breite des entstehenden Nebenleeraums entspricht der Breite des platzierten Packstücks
                        side_box_length = main_box_length + selected_orientation[2] # Startposition für die Längenkoordinate vorgeben
                        side_box_width = main_box_width - selected_orientation[0] # Startposition für Breitenkoordinate vorgeben 
                        side_box_height = main_box_height # Startposition für Höhenkoordinate vorgeben
                        
                        # Leerraum wird mit Nebenreihen befüllt
                        raw_length_list = self.Add_Single_Layer (raw_length_list, side_raw_width, side_raw_length, side_box_width, side_box_length, side_box_height)
        
        # Entferne die platzierten Packstücke aus den verbleibenden Packstücken
        self.remove_packed_articles_from_rest_packages()          
                        
        return raw_length_list         


    def Add_Single_Layer (self, raw_length_list, side_raw_width, side_raw_length, side_box_width, side_box_length, side_box_height):
             
        initial_width = side_raw_width # Setze initiale Breitenkoordinate für Nebenleerraum
        initial_pos_width = side_box_width # Setze initiale Breite für Nebenleerraum
        
        available_length = side_raw_length # Setze initiale Nebenleerraumlänge. Diese wird fortlaufend um die Länge der Nebenreihen reduziert.
        
        layer_completed = False # Variable, die als Abbruchkriterium für das Layer-Building fungiert. Kann keine weitere Nebenreihe erzeugt werden, wird diese auf True gesetzt.
        
        while layer_completed == False:
            
            # Generiere eine Liste, in der die Längen der Packstücke der Nebenreihen gespeichert werden.
            # Diese ist notwendig, sofern ein Packstück die Nebenreihenlänge überschreitet.
            side_length = []
            
            # Durchlaufe die verfügbaren Packstücke
            for article in self.rest_packages:

                selected_orientation = () # Initialisiere ein Tupel, in welches die Orientierung des potenziell zu platzierenden Packstücks gespeichert wird

                for orientation in article: # Durchlaufe die Orientierungen eines Packstücks
                    
                    # Überprüfe ob das betrachtete Packstück in der betrachteten Orientierung platziert werden kann
                    if (Check_Placement(article, self.articles_in_raw, self.container, self.max_weight, self.carried_weight, orientation, side_raw_width, side_raw_length, side_box_length, side_box_height, self.factor_overfilling) == True):
        
                        # Speichere die Daten über die Orientierung zwischen, welche potenziell platziert werden kann
                        possible_orientation = (orientation[2], orientation[4], orientation[3], side_box_width, side_box_height, side_box_length, False, orientation[1], orientation[5])
                        
                        # Prüfe ob das Raw Building im Rahmen einer Stapelung durchgeführt wird
                        if (self.call_from_stack_building == True):
                            
                            # Rufe Methode zur Prüfung der Überschreitung der maximalen Traglast auf
                            # Übergebe: Orientierung, Packstücke der Stapelgrundfläche aus width_extension, Packstücke der Stapelgrundfläche aus length_extension, welche Gewichtzuweisung aller gepackten Packstücke enthält
                            # Erhalte: Information, ob Platzierung zulässig
                            placement_allowed = self.Check_Overload_Of_Artcicles(possible_orientation)
                            
                            if (placement_allowed == True): # Prüfe ob die Orientierung platziert werden darf
                                selected_orientation = possible_orientation # Wähle Orientierung als zu platzierende Orientierung aus
                                break # Überprüfe keine weiteren Orientierungen

                        else:
                            # Wähle die Orientierung des potenziell zu platzierenden Packstücks als zu platzierendes Packstück
                            selected_orientation = possible_orientation
                            break # Überprüfe keine weiteren Orientierungen


                # Prüfe ob es eine zulässige Orientierung gibt                
                if len(selected_orientation) != 0:
                    
                    if selected_orientation[2] > available_length: # Wenn Packstücke die Länge der Hauptreihe überschreitet, füge diese Länge der Liste hinzu
                        length_to_add = side_box_length + selected_orientation[2]
                        raw_length_list.append(length_to_add)

                    self.add_article(selected_orientation[0],selected_orientation[1], selected_orientation[2], selected_orientation[3], selected_orientation[4], selected_orientation[5], selected_orientation[6], selected_orientation[7], self.package_sequence_nr, selected_orientation[8])
                    self.package_sequence_nr += 1 # Erhöhe den Zähler für die Packstückreihenfolge um 1
                    self.articles_in_raw.append(article) # Erweitere die in dem Nebenleerraum platzierten Packstücke
                    self.packages_packed.append(article) # Erweitere die platzierten Packstücke
                    side_length.append(selected_orientation[2]) # Füge die Länge des Packstücks in die Liste der Hauptreihenlänge hinzu
                            
                    side_raw_width = side_raw_width - selected_orientation[0] # Aktualisiere die verfügbare Breite der Nebenreibe
                    side_box_width = side_box_width + selected_orientation[0] # Aktualisiere die Breitenkoordinate für nächstes Packstück
                    
                    # Platziere nächstes Packstück in der Nebenreihe
            
            if len(side_length) > 0:   # Reduziere die Länge der verfügbaren Schicht, falls eine Nebenreihe komplettiert wurde
                available_length = available_length - max(side_length)
            
                # Setze die Länge der neuen Reihe auf die verfügbare Schichtlänge zurück. Aktualisere die Längenkoordinate für neue Nebenreihen.
                side_raw_length = available_length 
                side_box_length = side_box_length + max(side_length)

                # Setze verfügbare Breite für neue Nebenreihen zurück und setze die Breitenkoordinate zurück
                side_raw_width = initial_width
                side_box_width = initial_pos_width
                
            else:  # Wurde keine Packstücklänge in der Liste gespeichert, konnte auch keine weitere Nebenreihe erzeugt werden. Das Layer-Building gilt als abgeschlossen.
                layer_completed = True 
                
                
        return raw_length_list



    def Stack_Building (self, article_packed):
            
        # Füge alle Indexes der Packstücke, die als Basis für einen Stapel fungieren einer Liste hinzu. Hierüber wird später die Variable isStack auf True gesetzt
        stack_indices = []

        # Initialisiere Liste, welche durch die width_extension mit Tupeln befüllt wird, die alle Packstücke um die die Stapelfläche erweitert wurde enthalten. Tupel: (Packstück ID, Packstück Breite, Packstück Länge, Packstück Breitenkoordinate, Packstück Längenkoordinate)
        # Zusätzlich wird dieser Liste ein Tupel des ersten Packstücks der Stapelfläche hinzugefügt
        self.articles_from_width_extension = []
        # Initialisiere Liste, welche durch die length_extension mit Tupeln befüllt wird, die alle Packstücke um die die Stapelfläche erweitert wurde enthalten. Tupel: (Packstück ID, Packstück Breite, Packstück Länge, Packstück Breitenkoordinate, Packstück Längenkoordinate)
        self.articles_from_length_extension = []

        # Initialisiere Variable um zu definieren, ob das die Add_One_Raw Funktion durch eine Stack Building Funktion aufgerufen wird
        self.call_from_stack_building = True

        # Packstücke auf einem Stapel werden nicht übersprungen
        # change_sequence = False
        
        # Keine Überschreitung der Reihenlänge auf einem Stapel zugelassen
        self.factor_overfilling = 1.0
        
        # Wenn ein Packstück noch nicht als Basis für einen Stapel fungierte (isStack = False), dann erzeuge die Stapelfläche
        if article_packed.is_stack == False:
            stack_indices.append(self.article_locations.index(article_packed)) # Füge den Packstückindex der entsprechenden Liste hinzu
            stack_width = article_packed.width # Stapelbreite/Leerraumbreite entspricht der Breite des Packstücks
            stack_height = article_packed.height # Stapelhöhe entspricht der Höhe des Packstücks
            stack_length = article_packed.length # Stapellänge/Leerraumlänge entspricht der Breite des Packstücks
            stack_box_width = article_packed.x # Breitenkoordindate entspricht der Breitenkoordinate des Packstücks
            stack_box_height = article_packed.y + article_packed.height  # Höhenkoordindate entspricht der Höhenkoordinate des Packstücks zuzuglüch der Packstückhöhe 
            stack_box_length = article_packed.z # Längenkoordindate entspricht der Längenkoordinate des Packstücks
            
            # Füge Informationen des ersten Packstücks der Stapelfläche der Liste hinzu. Tupel: (Packstück ID, Packstück Breite, Packstück Länge, Packstück Breitenkoordinate, Packstück Längenkoordinate)
            self.articles_from_width_extension.append((article_packed.package_ID, article_packed.width, article_packed.length, article_packed.x, article_packed.z))
            
            # Generiere größtmögliche Stapelbreite
            stack_width, stack_length, stack_box_height, stack_indices = self.width_extension (stack_indices, stack_width, stack_length, stack_box_width, stack_box_length, stack_box_height)
            # Generiere größtmögliche Stapellänge
            stack_length, stack_indices = self.length_extension (stack_indices, stack_width, stack_length, stack_box_width, stack_box_length, stack_box_height)
        
            '''Führe Stapelvorgang durch'''
            stack_layer_completed = False # Abbruchkriterium für das Stack-Building

            stack_layer_length = stack_length # Initiale Leerraumlänge. Diese wird mit dem Platzieren von Hauptreihen auf dem Stapel fortlaufend reduziert.
            initial_stack_width = stack_width # Initiale Leerraumbreite
            initial_stack_box_width = stack_box_width # Initiale Breitenkoordinate auf dem Stapel

            while stack_layer_completed == False:
                
                rest_packages_before_stacking = len(self.rest_packages) # Variable, welche der Anzahl an verbleibenden Packstücke entspricht.
                
                # Rufe Methode zur Bildung einer Reihe auf
                stack_raw_length_list = self.Add_One_Raw(stack_width, stack_length, stack_box_width, stack_box_length, stack_box_height)
                
                # Wenn keine Packstücke platziert wurden, unterbreche das Stack-Building
                if len(stack_raw_length_list) == 0: 
                    stack_layer_completed = True

                else:
                    stack_box_length = stack_box_length + max(stack_raw_length_list) # Neue Längenkoordinate entspricht alter Längenkoordindate zuzüglich der größten Packstücklänge der platzierten Reihe
                    stack_box_width = initial_stack_box_width # Breitenkoordiante wird auf die initiale Breitenkoordinate gesetzt
                    
                    stack_layer_length = stack_layer_length - max(stack_raw_length_list) # Aktualisiere verfügbare Leerraumlänge
                    stack_length = stack_layer_length # Neue initiale Reihenlänge entspricht der verfügbaren Leerraumlänge
                    stack_width = initial_stack_width # Verfügbare Reihenbreite wird zurückgesetzt
                    
                # Setze die Variable IsStack auf True für Packstücke, die als Basis fungierten. Damit werden keine weiteren Packstücken auf diesen platziert.
                if len(self.rest_packages) < rest_packages_before_stacking:
                    for article_indice in stack_indices: # Durchlaufe hierzu die Indizes der Packstücke, die als Basis für den Stapel fungieren
                        article_to_change = self.article_locations[article_indice] # Wähle das Packstück aus der Lösungskomponente
                        # Setze nun die Variable isSTack auf True
                        article_to_change.set_is_stack_to_true()
            
            '''Stapelung beendet'''
                
        return


    def width_extension (self, stack_indices, stack_width, stack_length, stack_box_width, stack_box_length, stack_box_height):
    
        # Generiere größt mögliche Stapelbreite
        extendible_in_width = True
        while extendible_in_width == True:
            stack_width_old = stack_width # Variable, die der Stapelbreite vor der Erweiterung entspricht
            for article_to_extend in self.article_locations:
                if ((article_to_extend.x >= stack_width + stack_box_width) and # Packstück muss größere Breitekoordinate als betrachtetes Packstück aufweisen
                    (article_to_extend.x <= stack_width + stack_box_width + 5) and # Packstück muss in der Breite nahezu an den Stapel angrenzen
                    (article_to_extend.z ==  stack_box_length) and # Packstück muss dieselbe Längenkoordinate aufweisen
                    (article_to_extend.length <= stack_length + 5) and # Packstück muss in etwa diesselbe Länge aufweisen
                    (article_to_extend.length >= stack_length - 5) and # Packstück muss in etwa diesselbe Länge aufweisen
                    (article_to_extend.y + article_to_extend.height <= stack_box_height + 5) and  # Packstück muss in etwa diesselbe Höhe aufweisen
                    (article_to_extend.y + article_to_extend.height >= stack_box_height - 5) and  # Packstück muss in etwa diesselbe Höhe aufweisen
                    (article_to_extend.is_stack == False)): # Packstücke darf ebenso nicht als Basis für einen Stapel fungiert haben

                        stack_width = stack_width + article_to_extend.width # Falls Bedingungen erfüllt sind, erweitere Stapelbreite
                        
                        # Falls das Packstück ein leicht größere Höhe aufweist, passe die Höhenkoordinate an
                        if (article_to_extend.y + article_to_extend.height > stack_box_height):
                            stack_box_height = article_to_extend.y + article_to_extend.height

                        # Falls das Packstück ein kleinere Länge aufweist, passe die verfügbare Stapellänge an
                        if stack_length < article_to_extend.length:
                            stack_length = article_to_extend.length
                        
                        # Speichere den Index des Packstücks in einer Liste ab. Damit kann die Variable isStack nach dem Stapelvorgang auf True gesetzt werden
                        stack_indices.append(self.article_locations.index(article_to_extend))

                        # Füge Informationen der Liste hinzu, die alle Tupel der Packstücke um die die Stapelfläche erweitert wurde enthalten. Tupel: (Packstück ID, Packstück Breite, Packstück Länge, Packstück Breitenkoordinate, Packstück Längenkoordinate)
                        self.articles_from_width_extension.append((article_to_extend.package_ID, article_to_extend.width, article_to_extend.length, article_to_extend.x, article_to_extend.z)) 

            # Terminiere, sofern die Stapelbreite nicht vergößert werden konnte
            if stack_width == stack_width_old: 
                    extendible_in_width = False
        
        return stack_width, stack_length, stack_box_height, stack_indices


    def length_extension (self, stack_indices, stack_width, stack_length, stack_box_width,stack_box_length,stack_box_height):        
    
        # Es muss eine Menge an Packstücken identifiziert werden, die über die gesamte Breite des Stapels angrenzt
        # Weiterhin muss diese Menge an Packstücke über eine identische Länge verfügen
        extendible_in_length = True
        while extendible_in_length == True:
            width_check = stack_box_width # Variable zur Überprüfung ob die Menge an Packstücken an die gesamte Breite des Stapels angrenzt
            length_check = 0 # Variable zur Überprüfung ob die Menge an Packstücken über diesselbe Länge verfügen
            width_extension = 0
            LDU = 0 # Length Determining Unit: Packstück, welches die Länge der Erweiterung vorgibt

            temporary_indices = [] # Liste zur Speicherung der temporären Packstück-Indexes

            for article_to_extend in self.article_locations:
                if ((article_to_extend.x == width_check) and # Es muss ein Packstück mit der selben Breitenkoordinate geben
                    (article_to_extend.z >= stack_box_length + stack_length) and # Das Packstück muss in etwa in der Länge an den Stapel angrenzen
                    (article_to_extend.z <= stack_box_length + stack_length + 5) and # Das Packstück muss in etwa in der Länge an den Stapel angrenzen
                    (article_to_extend.y + article_to_extend.height <= stack_box_height + 5) and # Packstück muss in etwa diesselbe Höhe aufweisen
                    (article_to_extend.y + article_to_extend.height >= stack_box_height - 5) and # Packstück muss in etwa diesselbe Höhe aufweisen
                    (article_to_extend.is_stack == False)): # Das Packstück darf noch keine Basis eines Stapels darstellen

                        # Das zuerst identifizierte Packstück gibt die Länge des Hilfsstapels vor
                        if LDU == 0:
                            length_check = article_to_extend.length
                            LDU = 1

                        # Weist das Packstück  ein passende Länge auf, wird der Hilftstapel erweitert
                        if ((article_to_extend.length <= length_check + 5) and
                            (article_to_extend.length >= length_check - 5)):

                            width_check = width_check + article_to_extend.width # Erweitere die Breitenkoordinate des Hilfsstapels, damit weitere Packstücke zur Erweiterung in der Breite identifziert werden können
                            width_extension = width_extension + article_to_extend.width # Erweitere die Breite des Hilfsstapels
                            temporary_indices.append(self.article_locations.index(article_to_extend)) # Füge den Index den temporären Stapelindizes hinzu
                            if article_to_extend.length > length_check: # Passe die Länge des Hilfsstapels an, sofern die Hilfsstapellänge durch das Packstück im Rahmen der Toleranz überschritten wird
                                length_check = article_to_extend.length
                            
                            # Füge Informationen der Liste hinzu, die alle Tupel der Packstücke um die die Stapelfläche erweitert wurde enthalten. Tupel: (Packstück ID, Packstück Breite, Packstück Länge, Packstück Breitenkoordinate, Packstück Längenkoordinate)
                            self.articles_from_length_extension.append((article_to_extend.package_ID, article_to_extend.width, article_to_extend.length, article_to_extend.x, article_to_extend.z)) 

            # Konnte eine Menge an Packstücke gefunden werden, welche über die gesamte Stapelbreite angrenzt, wird die endgültige Erweiterung der Stapelfläche durchgeführt
            # Der Stapel wird dann um die Länge des Hilfsstapels erweitert
            if ((width_extension >= stack_width - 5 )and 
                (width_extension <= stack_width + 5 )):
                    stack_length = stack_length + length_check
                    for indice in temporary_indices:
                        stack_indices.append(indice)     
            else: # Unterbreche, sofern die Breite des Hilfsstapels nicht der Breite des zu erweiternden Stapels entspricht
                break
            
            # Wurde kein passendes Packstück identiziert, unterbreche die Erweiterung
            if len(temporary_indices)==0:
                break
                        
        return stack_length, stack_indices



    def Create_Empty_Spaces(self, min_dimension):
    
        empty_space_pos_width = [] # Liste zur Speicherung der initialen Breitenposition aller Leerräume
        empty_space_pos_length = [] # Liste zur Speicherung der initialen Längenposition aller Leerräume
        empty_space_pos_height = 0 # Initiale Höhenposition der Leerräume = 0, da die Leerräume auf dem Boden des Containers gebildet werden
        empty_space_width = [] # Liste zur Speicherung der Länge aller Leerräume
        empty_space_length = [] # Liste zur Speicherung der Breite aller Leerräume
        empty_space_height = self.container.height # Höhe aller Leerräume entspricht der Containerhöhe
        empty_spaces = [] # Initialisierung Leerraumliste
        
        # Es wird eine Hilfsvariable definiert, welche der Längenposition einer Hauptreihe entspricht. Damit können Hauptreihen voneinander unterschieden werden.
        current_length = 0
        # Die erste Hauptreihe und damit auch der entsprechende reihenübergreifende Leeraum fangen bei der Längenposition 0 an
        # Diese wird der entsprechenden Liste hinzugefügt
        empty_space_pos_length.append(current_length)
        
        # Es wird eine Variable für die Anzahl der Haupreihen definiert
        number_of_walls = 1 
        
        # Bestimme Anzahl an Walls. Für jede Wall wird ein reihenübergreifender Leeraum berechnet.
        for article in self.article_locations:
            if ((article.y == 0)and # Höhenposition = 0: Wall muss einen Boden haben
                (article.x == 0)and # Breitenposition = 0: Wall muss einen Anfang haben
                (article.z != current_length)): # Längenposition muss sich von der Längenposition des vorherigen Packstücks unterscheiden
                number_of_walls = number_of_walls + 1 # Erhöhe Anzahl der Hauptreihen
                current_length = article.z # Aktualsiere die Längenposition bzw. die Hilfsvariable
                empty_space_pos_length.append(current_length) # Füge die Längenposition des Leerraums in die entsprechende Liste hinzu
        
        # Nun ist die Anzahl an Walls definiert. Weiterhin wurde für jeden reihenübergreifenden Leerraum dessen initiale Längenposition ermittelt.
        # Nun soll die initiale Breitenposition errechnet und der entsprechenden Liste hinzugefügt werden
        # Die initiale Breitenposition der reihenübergreifenden Leerräume entspricht dem Ende der Hauptreihen
        # Durchlaufe hierzu die Packstücke aus denselben Hauptreihen
        for i in range(number_of_walls):
            column_end = 0 # Hilfsvariable column_end wird zur Berechnung der initialen Breitenposition pro Leerraum verwendet. Initialisierung auf 0 für jede Hauptreihe.
            for article in self.article_locations:
                if ((article.y == 0)and # Höhenposition des Packstücks muss 0 sein
                    (article.z == empty_space_pos_length[i])and # Die Längenposition von Packstücken innerhalb einer Hauptreihe muss übereinstimmen
                    (article.x + article.width > column_end)): # Existiert ein Packstück, welches das Ende der Hauptreihe in der Breite überschreitet?
                        column_end = article.x + article.width # Falls ja, aktualisiere das Ende der Hauptreihenbreite
            empty_space_pos_width.append(column_end)
        
        # Jetzt sind die Initialkoordinaten für sämtliche reihenübergreifende Leerräume definiert
        # Als nächstes werden die Länge und Breite der Leerräume den entsprechenden Listen hinzugefügt
        # Durchlaufe hierzu erneut die Hauptreihen
        for i in range(number_of_walls):
            
            end = False # Hilfsvariable, welche angibt, ob das Ende eines Leerraums in der Länge erreicht ist
            
            # Verfügbare Leerraumbreite entspricht der Containerbreite abzüglich der initialen Breitenposition des Leerraums
            space_width = self.container.width - empty_space_pos_width[i]
            empty_space_width.append(space_width) # Füge die Breite des Leerraums in die entsprechende Liste hinzu
            
            # Bestimmen nun die maximale Leerraumlänge
            # Durchlaufe hierzu die bereits platzierten Packstücke und schaue, ob ein Packstück den Leerraum in der Länge durchkreuzt
            for article in self.article_locations:
                if ((article.z > empty_space_pos_length[i]) and # Ein solches Packstück muss über eine größere initiale Längenposition als der Leerraum verfügen
                    (article.y == 0 ) and #Ein solches Packstück muss sich auf dem Boden des Containers befinden
                    (article.y + article.width > empty_space_pos_width[i])): # Ein solches Packstück muss den Leerraum in der Breite schneiden. Breitenpositon + Breite des Packstücks > Breitenposition des Leerraums
                        space_length = article.z - empty_space_pos_length[i] # Falls ein Packstück durchkreuzt, errechnet sich die Leerraumlänge aus der Längenposition des identifzierten Packstücks abzüglich der intialen
                                                                            # Längenposition des Leerraums
                        empty_space_length.append(space_length) # Füge die Leerraumlänge der entsprechenden Liste hinzu
                        end = True # Ende des Leeraums ist erreicht
                        break
            
            # Wurde kein Packstück identifiziert, welches den Leerraum in der Länge durchkreuzt, so reicht die Leerrlänge bis zur Containerwand
            if end == False:
                space_length = self.container.length - empty_space_pos_length[i] # Leerraumlänge = Containerlänge - initiale Längenposition des Leerraums
                empty_space_length.append(space_length) # Füge die Leerraumlänge der entsprechenden List hinzu
        
        # Füge die Leerräume mit den enstprechenden Informationen (Dimensionen, Koordianten) der Leerraumliste hinzu
        for i in range(len(empty_space_pos_width)):
                if ((empty_space_width[i] > min_dimension) and # Länge und Breite des Leerraums müssen größer sein als die minimale Packstückdimension
                    (empty_space_length[i] > min_dimension)): 
                        empty_spaces.append((empty_space_width[i], empty_space_length[i], empty_space_height ,empty_space_pos_width[i], empty_space_pos_length[i], empty_space_pos_height))
        
        # Überprüfe ob es Schnittpunkte gibt
        # Führe hierzu paarweise Vergleiche zwischen den Leerräumen durch
        
        abbruch = False # Hilfsvariable, die als Abbruchkriterium fungiert
        while abbruch == False:
            
            length_empty_spaces_old = len(empty_spaces) # Hilfsvariable, welche der Anzahl an Leerräumen entspricht. Damit wird das Abbruchkriterium beeinflusst. 
            # Durchlaufe die Leerräume
            for i in range(len(empty_spaces)-1):
                # Prüfe zunächst, ob der betrachtete Leerraum in der Länge den darauffolgenden Leerraum durchkreuzt
                if (empty_spaces[i][1] + empty_spaces[i][4] > empty_spaces[i+1][4]): # Es musss gelte: Längenposition des Leerraums + Länge des Leerraums > Längenposition des nächsten Leerraums
                    # Falls Leerraum in der Längen durchkreuzt, prüfe welcher Leerraum die größere Grundfläche und damit das größere Volumen aufweist
                    # Grundfläche errechnet durch Leerraumbreite * Leerraumlänge
                
                    # Entferne den nachfolgenden Leerraum, falls dieser eine kleinere Grundfläche als der betrachtete Leerraum aufweist
                    if empty_spaces[i][0]*empty_spaces[i][1] > empty_spaces[i+1][0]*empty_spaces[i+1][1]: 
                        empty_spaces.remove(empty_spaces[i+1]) 
                        break
                        
                    # Bei Gleichstand: Selektiere den Leerraum mit der größeren Länge
                    elif empty_spaces[i][0]*empty_spaces[i][1] == empty_spaces[i+1][0]*empty_spaces[i+1][1]:
                        if empty_spaces[i][1] > empty_spaces[i+1][1]:
                            empty_spaces.remove(empty_spaces[i+1])
                            break
                        else:
                            empty_spaces.remove(empty_spaces[i])
                            break
                            
                    # Entferne den betrachteten Leerraum, falls dieser eine kleinere Grundfläche als der nachfolgende Leerraum aufweist
                    else:
                        empty_spaces.remove(empty_spaces[i])
                        break
                        
            # Unterbreche, falls keine weiteren Überschneidungen existieren
            if len(empty_spaces) == length_empty_spaces_old:
                abbruch = True
                                    
        return empty_spaces # Gebe sämtliche Leerräume zurück


    def Modified_Stack_Building (self, article_packed):
        global packplan
        packplan = []
        stack_indices = [] # Liste zur Speicherung der Indizes der Packstücke, die als Basis für den Stapel fungieren

        # Initialisiere Liste, welche durch die width_extension mit Tupeln befüllt wird, die alle Packstücke um die die Stapelfläche erweitert wurde enthalten. Tupel: (Packstück ID, Packstück Breite, Packstück Länge, Packstück Breitenkoordinate, Packstück Längenkoordinate)
        self.articles_from_width_extension = []
        # Initialisiere Liste, welche durch die length_extension mit Tupeln befüllt wird, die alle Packstücke um die die Stapelfläche erweitert wurde enthalten. Tupel: (Packstück ID, Packstück Breite, Packstück Länge, Packstück Breitenkoordinate, Packstück Längenkoordinate)
        self.articles_from_length_extension = []

        if article_packed.is_stack == False:

            stack_indices.append(self.article_locations.index(article_packed))
            stack_width = article_packed.width # Stapelbreite entspricht der Breite des Packstücks
            stack_height = article_packed.height # Stapelhöhe entspricht der Höhe des Packstücks
            stack_length = article_packed.length # Stapellänge entspricht der Breite des Packstücks
            stack_box_width = article_packed.x # Breitenkoordindate entspricht der Breitenkoordinate des Packstücks
            stack_box_height = article_packed.y + article_packed.height  # Höhenkoordindate entspricht entspricht der Höhenkoordinate des Packstücks zuzuglüch der Packstückhöhe 
            stack_box_length = article_packed.z # Längenkoordindate entspricht der Längenkoordinate des Packstücks

            # Füge Informationen des ersten Packstücks der Stapelfläche der Liste hinzu. Tupel: (Packstück ID, Packstück Breite, Packstück Länge, Packstück Breitenkoordinate, Packstück Längenkoordinate)
            self.articles_from_width_extension.append((article_packed.package_ID, article_packed.width, article_packed.length, article_packed.x, article_packed.z))

            # Generiere größtmögliche Stapelbreite
            stack_width, stack_length, stack_box_height, stack_indices = self.width_extension (stack_indices, stack_width, stack_length, stack_box_width, stack_box_length, stack_box_height)
            # Generiere größtmögliche Stapellänge
            stack_length, stack_indices = self.length_extension (stack_indices, stack_width, stack_length, stack_box_width, stack_box_length, stack_box_height)

            # Führe Stapelvorgang durch. Jetzt jedoch mit zugelassenen Überhängen.
            stack_completed = False
            while stack_completed == False:

                stack_raws = []  # Liste zur Speicherung der platzierten Packstücke
                stack_raw_length_list = []  # Liste zur Speicherung der Längen platzierter Packstücke
    
                stack_raw_width = stack_width  # Initiale Stapelbreite bzw. Breite des Leerraums
                
                # Durchlaufe die verbleibenden Packstücke
                for article_to_pack in self.rest_packages:                  
                    selected_orientation = () # Initialisiere ein Tupel, in welches die Orientierung des potenziell zu platzierenden Packstücks gespeichert wird
                    for orientation in article_to_pack: # Durchlaufe die möglichen Orientierungen eines Packstücks (sortiert nach größtmöglicher Grundfläche)

                        # Prüfe ob das Packstück auf den Stapel passt & keine anderen Packstücke überschneidet
                        if (Check_Placement_Modified_Stacking (self.container, self.max_weight, self.carried_weight, self.article_locations, orientation, stack_raw_width, stack_length, stack_box_length, stack_box_height, stack_box_width) == True):

                            # Falls  Bedingungen eingehalten werden, füge Orientierung der Liste hizu
                            possible_orientation = (orientation[2], orientation[4], orientation[3], stack_box_width, stack_box_height , stack_box_length, False, orientation[1], orientation[5])
                            
                            # Rufe Methode zur Prüfung der Überschreitung der maximalen Traglast auf
                            # Übergebe: Orientierung, Packstücke der Stapelgrundfläche aus width_extension, Packstücke der Stapelgrundfläche aus length_extension, welche Gewichtzuweisung aller gepackten Packstücke enthält
                            # Erhalte: Information, ob Platzierung zulässig
                            placement_allowed = self.Check_Overload_Of_Artcicles(possible_orientation)

                            if (placement_allowed == True): # Prüfe ob die Orientierung platziert werden darf
                                selected_orientation = possible_orientation # Wähle Orientierung als zu platzierende Orientierung aus
                                break # Überprüfe keine weiteren Orientierungen

                    # Prüfe ob es eine zulässige Orientierung gibt
                    if len(selected_orientation) != 0:

                        # Erweitere die Lösungskomponente
                        self.add_article(selected_orientation[0],selected_orientation[1], selected_orientation[2], selected_orientation[3], selected_orientation[4], selected_orientation[5], 
                                                    selected_orientation[6], selected_orientation[7], self.package_sequence_nr, selected_orientation[8])
                        self.package_sequence_nr += 1
                        self.packages_packed.append(article_to_pack) # Erweitere die gepackten Packstücke
                        stack_raws.append(article_to_pack) # Erweitere die Packstückliste des Stapels 

                        stack_raw_length_list.append(selected_orientation[2])  # Füge die Packstücklänge in die entsprechende Liste hinzu
                        stack_raw_width = stack_raw_width - selected_orientation[0] # Reduziere die verfügbare Reihenbreite
                        stack_box_width = stack_box_width + selected_orientation[0] # Akualsiere die Breitenkoordinate
                
                # Entferne die auf dem Stapel platzierten Packstücke von den verbleibenden Packstücken
                for article_to_remove in stack_raws:
                    self.rest_packages.remove(article_to_remove)

                # Setze die Variable IsStack auf True für Packstücke, die als Basis fungierten. Damit werden keine weiteren Packstücken auf diesen platziert.
                if len(stack_raws) > 0:
                    for article_indice in stack_indices:
                        article_to_change = self.article_locations[article_indice]
                        article_to_change.set_is_stack_to_true()
                
                # Falls keine Packstücke auf dem Stapel platziert wurden, unterbreche das Modified Stack-Building
                if len(stack_raws) == 0: 
                    stack_completed = True
                else: # Falls doch, passe die Längenkoordinate, die Breitenkoordinate und die Leerraumlänge an
                    stack_box_length = stack_box_length + max(stack_raw_length_list)
                    stack_box_width = article_packed.x
                    stack_length = stack_length - max(stack_raw_length_list)

        for artloc in self.article_locations:
            packplan_package = [artloc.width, artloc.height, artloc.length, artloc.x, artloc.y, artloc.z, artloc.is_stack, artloc.package_ID, artloc.package_sequence_nr, artloc.package_weight]
            packplan.append(packplan_package)
   
    ''' Ende Funktionen des Single Knapsack Algortihmus'''



    '''Funktionen zur Visualisierung'''
    def get_lineset(self, offset_x=0, offset_y=0, offset_z=0):
        points = [[0+offset_x, 0+offset_y, 0+offset_z],
                  [self.container.width+offset_x, 0+offset_y, 0+offset_z],
                  [0+offset_x, self.container.height+offset_y, 0+offset_z],
                  [self.container.width+offset_x, self.container.height+offset_y, 0+offset_z],
                  [0+offset_x, 0+offset_y, self.container.length+offset_z],
                  [self.container.width+offset_x, 0+offset_y, self.container.length+offset_z],
                  [0+offset_x, self.container.height+offset_y, self.container.length+offset_z],
                  [self.container.width+offset_x, self.container.height+offset_y, self.container.length+offset_z]]

        lines = [[0, 1], [0, 2], [1, 3], [2, 3], [4, 5], [4, 6], [5, 7], [6, 7], [0, 4], [1, 5], [2, 6], [3, 7]]
        colors = [[0, 0, 0] for i in range(len(lines))]

        line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(points), lines=o3d.utility.Vector2iVector(lines))
        line_set.colors = o3d.utility.Vector3dVector(colors)    
        return line_set

    def save_packplan(self):
        self.timestamp = datetime.now().strftime("%Y-%m-%d %Hh%Mm%Ss")
        file_path_packplan = 'Simulationen//' + str(self.timestamp) + ' BI01' + ' Simulation Packplan.csv'
        pckpln = []
        for artloc in self.article_locations:
            package = [artloc.width, artloc.height, artloc.length, artloc.x, artloc.y, artloc.z, artloc.is_stack, artloc.package_ID, artloc.package_sequence_nr, artloc.package_weight]
            pckpln.append(package)
        
        results_packplan = pd.DataFrame(pckpln)
        
        # Speichere den Packplan global ab
        Packplan.set_packplan(packplan)
        
        pckpln = []
        # results_packplan.to_csv(path_or_buf=file_path_packplan, header=False, index=False)

    def save_container(self):
        self.timestamp = datetime.now().strftime("%Y-%m-%d %Hh%Mm%Ss")
        file_path_container = 'Simulationen//' + str(self.timestamp) + ' BI01' + ' Simulation Container.csv'
        container = []
        # print(self.container[0], self.container[1], self.container[2])
        cntnr = [self.container[0], self.container[1], self.container[2]]
        container.append(cntnr)
        results_container = pd.DataFrame(container)

        container_transfer.set_container(container)

        container = []
        # results_container.to_csv(path_or_buf=file_path_container, header=False, index=False)


    
    def get_geometry(self, offset_x=0, offset_y=0, offset_z=0):
        line_set=self.get_lineset(offset_x, offset_y, offset_z)
        
        geo_list=[line_set]
        for artloc in self.article_locations:
            mesh_box = o3d.geometry.TriangleMesh.create_box(width=artloc.width, height=artloc.height, depth=artloc.length)
            mesh_box.translate(np.array([artloc.x+offset_x,artloc.y+offset_y,artloc.z+offset_z]))   
            mesh_box.compute_vertex_normals()

            rvalue=random.random()
            mesh_box.paint_uniform_color([rvalue, 1-rvalue, random.random()])    
            geo_list.append(mesh_box)
        return geo_list
    
    def __repr__(self):
        return str(self.container) + "\n    " + "\n    ".join(str(x) for x in self.article_locations)
    '''Ende Funktionen zur Visualisierung'''