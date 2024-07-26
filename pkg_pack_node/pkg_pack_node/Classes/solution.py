import open3d as o3d
from Classes.container import Container
import numpy as np
import os

class Solution:
    
    def __init__(self, modified_order_data, container_data):
        self.containers=[] # Liste der Container, aus der die Lösung besteht
        self.container_number = 0 # Anzahl der Container, aus der die Lösung besteht
        self.usage_average = 0 # durschnittliche Containerauslastung der Lösung
        self.costs = 0 # Summe der Kosten aller Container in der Lösung

        # Variablen des Greedy Verfahrens
        self.modified_order_data = modified_order_data
        self.container_data = container_data

        self.packages_packed_list = [] # Liste in der die platzierten Packstücke aller Containertypen abgespeichert werden
        self.container_to_add_list = [] # Liste in der die einzelnen Lösungskomponenten bzw. Instanzen abgespeichert werden
        self.usage_list = [] # Liste in der die Auslastungen aller Containertypen abgespeichert werden

        self.index_selected_container = 0 # Index des gewählten Containers
        self.usage_selected = 0 # Volumenauslastung des gewählten Containers


    # Füge einen übergebenen Container der Lösung hinzu
    def add_container(self, container):
        self.containers.append(container) # Füge Container der Lösung hinzu
        self.costs += container.costs # Erhöhe die Kosten der Lösung
        self.container_number += 1 # Erhöhe die Anzahl der Container in der Lösung

    # Berechne die durschnittliche Volumenauslastung der Lösung
    def set_usage_average(self):
        usage_total = 0 # Initialisieren Volumenauslastung

        for container in self.containers:
            usage_total += container.volume_usage_rate # Summiere Volumenauslastung aller Container in der Lösung auf
        
        self.usage_average = usage_total/len(self.containers) # Teile Summe durch Containeranzahl

    '''Funktionen für das Greedy Verfahren'''
    def calculate_solution (self):
        # Packe einen neuen Container, solange verbleibende Packstücke existieren
        while len(self.modified_order_data) > 0:

            
            # Entferne die Daten aus den Listen
            self.packages_packed_list = []
            self.container_to_add_list = []
            self.usage_list = []

            # Wende den SKA auf jeden Container-Typ an
            global rest_packages
            rest_packages = self.pack_each_container_type()

            # Nun liegen die Ergebnisse für jeden Containertyp vor
            # Das Greedy-Verfahren selektiert nun den Containertyp mit der höchsten Volumenauslastung
            self.index_selected_container = self.usage_list.index(max(self.usage_list)) # Index des Containertyps mit der höchsten Volumenauslastung, bei mehreren gleichen Werten wählt max() in Python den Index des ersten Wertes
            self.usage_selected = self.usage_list[self.index_selected_container] # Auslastung des Containertyps mit der höchsten Volumenauslastung

            
            # Prüfe, ob mehrere Container mit der selben Auslastung existieren, falls ja: wähle den größeren
            self.check_for_same_volume_usage_and_select_bigger()
                             
            # Prüfe, ob es ein Container existiert, welcher alle verbleibenden Packstücke packt und wähle diesen falls die in der Funktion beschriebenen Voraussetzungen erfüllt sind
            self.last_container_optimization(rest_packages)
                
            # Selektiere nun die Lösungskomponente, die gepackten Packstücke, die Containerauslastung und das gepackte Gewicht
            packages_packed_selected = self.packages_packed_list [self.index_selected_container]
            container_to_add_selected = self.container_to_add_list [self.index_selected_container]
            self.usage_selected = self.usage_list [self.index_selected_container]
            
            # Füge die Lösungskomponente der Lösung hinzu
            self.add_container(container_to_add_selected)

            # Die Packstücke welcher in der selektierten Instanz gepackt wurden, müssen aus modifizierten Bestelldaten entfernt werden
            for package in packages_packed_selected:
                self.modified_order_data.remove(package)
        
        # Errechne die durchschnittliche Containerauslastung der Lösung
        self.set_usage_average()


    # Prüfe, ob mehrere Container mit der selben Auslastung existieren, falls ja: wähle den größeren
    def check_for_same_volume_usage_and_select_bigger(self):
        # Durchlaufe dazu die Liste mit den gespeicherten Containerauslastungen
        for index in range(len(self.usage_list)):
            if self.usage_list[index] == (self.usage_selected): # Überprüfe für jeden Container, ob er die selbe Auslastung hat, wie der Container mit der höchsten Auslastung
                if (self.container_to_add_list[index].container.width * self.container_to_add_list[index].container.height * self.container_to_add_list[index].container.length) > \
                    (self.container_to_add_list[self.index_selected_container].container.width * self.container_to_add_list[self.index_selected_container].container.height * self.container_to_add_list[self.index_selected_container].container.length): # Überprüfe bei gleicher Auslastung, ob der bisher selektierte Container kleiner ist
                    self.index_selected_container = index # Ändere den Index des selektierten Containers, falls der bisher selektierte kleiner war
                    self.usage_selected = self.usage_list[self.index_selected_container] # Ändere die Auslastung des selektierten Containers


    # Wende den SKA auf jeden Container-Typ an
    def pack_each_container_type(self):
        global rest_packages
        for container_type in self.container_data:
            global rest_packages

            # Sicherstellung, dass jedem Containertyp diesselben Packstücke übergeben werden
            rest_packages = [package for package in self.modified_order_data]
            
            # Anwendung SKA zur Generierung einer Lösungskomponente
            container_to_add = Container(container_type[0], container_type[2], container_type[1], container_type[3], container_type[4], rest_packages)
            packages_packed = container_to_add.pack_container()

            container_to_add.set_volume_usage_rate()# Berechne Containerauslastungsrate der Lösungskomponente

            # Füge die Ergebnisse für den betrachteten Containertyp den erzeugten Listen hinzu
            # Aber nur, falls mindestens ein Packstück platziert wurde
            if container_to_add.volume_used != 0:
                
                self.packages_packed_list.append(packages_packed) # Erweitere die Liste in der die platzierten Packstücke aller Containertypen abgespeichert werden
                self.container_to_add_list.append(container_to_add) # Erweitere die Liste in der die einzelnen Lösungskomponenten bzw. Instanzen abgespeichert werden
                self.usage_list.append(container_to_add.volume_usage_rate) # Erweitere die Liste in der die Auslastungen aller Containertypen abgespeichert werden

        return rest_packages
    
    # Prüft, ob es ein Container existiert, welcher alle verbleibenden Packstücke packt und wählt diesen falls die in der Funktion beschriebenen Voraussetzungen erfüllt sind
    def last_container_optimization(self, rest_packages):
            
            # Prüfe zunächst, ob Container existieren, welche alle restlichen Packstücke packen & ermittle daraus den Container mit der höchsten Auslastung
            container_rest_packages_volume_usage_rate = 0 # Initialisiere die Auslastung des Container, der alle restlichen Packstücke packt
            for index, container_packages in enumerate(self.packages_packed_list):
                if (len(container_packages) == len(self.modified_order_data)) and (self.container_to_add_list[self.index_selected_container].volume_usage_rate > container_rest_packages_volume_usage_rate):
                    container_rest_packages_volume_usage_rate = self.container_to_add_list[index].volume_usage_rate # Auslastung des Containers der alle restlichen packstücke packt & die höchste Auslastung hat
                    index_container_all_rest_packages = index # Index des Containers der alle restlichen packstücke packt & die höchste Auslastung hat

                    # Es gibt einen Container, der alle restlichen Packstücke packt

                    # Berechne das Volumen der nicht durch den aktuell selektierten Container gepackten Packstücke
                    remaining_articles = [article for article in rest_packages if article not in self.packages_packed_list [self.index_selected_container]] # Erzeuge eine Liste, die alle Packstücke enthält die durch den selektierten Container nicht gepackt wurden
                    volume_of_remaining_articles = 0 # Initialisiere das Gesamtvolumen der nicht gepackten Packstücke
                    for article in remaining_articles:
                        volume_of_remaining_articles += article[0][6] # Summiere die Volumen der Packstücke

                
                    # Nun wird ermittelt, ob der Container der alle restlichen Packstücke anstatt dem bisher selektierten Container verwendet werden soll
                    # Prüfe, ob es einen Containertypen gibt, der alle restlichen Packstücke tragen kann & dessen Ausalstung verrechnet mit dem aktuell selektierten Container im Mittel höher ist, als die Auslastung des Containers, der alle restlichen Packstücke packen kann
                    use_container_all_rest_packages = True # Initialisiere die Verwendung des Containers der alle restlichen Packstücke packt mit True
                    for container in self.container_data:
                        container_volume = container[0]*container[1]*container[2]/1000

                        # Prüfe, ob der Conrtainertyp ein größeres Volumen als alle restlichen Packstücke hat & ob die durchschnittliche Auslastung aus dem bisher selektierten Container und dem nächsten Container (befüllt mit allen Restartikeln) größer ist, als die Auslastung des Containers, der alle restlichen Packstücke packt
                        if volume_of_remaining_articles/container_volume < 1 and \
                            (self.container_to_add_list[self.index_selected_container].volume_usage_rate + volume_of_remaining_articles/container_volume) / 2 > self.container_to_add_list[index_container_all_rest_packages].volume_usage_rate:
                            # Trifft die beiden Bedingungen zu soll der Container der alle restlichen Packstücke tragen kann nicht verwendet werden
                            use_container_all_rest_packages = False

                    if (use_container_all_rest_packages): # Prüfe, ob der Container der alle restlichen Packstücke tragen kann verwendet werden soll
                        self.index_selected_container = index_container_all_rest_packages # Verwende den Container, der alle restlichen Packstücke packt als Lösungskomponente
    '''Ende der Funktionen für das Greedy Verfahren'''
    def print_packplan(self):
        for container in self.containers:
            container.save_packplan()
            container.save_container()


    # Visualisiere die Container in der Lösung
    def visualize(self):

        vis = o3d.visualization.Visualizer()
        vis.create_window(visible=False)
        offset_x=0
        offset_x_delta=100
        
        for container in self.containers:
            for geo in container.get_geometry(offset_x=offset_x):
                vis.add_geometry(geo)
            
            offset_x+=offset_x_delta
            offset_x+=container.container.width
      
        # Hole die ViewControl-Instanz
        view_control = vis.get_view_control()

        # Setze die gewünschte Ansicht
        view_control.set_front([1.2, 2.0, -1.5])  # Blickrichtung
        view_control.set_up([0.0, 1.0, 0.0])      # Aufwärtsrichtung
        view_control.set_zoom(0.8)                # Zoomstufe

        # Render das Bild
        vis.poll_events()
        vis.update_renderer()

        # Erhalte den Bildschirmpuffer als Float-Puffer
        float_buffer = vis.capture_screen_float_buffer(do_render=True)

        # Konvertiere den Float-Puffer in ein numpy-Array und dann in ein Bild
        image_array = np.asarray(float_buffer)
        image_array = (image_array * 255).astype(np.uint8)
        image = o3d.geometry.Image(image_array)
        
        vis.destroy_window()

        # Speichere das Bild
        o3d.io.write_image("solution_screenshot.png", image)


    def __repr__(self):
        
        return "packing solution\n  " + "\n  ".join(str(x) for x in self.containers)