""" 
3D Packalgorithmus

Version 2.0

Version 1.0
Autor: Dominik Lipfert
Studiengang:Wirtschaftsingenierwesen Master
basierend auf dem Packalgorithmus von Reiplinger (2021)

Version 1.1 Änderungen Luisa Schlenker

Version 2.0 Umwandlung in Pack_Algorithm.py, Implementierung mit Hardware
"""

"""
Gliederung

0. Klassen des Packalgorithmus
 - Class: Article
 - Class: Container
 - Class: Solution (=Individuum)
 - Class: Genetic Algorithm
 - Class: Packing Algorithm
 - Class: Grid Search
1. Datenvorverarbeitung 
2. Restriktionen in Cython
 - Platzierungsbedingungen
 - Bedingung für Traglast einzelner Packstücke
3. Anwendung & Ergebnisausgabe
"""

import collections
import numpy as np
import open3d as o3d
from IPython.core.display import display, HTML
import random

import pandas as pd
import itertools
import time
import math
import copy
import matplotlib.pyplot as plt
from functools import reduce
from operator import add
import csv
from datetime import datetime
from IPython.core.display import display, HTML

import copy
import numpy as np
import time
import pandas as pd
import os
from Classes.packplan import Packplan
from Classes.container_transfer import container_transfer

#%load_ext Cython

display(HTML("<style>.container { width:100% !important; }</style>"))

# 0. Klassen des Packalgorithmus
# Class: Article
from Classes.article import Article

# Class: Container
from Classes.container import Container
# Class: Solution (=Individual)
from Classes.solution import Solution

# Class: Population
from Classes.population import Population

# Class: Genetic Algorithm
import itertools
from Classes.genetic_algorithm import Genetic_Algorithm




# Class: Packing Algorithm
from Classes.packing_algorithm import Packing_Algorithm
from Classes.packplan import Packplan
from Classes.container_transfer import container_transfer


# Class: Grid Search
#import multiprocessing as mp
# Klasse, in der verschiedene Ausprägungen der Parameter für den evolutionären Algorithmus definiert werden können. 

from Classes.grid_search import Grid_Search

# 1. Datenvorverarbeitung
 

# 2. Restriktionen in Cython

# 2.1 Platzierungsbedingung


from cython_files.cython_part import Check_Placement_Modified_Stacking
from cython_files.cython_part import Check_Placement
from cython_files.cython_part import Calculate_Overlapping_Areas

# 3. Anwendung und Ergebnisausgabe

from Classes.order_data_transfer import order_data_transfer
from Classes.items_transfer import items_transfer

# # Ausführen des Algorithmus mit Line-by-Line-Profiler 
# %load_ext line_profiler

# # Erzeuge eine Instanz des Algorithmus mit den gewünschten Paramtern (Dateiname der Bestelldaten, Populationsgröße, Variante der Paarungsselektion, Variante des Crossover, Anzahl zu übernehmende beste Individuen in Folgepopulation)
# packing_algorithm = Packing_Algorithm("06_Datensatz_50-75 Artikel_leicht_heterogen2", 10, 10, 0, [0], 1)
# # Ruf Algorithmus mit Profiler auf und definiere für welche Funktion die Zeit gemessen werden soll
# %lprun -f Container.run_Wall_Building packing_algorithm.run_algorithm()

class Pack_Algorithm:
    # Materialstamm
    material_master = {
        "Label ODTF": ["Box_Gluehlampe", "a", "b", "c", "d", "Box_Wischblatt", "e", "Keilriemen_gross", "f", "g", "Box_Bremsbacke", "h", "Keilriemen_klein", "Tuete", "Box_Messwertgeber"],
        "Materialnummer": ["0250201042EAF", "F009D02804810", "02215044708SE", "F026400391HWS", "043327177341N", "3397004629FUP", "93203350523CT", "198794847900N", "0986580826FP5", "0986580371KM9", "0204114675EE9", "3397009843HR1", "1987947949KM1", "1928402070000", "94436128953CA"],
        "Label Bosch": ["Gluhstiftkerze", "Vakuumpumpe", "Zundspule", "Luftfiltereinsatz", "Lochduse", "Wischblatt", "Fanfare", "Keilrippenriemen", "Ekp-Einbaueinheit", "Elektrokraftstoffpumpe", "Trommelbremsbackensatz", "Wischblattsatz", "Keilrippenriemen", "Steckergehause", "Messwertgeber"],
        "Länge [mm]": [130, 195, 215, 161, 84, 364, 475, 510, 285, 255, 238, 715, 265, 260, 85],
        "Breite [mm]": [104, 177, 66, 161, 78, 150, 129, 125, 235, 105, 218, 272, 85, 155, 87],
        "Höhe [mm]": [45, 95, 66, 182, 81, 59, 129, 25, 200, 68, 100, 55, 24, 120, 53],
        "Gewicht [kg]": [1, 1.667, 0.256, 0.396, 1.2, 0.536, 3.116, 0.113, 1.268, 0.753, 2.317, 2.083, 0.148, 0.3, 0.156],
        "Orientierungen": ['13', '13', '13', '13', '13', '13', '13', '13', '13', '13', '13', '13', '13', '13', '13']
        }

    material_master = pd.DataFrame(material_master)

    # order = ["Box_Gluehlampe", "Box_Wischblatt", "Keilriemen_gross", "Box_Bremsbacke", "Keilriemen_klein", "Tuete", "Keilriemen_gross","Box_Gluehlampe", "Box_Gluehlampe"]
    # print("Order: ", order)
    
    order = items_transfer.get_items()
    print("Order:\n", order)
    order_data = []
    
    # Filtere Materialstamm nach den Artikeln der Bestellung
    for item in order:
        filtered = material_master[material_master["Label ODTF"] == item]
        for index, row in filtered.iterrows():
            order_data.append((
                item, # wird nur hinzugefügt um später in Ausgabe zu schreiben
                row["Länge [mm]"], 
                row["Breite [mm]"], 
                row["Höhe [mm]"], 
                row["Gewicht [kg]"], 
                row["Orientierungen"]
            ))

    print("Order data:\n", order_data)
    order_data_transfer.set_order_data(order_data)

    # Starte Simulation mit verschiedenen Parametern
    simulation = Grid_Search() # Erzeuge eine Instanz der Gittersuche
    simulation.run_grid_search() # Führe Simulationen mit den in der Gittersuche definierten Parametern durch

    print("Simulation done")

    # Führe Packplan_Processing aus
    from Packplan_Processing import Packplan_Processing
    # Packplan_Processing()
