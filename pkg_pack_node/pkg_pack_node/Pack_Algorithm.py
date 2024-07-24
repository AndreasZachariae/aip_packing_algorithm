import pandas as pd
import yaml
from Classes.grid_search import Grid_Search
from Classes.order_data_transfer import order_data_transfer
from Classes.items_transfer import items_transfer

class Pack_Algorithm:
    
    # Lese Materialstamm aus YAML-Datei
    with open('aip_packing_algorithm/pkg_pack_node/pkg_pack_node/material_master.yaml', 'r') as file:
        material_master = yaml.load(file, Loader=yaml.FullLoader)

    # Konvertiere das Dictionary in ein DataFrame
    material_master = pd.DataFrame(material_master)

    # Für Simulation ohne ROS die folgende Zeile aktivieren
    order = ["Box_Gluehlampe", "Keilriemen_gross", "Tuete", "Box_Bremsbacke", "Box_Wischblatt", "Keilriemen_klein"]
    
    # Für Simulation mit ROS die folgende Zeile deaktivieren
    # order = items_transfer.get_items()
    
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

    # print("Order data:\n", order_data)
    order_data_transfer.set_order_data(order_data)

    # Starte Simulation mit verschiedenen Parametern
    simulation = Grid_Search() # Erzeuge eine Instanz der Gittersuche
    simulation.run_grid_search() # Führe Simulationen mit den in der Gittersuche definierten Parametern durch

    print("Simulation done")

    # Führe Packplan_Processing aus

# Für Simulation ohne ROS folgende Zeile aktivieren
if __name__ == '__main__':

    from Packplan_Processing import Packplan_Processing
    final_packplan, final_container = Packplan_Processing.generate_output()
