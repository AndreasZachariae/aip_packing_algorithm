import pandas as pd
from Classes.grid_search import Grid_Search
from Classes.order_data_transfer import order_data_transfer
from Classes.items_transfer import items_transfer

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

    # Für Simulation ohne ROS die folgende Zeile aktivieren
    order = ["Box_Gluehlampe", "Box_Wischblatt", "Keilriemen_gross", "Box_Bremsbacke", "Keilriemen_klein", "Tuete"]
    
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
