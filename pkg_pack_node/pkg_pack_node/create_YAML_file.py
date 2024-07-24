import pandas as pd
import yaml

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

# Konvertiere das DataFrame in ein Dictionary
material_master_df = pd.DataFrame(material_master)
material_master_dict = material_master_df.to_dict(orient='list')

# Speichere das Dictionary als YAML-Datei
with open('material_master.yaml', 'w') as file:
    yaml.dump(material_master_dict, file, default_flow_style=False, allow_unicode=True)

print("YAML-Datei erfolgreich erstellt.")
