# Cython part of pack algorithm

def Check_Placement_Modified_Stacking ((int, int, int) container, float max_weight, float carried_weight, article_locations, (float, int, int, int, int, float, float) orientation, int stack_raw_width, int stack_length, unsigned int stack_box_length, int stack_box_height, int stack_box_width):

    center_of_gravity_width = orientation[2]/2 # Schwerpunktbreite 
    center_of_gravity_length = orientation[3]/2 # Schwerpunktlänge
    cdef int add_allowed = 0

    # Prüfe ob das Packstück auf den Stapel passt
    if ((center_of_gravity_width <= stack_raw_width) and # Schwerpunkt darf nicht die Stapelbreite überschreiten
        (center_of_gravity_length <= stack_length) and # Schwerpunkt darf nicht die Stapellänge überschreiten
        (orientation[4] + stack_box_height <= container[1]) and # Containerhöhe darf nicht überschritten werden
        (orientation[3] + stack_box_length <= container[2]) and # Containerlänge darf nicht überschritten werden
        (orientation[2] + stack_box_width <= container[0]) and # Containerbreite darf nicht überschritten werden
        (carried_weight + orientation[5] <= max_weight)): # Gewichtsobergrenzte darf nicht überschritten werden

        add_allowed = True

        # Prüfe, ob Überschneidungen zu anderen Packstücke existieren
        for article in article_locations:
            if ((((article.y <= stack_box_height) and (article.height + article.y > stack_box_height)) or ((article.y > stack_box_height) and (article.y < stack_box_height + orientation[4])))and # Überschneidungen in der Höhe
                (((article.x > stack_box_width) and (article.x < stack_box_width + orientation[2])) or ((article.x <= stack_box_width) and (article.x + article.width > stack_box_width))) and # Überschneidungen in der Breite
                (((article.z > stack_box_length) and (article.z < stack_box_length + orientation[3])) or ((article.z <= stack_box_length) and (article.z +article.length > stack_box_length)))): # Überschneidungen in der Länge
                    
                add_allowed = False
                break

    return add_allowed

def Check_Placement (article_to_add, articles_in_raw, (int, int, int) container, float max_weight, float carried_weight, (float, int, int, int, int, float, float) orientation, unsigned int raw_width, int raw_length, unsigned int box_length, unsigned int box_height, float factor_overfilling):

    # cdef int add_allowed = 0
    # add_allowed: cython.int = 0
    cdef int add_allowed = 0


    if ((orientation[2] <= raw_width) and # Breite der Orientierung muss kleiner gleich der Breite des Leerraums sein
        (orientation[3] <= raw_length * factor_overfilling) and # Länge der Orientierung darf Reihenlänge bis zu einer gewissen Toleranz T nicht überschreiten. T = factor_overfilling.
        (orientation[4] + box_height <= container[1]) and # Höhe der Orientierung zzgl. der Höhenposition muss kleiner gleich der Containerhöhe sein
        (orientation[3] + box_length <= container[2]) and # Länge der Orientierung zzgl. der Längenpoistion darf Containerlänge nicht überschreiten
        ((carried_weight + orientation[5]) <= max_weight)): # Sicherstellung der Gewichtseinhaltung

            # Stelle sicher, dass zu betrachtetes Packstück noch nicht gepackt wurde
            # Bedingung wird aufgrund der erzeugten Nebenreihen benötigt
            add_allowed = article_to_add not in articles_in_raw

    return add_allowed

# 2.2 Bedingung für Traglast einzelner Packstücke

def Calculate_Overlapping_Areas(selected_orientation, articles_from_extension):
    # Liefert für jedes Packstück der Stapelgrundfläche, auf das gestapelt wird, zu welchem Anteil es von dem zu packenden Packstück überdeckt wird
    
    # Definiere Datentypen
    cdef int width_article_to_pack
    cdef int length_article_to_pack
    cdef float width_coordinate_article_to_pack
    cdef float length_coordinate_article_to_pack
    
    cdef int package_ID_article_base
    cdef int width_article_base
    cdef int length_article_base
    cdef float width_coordinate_article_base
    cdef float length_coordinate_article_base

    cdef float overlapping_area
    cdef float overlap_in_weight
    cdef float overlap_in_length

    # Initialisiere Listen zum Speichern der Packstück ID aller Packstücke, welche durch das zu packende Packstück überdeckt werden + die Göße der übderdeckten Fläche
    overlapping_area_of_each_base_article  = []

    
    # Initialisiere Variablen für die Maße und Koordinaten des zu packenden Packstücks
    width_article_to_pack = selected_orientation[2] #0
    length_article_to_pack = selected_orientation[0] #2
    width_coordinate_article_to_pack = selected_orientation[3] #3
    length_coordinate_article_to_pack = selected_orientation[5]

    for article in articles_from_extension: # Durchlaufe alle Packstück der Stapelgrundfläche
        
        # Initialisiere Variablen für die Maße, Koordinaten und ID des zu Packstücks aus der Fläche, die als Basis für den Stapel dient
        package_ID_article_base = article[0] # Packstück ID
        width_article_base = article[1] # Packstück Breite
        length_article_base = article[2] # Packstück Länge
        width_coordinate_article_base = article[3] # Packstück Breitenkoordinate
        length_coordinate_article_base = article[4] # Packstück Längenkoordinate

        # Initialisiere Variable für die Überlappende Fläche
        overlapping_area = 0
        
    
        # Brechne die Überlappung in der Breite
        overlap_in_weight = min((width_coordinate_article_base + width_article_base), (width_coordinate_article_to_pack + width_article_to_pack)) - max(width_coordinate_article_base, width_coordinate_article_to_pack)
        # Berechne die Überlappung in der Länge
        overlap_in_length = min((length_coordinate_article_base + length_article_base), (length_coordinate_article_to_pack + length_article_to_pack)) - max(length_coordinate_article_base, length_coordinate_article_to_pack)
        
        if (overlap_in_weight >= 0) and (overlap_in_length >= 0):
            overlapping_area = overlap_in_weight * overlap_in_length # Berechnen die überlappende Fläche
        
        if (overlapping_area > 0): # Überprüfe, ob es eine überlappende Fläche gibt
            # Füge Informationen über das Packstück der Stapelgrundfläche der Liste hinzu
            overlapping_area_of_each_base_article.append((package_ID_article_base, overlapping_area))
   


    return overlapping_area_of_each_base_article