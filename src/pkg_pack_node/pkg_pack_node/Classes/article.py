from functools import reduce
from operator import add

class Article:
    def __init__(self, width, height, length, x, y, z, is_stack, package_ID, package_sequence_nr, package_weight):
        self.width = width # Packstückbreite
        self.height = height # Packstückhöhe
        self.length = length # Packstücklänge
        self.x = x # Breitenkoordinate
        self.y = y # Höhenkoordinate
        self.z = z # Längenkoordinate
        self.is_stack = is_stack # 0 wenn sich keine Packstücke auf dem Packstück befinden, sonst 1
        self.package_ID = package_ID # eindeuitge ID des Packstücks
        self.package_sequence_nr = package_sequence_nr # Entspritcht Reihenfolge, nachdem Packstücke in Container gepackt werden
        self.package_weight = package_weight # Gewicht des Packstücks
        self.supported_articles = [(0,0)] # Tupel mit (Packstück-ID, anteiliges Gewicht) von Packstücken, umittelbar auf diese Packstück-Instanz

    def set_is_stack_to_true(self):
        self.is_stack = True

    # Überprüfe beim beladen des Packstücks mit einer zusätzliche Last, ob die maximale Traglast des Packstücks durch diese überschritten wird
    def check_if_article_has_overload(self, weight_of_article_to_stack):
        
        # Definiere den Faktor, der vorgierbt das Wievielfache seines Eigengewicht ein Packstück tragen darf
        overload_factor = 1.0

        # Initialisiere die Variable, die bestimmt ob das Packstück überladen ist
        article_has_overload = False

        supported_weights = [supported_article[1] for supported_article in self.supported_articles] # Erzeuge eine Liste, welche die Last jedes einzelnen Packstücks enthät, das das Packstück bereits trägt
        supported_weight_sum = reduce(add, supported_weights) # Berechne die gesamte Last, die das Packstück bereits trägt

        if ((supported_weight_sum + weight_of_article_to_stack) > (self.package_weight * overload_factor)): # Prüfe ob die gesamte Last die das Pakcstück bereits trägt + die zusätzliche Last durch das neue Packstück die maximale Traglast übersteigt
            article_has_overload = True # Setzte die Variable, die bestimmt ob das Packstück überladen ist auf True
        
        return article_has_overload # Liefere die Information, ob das Packstück durch die zusätzliche Last überladen wird zurück

    # Fügt Packstück ID und die damit verbundene Last der Liste alle Packstücke, die durch diese Packstück getragen werden hinzu
    def add_supported_article(self, supported_article_ID, supported_article_weight):
        self.supported_articles.append((supported_article_ID, supported_article_weight))