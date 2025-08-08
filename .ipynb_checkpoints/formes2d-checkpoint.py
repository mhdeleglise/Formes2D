from abc import ABC, abstractmethod
import math

# Classe abstraite : Forme2D
class Forme2D(ABC):
    @abstractmethod
    def aire(self):
        """Calcule l'aire de la forme"""
        pass

    #@abstractmethod
    #def perimetre(self):
    #    """Calcule le périmètre de la forme"""
    #    pass

    def afficher(self):
        """Affiche une description générique"""
        print(f"{self.__class__.__name__} : aire = {self.aire()}, périmètre = {self.perimetre()}")

    @abstractmethod
    def draw(self, ax):
        """Dessine la forme sur un objet matplotlib Axes"""
        pass

    @abstractmethod
    def translate(self, dx, dy):
        pass

    @abstractmethod
    def rotation(self, angle_degres, centre=None):
        """Angle en degrés, rotation autour du point 'centre' (objet Point)."""
        pass
