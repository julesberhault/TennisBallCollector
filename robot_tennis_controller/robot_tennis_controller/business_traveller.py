import random
import math
from mpl_toolkits.basemap import Basemap
import matplotlib.pyplot as plt
import numpy as np


class Ville:
   def __init__(self, lon, lat, nom):
      self.lon = lon
      self.lat = lat
      self.nom = nom
   

   def distance(self, ville):
      distanceX = (ville.lon-self.lon)*40000*math.cos((self.lat+ville.lat)*math.pi/360)/360
      distanceY = (self.lat-ville.lat)*40000/360
      distance = math.sqrt( (distanceX*distanceX) + (distanceY*distanceY) )
      return distance

class GestionnaireCircuit:
   villesDestinations = []
   
   def ajouterVille(self, ville):
      self.villesDestinations.append(ville)
   
   def getVille(self, index):
      return self.villesDestinations[index]
   
   def nombreVilles(self):
      return len(self.villesDestinations)

class Circuit:
   def __init__(self, gestionnaireCircuit, circuit=None):
      self.gestionnaireCircuit = gestionnaireCircuit
      self.circuit = []
      self.fitness = 0.0
      self.distance = 0
      if circuit is not None:
         self.circuit = circuit
      else:
         for i in range(0, self.gestionnaireCircuit.nombreVilles()):
            self.circuit.append(None)

   def __len__(self):
      return len(self.circuit)
   
   def __getitem__(self, index):
     return self.circuit[index]

   def __setitem__(self, key, value):
     self.circuit[key] = value

   def genererIndividu(self):
     for indiceVille in range(0, self.gestionnaireCircuit.nombreVilles()):
        self.setVille(indiceVille, self.gestionnaireCircuit.getVille(indiceVille))
     random.shuffle(self.circuit)

   def getVille(self, circuitPosition):
     return self.circuit[circuitPosition]

   def setVille(self, circuitPosition, ville):
     self.circuit[circuitPosition] = ville
     self.fitness = 0.0
     self.distance = 0

   def getFitness(self):
     if self.fitness == 0:
        self.fitness = 1/float(self.getDistance())
     return self.fitness

   def getDistance(self):
     if self.distance == 0:
        circuitDistance = 0
        for indiceVille in range(0, self.tailleCircuit()):
           villeOrigine = self.getVille(indiceVille)
           villeArrivee = None
           if indiceVille+1 < self.tailleCircuit():
              villeArrivee = self.getVille(indiceVille+1)
           else:
              villeArrivee = self.getVille(0)
           circuitDistance += villeOrigine.distance(villeArrivee)
        self.distance = circuitDistance
     return self.distance

   def tailleCircuit(self):
     return len(self.circuit)

   def contientVille(self, ville):
     return ville in self.circuit

class Population:
   def __init__(self, gestionnaireCircuit, taillePopulation, init):
      self.circuits = []
      for i in range(0, taillePopulation):
         self.circuits.append(None)
      
      if init:
         for i in range(0, taillePopulation):
            nouveauCircuit = Circuit(gestionnaireCircuit)
            nouveauCircuit.genererIndividu()
            self.sauvegarderCircuit(i, nouveauCircuit)
      
   def __setitem__(self, key, value):
      self.circuits[key] = value
   
   def __getitem__(self, index):
      return self.circuits[index]
   
   def sauvegarderCircuit(self, index, circuit):
      self.circuits[index] = circuit
   
   def getCircuit(self, index):
      return self.circuits[index]
   
   def getFittest(self):
      fittest = self.circuits[0]
      for i in range(0, self.taillePopulation()):
         if fittest.getFitness() <= self.getCircuit(i).getFitness():
            fittest = self.getCircuit(i)
      return fittest
   
   def taillePopulation(self):
      return len(self.circuits)

class GA:
   def __init__(self, gestionnaireCircuit):
      self.gestionnaireCircuit = gestionnaireCircuit
      self.tauxMutation = 0.015
      self.tailleTournoi = 5
      self.elitisme = True
   
   def evoluerPopulation(self, pop):
      nouvellePopulation = Population(self.gestionnaireCircuit, pop.taillePopulation(), False)
      elitismeOffset = 0
      if self.elitisme:
         nouvellePopulation.sauvegarderCircuit(0, pop.getFittest())
         elitismeOffset = 1
      
      for i in range(elitismeOffset, nouvellePopulation.taillePopulation()):
         parent1 = self.selectionTournoi(pop)
         parent2 = self.selectionTournoi(pop)
         enfant = self.crossover(parent1, parent2)
         nouvellePopulation.sauvegarderCircuit(i, enfant)
      
      for i in range(elitismeOffset, nouvellePopulation.taillePopulation()):
         self.muter(nouvellePopulation.getCircuit(i))
      
      return nouvellePopulation


   def crossover(self, parent1, parent2):
      enfant = Circuit(self.gestionnaireCircuit)
      
      startPos = int(random.random() * parent1.tailleCircuit())
      endPos = int(random.random() * parent1.tailleCircuit())
      
      for i in range(0, enfant.tailleCircuit()):
         if startPos < endPos and i > startPos and i < endPos:
            enfant.setVille(i, parent1.getVille(i))
         elif startPos > endPos:
            if not (i < startPos and i > endPos):
               enfant.setVille(i, parent1.getVille(i))
      
      for i in range(0, parent2.tailleCircuit()):
         if not enfant.contientVille(parent2.getVille(i)):
            for ii in range(0, enfant.tailleCircuit()):
               if enfant.getVille(ii) == None:
                  enfant.setVille(ii, parent2.getVille(i))
                  break
      
      return enfant
   
   def muter(self, circuit):
     for circuitPos1 in range(0, circuit.tailleCircuit()):
        if random.random() < self.tauxMutation:
           circuitPos2 = int(circuit.tailleCircuit() * random.random())
           
           ville1 = circuit.getVille(circuitPos1)
           ville2 = circuit.getVille(circuitPos2)
           
           circuit.setVille(circuitPos2, ville1)
           circuit.setVille(circuitPos1, ville2)

   def selectionTournoi(self, pop):
     tournoi = Population(self.gestionnaireCircuit, self.tailleTournoi, False)
     for i in range(0, self.tailleTournoi):
        randomId = int(random.random() * pop.taillePopulation())
        tournoi.sauvegarderCircuit(i, pop.getCircuit(randomId))
     fittest = tournoi.getFittest()
     return fittest

if __name__ == '__main__':
   
   gc = GestionnaireCircuit()   

   #on cree nos villes
   ville1 = Ville(3.002556, 45.846117, 'Clermont-Ferrand')
   gc.ajouterVille(ville1)
   ville2 = Ville(-0.644905, 44.896839, 'Bordeaux')
   gc.ajouterVille(ville2)
   ville3 = Ville(-1.380989, 43.470961, 'Bayonne')
   gc.ajouterVille(ville3)
   ville4 = Ville(1.376579, 43.662010, 'Toulouse')
   gc.ajouterVille(ville4)
   ville5 = Ville(5.337151, 43.327276, 'Marseille')
   gc.ajouterVille(ville5)
   ville6 = Ville(7.265252, 43.745404, 'Nice')
   gc.ajouterVille(ville6)
   ville7 = Ville(-1.650154, 47.385427, 'Nantes')
   gc.ajouterVille(ville7)
   ville8 = Ville(-1.430427, 48.197310, 'Rennes')
   gc.ajouterVille(ville8)
   ville9 = Ville(2.414787, 48.953260, 'Paris')
   gc.ajouterVille(ville9)
   ville10 = Ville(3.090447, 50.612962, 'Lille')
   gc.ajouterVille(ville10)
   ville11 = Ville(5.013054, 47.370547, 'Dijon')
   gc.ajouterVille(ville11)
   ville12 = Ville(4.793327, 44.990153, 'Valence')
   gc.ajouterVille(ville12)
   ville13 = Ville(2.447746, 44.966838, 'Aurillac')
   gc.ajouterVille(ville13)
   ville14 = Ville(1.750115, 47.980822, 'Orleans')
   gc.ajouterVille(ville14)
   ville15 = Ville(4.134148, 49.323421, 'Reims')
   gc.ajouterVille(ville15)
   ville16 = Ville(7.506950, 48.580332, 'Strasbourg')
   gc.ajouterVille(ville16)
   ville17 = Ville(1.233757, 45.865246, 'Limoges')
   gc.ajouterVille(ville17)
   ville18 = Ville(4.047255,48.370925, 'Troyes')
   gc.ajouterVille(ville18)
   ville19 = Ville(0.103163,49.532415, 'Le Havre')
   gc.ajouterVille(ville19)
   ville20 = Ville(-1.495348, 49.667704, 'Cherbourg')
   gc.ajouterVille(ville20)
   ville21 = Ville(-4.494615, 48.447500, 'Brest')
   gc.ajouterVille(ville21)
   ville22 = Ville(-0.457140, 46.373545, 'Niort')
   gc.ajouterVille(ville22)


   #on initialise la population avec 50 circuits
   pop = Population(gc, 50, True)
   print("Distance initiale : " + str(pop.getFittest().getDistance()))
   
   # On fait evoluer notre population sur 100 generations
   ga = GA(gc)
   pop = ga.evoluerPopulation(pop)
   for i in range(0, 100):
      pop = ga.evoluerPopulation(pop)
   
   print("Distance finale : " + str(pop.getFittest().getDistance()))
   meilleurePopulation = pop.getFittest()

   #on genere une carte représentant notre solution
   lons = []
   lats = []
   noms = []
   for ville in meilleurePopulation.circuit:
      lons.append(ville.lon)
      lats.append(ville.lat)
      noms.append(ville.nom)

   lons.append(lons[0])
   lats.append(lats[0])
   noms.append(noms[0])


   map = Basemap(llcrnrlon=-5.5,llcrnrlat=42.3,urcrnrlon=9.3,urcrnrlat=51.,
             resolution='i', projection='tmerc', lat_0 = 45.5, lon_0 = -3.25)

   map.drawmapboundary(fill_color='aqua')
   map.fillcontinents(color='coral',lake_color='aqua')
   map.drawcoastlines()
   map.drawcountries()
   x,y = map(lons,lats)
   map.plot(x,y,'bo', markersize=12)
   for nom,xpt,ypt in zip(noms,x,y):
       plt.text(xpt+5000,ypt+25000,nom)

   map.plot(x, y, 'D-', markersize=10, linewidth=2, color='k', markerfacecolor='b') 
   plt.show()
