import random
import math
from mpl_toolkits.basemap import Basemap
import matplotlib.pyplot as plt
import numpy as np


class balle:
   def __init__(self, lon, lat, nom):
      self.lon = lon
      self.lat = lat
      self.nom = nom
   

   def distance(self, balle):
      distanceX = (balle.lon-self.lon)*40000*math.cos((self.lat+balle.lat)*math.pi/360)/360
      distanceY = (self.lat-balle.lat)*40000/360
      distance = math.sqrt( (distanceX*distanceX) + (distanceY*distanceY) )
      return distance

class GestionnaireCircuit:
   ballesDestinations = []
   
   def ajouterballe(self, balle):
      self.ballesDestinations.append(balle)
   
   def getballe(self, index):
      return self.ballesDestinations[index]
   
   def nombreballes(self):
      return len(self.ballesDestinations)

class Circuit:
   def __init__(self, gestionnaireCircuit, circuit=None):
      self.gestionnaireCircuit = gestionnaireCircuit
      self.circuit = []
      self.fitness = 0.0
      self.distance = 0
      if circuit is not None:
         self.circuit = circuit
      else:
         for i in range(0, self.gestionnaireCircuit.nombreballes()):
            self.circuit.append(None)

   def __len__(self):
      return len(self.circuit)
   
   def __getitem__(self, index):
     return self.circuit[index]

   def __setitem__(self, key, value):
     self.circuit[key] = value

   def genererIndividu(self):
     for indiceballe in range(0, self.gestionnaireCircuit.nombreballes()):
        self.setballe(indiceballe, self.gestionnaireCircuit.getballe(indiceballe))
     random.shuffle(self.circuit)

   def getballe(self, circuitPosition):
     return self.circuit[circuitPosition]

   def setballe(self, circuitPosition, balle):
     self.circuit[circuitPosition] = balle
     self.fitness = 0.0
     self.distance = 0

   def getFitness(self):
     if self.fitness == 0:
        self.fitness = 1/float(self.getDistance())
     return self.fitness

   def getDistance(self):
     if self.distance == 0:
        circuitDistance = 0
        for indiceballe in range(0, self.tailleCircuit()):
           balleOrigine = self.getballe(indiceballe)
           balleArrivee = None
           if indiceballe+1 < self.tailleCircuit():
              balleArrivee = self.getballe(indiceballe+1)
           else:
              balleArrivee = self.getballe(0)
           circuitDistance += balleOrigine.distance(balleArrivee)
        self.distance = circuitDistance
     return self.distance

   def tailleCircuit(self):
     return len(self.circuit)

   def contientballe(self, balle):
     return balle in self.circuit

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
            enfant.setballe(i, parent1.getballe(i))
         elif startPos > endPos:
            if not (i < startPos and i > endPos):
               enfant.setballe(i, parent1.getballe(i))
      
      for i in range(0, parent2.tailleCircuit()):
         if not enfant.contientballe(parent2.getballe(i)):
            for ii in range(0, enfant.tailleCircuit()):
               if enfant.getballe(ii) == None:
                  enfant.setballe(ii, parent2.getballe(i))
                  break
      
      return enfant
   
   def muter(self, circuit):
     for circuitPos1 in range(0, circuit.tailleCircuit()):
        if random.random() < self.tauxMutation:
           circuitPos2 = int(circuit.tailleCircuit() * random.random())
           
           balle1 = circuit.getballe(circuitPos1)
           balle2 = circuit.getballe(circuitPos2)
           
           circuit.setballe(circuitPos2, balle1)
           circuit.setballe(circuitPos1, balle2)

   def selectionTournoi(self, pop):
     tournoi = Population(self.gestionnaireCircuit, self.tailleTournoi, False)
     for i in range(0, self.tailleTournoi):
        randomId = int(random.random() * pop.taillePopulation())
        tournoi.sauvegarderCircuit(i, pop.getCircuit(randomId))
     fittest = tournoi.getFittest()
     return fittest

if __name__ == '__main__':
   
   gc = GestionnaireCircuit()

   #on cree nos balles
   for i in range(10):
       gc.ajouterballe(balle(random.uniform(-13.5, 13.5), random.uniform(-6.5, 6.5), 'Balle n°'+str(i)))

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
   for balle in meilleurePopulation.circuit:
      lons.append(balle.lon)
      lats.append(balle.lat)
      noms.append(balle.nom)

   lons.append(lons[0])
   lats.append(lats[0])
   noms.append(noms[0])

   img = plt.imread('../models/ground_texture.png')
   plt.figure()
   plt.imshow(img, extent=[-14, 14, -7, 7])

   x, y = lons, lats
   plt.plot(x,y,'ko', markersize=12)
   for nom,xpt,ypt in zip(noms,x,y):
       plt.text(xpt+0.5,ypt+0.5,nom)

   plt.plot(x, y, 'o-', markersize=10, linewidth=2, color='k', markerfacecolor='yellow') 
   plt.show()
