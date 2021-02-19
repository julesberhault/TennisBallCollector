import random
import math
from mpl_toolkits.basemap import Basemap
import matplotlib.pyplot as plt
import numpy as np


class sommet:
   def __init__(self, lon, lat, nom):
      self.lon = lon
      self.lat = lat
      self.nom = nom
   

   def distance(self, sommet):
      distanceX = (sommet.lon-self.lon)*40000*math.cos((self.lat+sommet.lat)*math.pi/360)/360
      distanceY = (self.lat-sommet.lat)*40000/360
      distance = math.sqrt( (distanceX*distanceX) + (distanceY*distanceY) )
      return distance

class GestionnaireCircuit:
   sommetsDestinations = []
   
   def ajoutersommet(self, sommet):
      self.sommetsDestinations.append(sommet)
   
   def getsommet(self, index):
      return self.sommetsDestinations[index]
   
   def nombresommets(self):
      return len(self.sommetsDestinations)

class Circuit:
   def __init__(self, gestionnaireCircuit, circuit=None):
      self.gestionnaireCircuit = gestionnaireCircuit
      self.circuit = []
      self.fitness = 0.0
      self.distance = 0
      if circuit is not None:
         self.circuit = circuit
      else:
         for i in range(0, self.gestionnaireCircuit.nombresommets()):
            self.circuit.append(None)

   def __len__(self):
      return len(self.circuit)
   
   def __getitem__(self, index):
     return self.circuit[index]

   def __setitem__(self, key, value):
     self.circuit[key] = value

   def genererIndividu(self):
     for indicesommet in range(0, self.gestionnaireCircuit.nombresommets()):
        self.setsommet(indicesommet, self.gestionnaireCircuit.getsommet(indicesommet))
     random.shuffle(self.circuit)

   def getsommet(self, circuitPosition):
     return self.circuit[circuitPosition]

   def setsommet(self, circuitPosition, sommet):
     self.circuit[circuitPosition] = sommet
     self.fitness = 0.0
     self.distance = 0

   def getFitness(self):
     if self.fitness == 0:
        self.fitness = 1/float(self.getDistance())
     return self.fitness

   def getDistance(self):
     if self.distance == 0:
        circuitDistance = 0
        for indicesommet in range(0, self.tailleCircuit()):
           sommetOrigine = self.getsommet(indicesommet)
           sommetArrivee = None
           if indicesommet+1 < self.tailleCircuit():
              sommetArrivee = self.getsommet(indicesommet+1)
           else:
              sommetArrivee = self.getsommet(0)
           circuitDistance += sommetOrigine.distance(sommetArrivee)
        self.distance = circuitDistance
     return self.distance

   def tailleCircuit(self):
     return len(self.circuit)

   def contientsommet(self, sommet):
     return sommet in self.circuit

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
            enfant.setsommet(i, parent1.getsommet(i))
         elif startPos > endPos:
            if not (i < startPos and i > endPos):
               enfant.setsommet(i, parent1.getsommet(i))
      
      for i in range(0, parent2.tailleCircuit()):
         if not enfant.contientsommet(parent2.getsommet(i)):
            for ii in range(0, enfant.tailleCircuit()):
               if enfant.getsommet(ii) == None:
                  enfant.setsommet(ii, parent2.getsommet(i))
                  break
      
      return enfant
   
   def muter(self, circuit):
     for circuitPos1 in range(0, circuit.tailleCircuit()):
        if random.random() < self.tauxMutation:
           circuitPos2 = int(circuit.tailleCircuit() * random.random())
           
           sommet1 = circuit.getsommet(circuitPos1)
           sommet2 = circuit.getsommet(circuitPos2)
           
           circuit.setsommet(circuitPos2, sommet1)
           circuit.setsommet(circuitPos1, sommet2)

   def selectionTournoi(self, pop):
     tournoi = Population(self.gestionnaireCircuit, self.tailleTournoi, False)
     for i in range(0, self.tailleTournoi):
        randomId = int(random.random() * pop.taillePopulation())
        tournoi.sauvegarderCircuit(i, pop.getCircuit(randomId))
     fittest = tournoi.getFittest()
     return fittest

if __name__ == '__main__':
   
   gc = GestionnaireCircuit()

   #on cree nos sommets
   gc.ajoutersommet(sommet(random.uniform(2, 13)*random.choice((-1, 1)), random.uniform(-6, 6), 'Robot'))
   for i in range(6):
       gc.ajoutersommet(sommet(random.uniform(2, 13)*random.choice((-1, 1)), random.uniform(-6, 6), 'Balle n°'+str(i+1)))
   gc.ajoutersommet(sommet(0, 6.5, 'Crossing Point n°1'))
   gc.ajoutersommet(sommet(0, -6.5, 'Crossing Point n°2'))
   gc.ajoutersommet(sommet(-13, 6.5, 'Area n°1'))
   gc.ajoutersommet(sommet(13, -6.5, 'Area n°2'))

   #on initialise la population avec 50 circuits
   pop = Population(gc, 50, True)
   print("Distance initiale : " + str(pop.getFittest().getDistance()/150))
   
   # On fait evoluer notre population sur 100 generations
   ga = GA(gc)
   pop = ga.evoluerPopulation(pop)
   for i in range(0, 100):
      pop = ga.evoluerPopulation(pop)
   
   print("Distance finale : " + str(pop.getFittest().getDistance()/150))
   meilleurePopulation = pop.getFittest()

   #on genere une carte représentant notre solution
   lons = []
   lats = []
   noms = []
   for sommet in meilleurePopulation.circuit:
      lons.append(sommet.lon)
      lats.append(sommet.lat)
      noms.append(sommet.nom)

   lons.append(lons[0])
   lats.append(lats[0])
   noms.append(noms[0])

   img = plt.imread('../models/ground_texture.png')
   plt.figure()
   plt.imshow(img, extent=[-15, 15, -8, 8])

   x, y = lons, lats
   plt.plot(x,y,'ko', markersize=12)
   plt.plot(x, y, 'o-', markersize=10, linewidth=2, color='k', markerfacecolor='yellow')
   for nom,xpt,ypt in zip(noms,x,y):
       plt.text(xpt+0.5,ypt+0.5,nom)
       if nom == 'Robot':
          plt.plot(xpt, ypt, 'D', markersize=12, linewidth=2, color='k', markerfacecolor='r')
       if nom[:1] == 'A':
          plt.plot(xpt, ypt, 'o', markersize=12, linewidth=2, color='k', markerfacecolor='g')
       if nom[:1] == 'C':
          plt.plot(xpt, ypt, 'o', markersize=12, linewidth=2, color='k', markerfacecolor='b')
           
   plt.show()
