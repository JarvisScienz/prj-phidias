#
#
#

import sys

sys.path.insert(0, "/home/corrado/software/packages/agentspeak/phidias/lib")

from phidias.Types import *
from phidias.Main import *
from phidias.Lib import *
from phidias.Agent import *

class geometry(SingletonBelief): pass

# beliefs interpreted by the robot
class go_to(Belief): pass
class go_to_next_block(Belief): pass
class new_block(Belief): pass
class sense_distance(Belief): pass
class sense_color(Belief): pass
class plot(Belief): pass
class reset_vectors(Belief): pass
class capture_block(Belief): pass
class release_block(Belief): pass

# beliefs sent by the robot
class target_got(Reactor): pass
class distance(Reactor): pass
class color(Reactor): pass
class remove(Reactor): pass
class pose(SingletonBelief): pass
class targetX(Belief): pass

class block(Belief): pass
class target(SingletonBelief): pass

class generate(Procedure): pass
class pick(Procedure): pass
class empty(Procedure): pass

class go(Procedure): pass
class sense(Procedure): pass
class scan(Procedure): pass
class _scan(Procedure): pass
class _scan_next(Procedure): pass
class reset(Procedure): pass
class capture(Procedure): pass
class release(Procedure): pass
class goto_container(Procedure): pass
class next_move(Procedure): pass

class link(Belief): pass
class coordinates_node(Belief): pass
class present_in_path(Belief): pass
class block_picked(SingletonBelief): pass
class set_path(Procedure): pass
class clear(Procedure): pass
class find_min_path(Procedure): pass
class path(Procedure): pass
class select_min(Procedure): pass
class show_min(Procedure): pass
class go_to_coordinates_block(Procedure): pass
class follow_min_path(Procedure): pass
class selected(SingletonBelief): pass
class runtime_path(SingletonBelief): pass
class index(SingletonBelief): pass
class container(Belief): pass
class first_index(Belief): pass
class last_index(Belief): pass
class should_sense(SingletonBelief): pass
class current_node(SingletonBelief): pass

class GetCoordinates(Action):
    def execute(self, X, Y, coordinates):
        value = "(" + str(X.value) + "," + str(Y.value) + ")"
        #print (value)
        coordinates(value)

class CheckFirstValue(ActiveBelief):
    def evaluate(self, path, node):
        #print("CheckFirstValue. Node: ", node.value, " - path: ", path.value[0])
        return path.value[0] == node.value
        #return True

def_vars('X','Y','target_y','A','_A','D', 'W', 'Gap', 'C', 'N', 'Src', 'Dest', 'Next', 'Cost', 'P', 'Total', 'CurrentMin', 'CurrentMinCost', "index", 'length', 'MinPath', 'Node', 'i', 'AuxLen', 'WriteNode', 'coordinates', 'content')

# ---------------------------------------------------------------------
# Agent 'main'
# ---------------------------------------------------------------------
class main(Agent):
    def main(self):
        # commands
        go(X,Y,A) >> [ +go_to(X,Y,A)[{'to': 'robot@127.0.0.1:6566'}],
                       +target(X, Y)]
        generate() >> [ +new_block()[{'to': 'robot@127.0.0.1:6566'}] ]
        sense() / should_sense() >> [ +sense_distance()[{'to': 'robot@127.0.0.1:6566'}],
                     +sense_color()[{'to': 'robot@127.0.0.1:6566'}] ]
        reset() >> [ +reset_vectors()[{'to': 'robot@127.0.0.1:6566'}]]
        capture() >> [+capture_block()[{'to': 'robot@127.0.0.1:6566'}]]
        release() >> [+release_block()[{'to': 'robot@127.0.0.1:6566'}]]

        #generate(N)
        generate(0) >> [show_line("Blocchi posizionati")]
        generate(N) / gt(N, 6) >> [ show_line("Il valore massimo di blocchi da generare è 6") ]
        generate(N) >> [ generate(), "N = N - 1", generate(N) ]

        #pick()
        pick(i) / (index(i) & eq(i, 16)) >> \
          [
              +plot() [{'to': 'robot@127.0.0.1:6566'}],
              show_line('end')
          ]

        pick(i) / (index(i) & eq(i, 6) & current_node(Src)) >> [
            +index(12),
            find_min_path(Src, 11)
        ]

        pick(i) / (index(i) & current_node(Src)) >> [
            "Dest = i + 1",  
            +index(Dest),
            #"print('from: ' + str(Src) + ' to ' + str(i))",
            find_min_path(Src, i)
        ]

        pick() >> [
            reset(),
            +index(1),
            +should_sense(),
            go_to_coordinates_block(6)
        ]
        
        next_move() / block_picked(X, Y, C) >> [
              #show_line("goto_container"),
              goto_container(C)  
          ]
        
        next_move() / (index(i) & should_sense())>> [
            #show_line("pick ", i),
            pick(i)
          ]
        
        +target_got()[{'from': _A}] / (target(X, Y) & runtime_path(MinPath) & index(N)) >> \
          [
              #show_line('Reached Position (', X,",",Y,")"),
              #show_line(MinPath),
              #sense(),
              #show_line("Index: ", index),
              #"index = index + 1",
#              go_to_coordinates_block(Next),
              follow_min_path(MinPath)
          ]

        +target_got()[{'from': _A}] / (target(X, Y) & index(N)) >> \
          [
              #show_line('2.Reached Position (', X,",",Y,")"),
              sense(),
              #next_move()
          ]

        +distance(D, i)[{'from':_A}] / (target(X, Y) & lt(D,0.02)) >> [ #show_line("Block found in position ", X),
                                                                      +block(i) ]

        +color(C)[{'from':_A}] / (target(X, Y) & container(C, Total, content) & eq (Total, 2) & block(i)) >> [ 
            show_line("Color ", C, " sampled in position ", X),
            -block(i),
            +block(i, C),
            show_line('Contenitore ', C, ' saturo. Non è possibile aggiungere il blocco (', X, ',', Y, ')'),
            next_move()
        ]

        +color(C)[{'from':_A}] / (target(X, Y) & block(i)) >> [ 
            #show_line("Color ", C, " sampled in position ", X),
            -block(i),
            +block(i, C),
            +block_picked(X, Y, C),
            capture(),
            show_line(C, " block captured"),
            -should_sense(),
            next_move()
        ]
        
        +color()[{'from':_A}] >> [next_move()]
        
        +remove(i, C)[{'from':_A}] / block(i, C) >> [-block(i, C)]
        
        +remove(i, C)[{'from':_A}] >> [show_line("The block (",i , " ", C, ") was not found in the knowledge base")]

        goto_container(C) / (current_node(Src) & block_picked(X,Y,C) & eq(C, 'red') & container(C, Total, content)) >> \
             [ show_line("Sposto l'elemento nel container ROSSO"), 
               -block_picked(X,Y,C),
               #delete_block(X),
               #"Src = Src - 1",
               +first_index(Src),
               +last_index(8),
               find_min_path(Src, 8),
               #release(),
               #go(0.08, -0.016, -90),
               #+target(0.08, 0.02),
               -container(C, Total, content), 
               "coordinates = ''", 
               GetCoordinates(X,Y,coordinates), 
               "content.append(coordinates)", 
               "Total = Total + 1", 
               +container(C, Total, content),
             ]

        goto_container(C) / (current_node(Src) & block_picked(X,Y,C) & eq(C, 'green') & container(C, Total, content)) >> \
             [ show_line("Sposto l'elemento nel container VERDE"), 
               -block_picked(X,Y,C),
               #delete_block(X),
               #"Src = Src - 1",
               +first_index(Src),
               +last_index(9),
               find_min_path(Src, 9),
               #release(),
               #go(0.095, -0.016, -90), 
               #+target(0.095, 0.02),
               -container(C, Total, content), 
               "coordinates = ''", 
               GetCoordinates(X,Y,coordinates), 
               "content.append(coordinates)", 
               "Total = Total + 1", 
               +container(C, Total, content),
             ]

        goto_container(C) / (current_node(Src) & block_picked(X,Y,C) & eq(C, 'blue') & container(C, Total, content)) >> \
             [ show_line("Sposto l'elemento nel container BLUE"), 
               -block_picked(X,Y,C),
               #delete_block(X),
               #"Src = Src - 1",
               +first_index(Src),
               +last_index(10),
               find_min_path(Src, 10),
               #release(),
               #go(0.12, -0.016, -90), 
               #+target(0.12, 0.02),
               -container(C, Total, content), 
               "coordinates = ''", 
               GetCoordinates(X,Y,coordinates), 
               "content.append(coordinates)", 
               "Total = Total + 1", 
               +container(C, Total, content),
             ]

        find_min_path(Src, Dest) / selected(CurrentMin, CurrentMinCost)>> \
        [
           #show_line("Cerco il path tra: ", Src, " e ", Dest),
           -selected(CurrentMin, CurrentMinCost),
           path([], 0, Src, Dest),
           show_min()
        ]

        find_min_path(Src, Dest) >> \
        [
           #show_line("Cerco il path tra: ", Src, " e ", Dest),
           path([], 0, Src, Dest),
           show_min()
        ]

        #Step finale. Nodo src e nodo dest coincidono.
        path(P, Total, Dest, Dest) >> \
        [ 
              "P.append(Dest)", 
              #show_line("Trovato nuovo minimo. Path: ", P, " - Costo: ", Total),
              +selected(P, Total)
        ]
        #Tramite ['all'] possiamo prendere in considerazioni tutte le possibilità di esecuzione, effettua la chiamata su tutti i link disponibili
        path(P, Total, Src,  Dest)['all'] / link(Src,Next,Cost) >> \
          [
              clear(),
              "P = P.copy()", #Usare copy perché sennò per tutta l'esecuzione, verrà utilizzato lo stesso array e il contenuto sarà errato. Minimum Cost Path ['A', 'B', 'C', 'A', 'D', 'B', 'C', 'A', 'E', 'D', 'B', 'C'], cost 5
              "P.append(Src)",
              "AuxLen = len(P)",
              set_path(P, AuxLen),
              "Total = Total + Cost",
              select_min(P, Total, Next, Dest)
          ]

        select_min(P, Total, Next, Dest) / (selected(CurrentMin, CurrentMinCost) & gt(Total, CurrentMinCost)) >> \
          [
              #show_line("Scartato path: ", P, " ", Next, ", - Costo: ", Total)
          ]
          
        select_min(P, Total, Next, Dest) / present_in_path(Next) >> \
          [
              #show_line("Individuato e scartato ciclo su nodo ", Next, ". Path: ", P)
          ]
          
        select_min(P, Total, Next, Dest) >> \
          [
              path(P, Total, Next, Dest)
          ]

        show_min() / selected(CurrentMin, CurrentMinCost)  >> \
          [
              #show_line("Path costo minimo: ", CurrentMin, ". Costo: ", CurrentMinCost),
              +runtime_path(CurrentMin),
              "length = len(CurrentMin)",
              show_line("Lunghezza array: ", length),
              follow_min_path(CurrentMin)
          ]
          
        set_path(P, AuxLen) >> [set_path(P, AuxLen, 0)]
        set_path(P, AuxLen, i) / geq(i, AuxLen) >> []#show_line("Path set")
        set_path(P, AuxLen, i) >> \
          [
              "WriteNode = P[i]",
              "i = i + 1",
              +present_in_path(WriteNode),
              set_path(P, AuxLen, i)
          ]
      
        clear()['all'] /present_in_path(Node) >> \
          [
              -present_in_path(Node)        
          ]

        follow_min_path(CurrentMin) / (runtime_path(MinPath) & gt(length, 0) & coordinates_node(Node, X, Y) & target(X,Y) & CheckFirstValue(MinPath, Node)) >> \
          [
              #show_line("EXP: ", CurrentMin, " - ", Node),
              #"Next = CurrentMin[0] if CurrentMin[0] != Node else CurrentMin[1]",
              #show_line("Bingo", Next),
              #"MinPath.pop(0)",
              "MinPath.pop(0)",
              +runtime_path(MinPath),
              "length = len(MinPath)",
              #show_line("Lunghezza array aggiornato: ", length),
              #go_to_coordinates_node(Next)
              #show_line("moving to : ", CurrentMin),
              follow_min_path(CurrentMin)
          ]
        
        follow_min_path(CurrentMin) / (runtime_path(MinPath) & gt(length, 0)) >> \
          [
              "Next = CurrentMin[0]",
              "MinPath.pop(0)",
              +runtime_path(MinPath),
              "length = len(MinPath)",
              show_line("Lunghezza array aggiornato: ", length),
              go_to_coordinates_block(Next),
              #follow_min_path(CurrentMin, index)
          ]
  
        follow_min_path(MinPath) / (selected(CurrentMin, CurrentMinCost) & first_index(N) & last_index(Node)) >> \
          [
              -selected(CurrentMin, CurrentMinCost),
              -runtime_path(MinPath),
              clear(),
              -last_index(Node),
              -first_index(N),
              release(),
              +should_sense(),
              find_min_path(Node,N)
          ]

        follow_min_path(MinPath) / (selected(CurrentMin, CurrentMinCost) & index(N)) >> \
          [
              #show_line("Follow min path 2"),
              -runtime_path(MinPath),
              -selected(CurrentMin, CurrentMinCost),
              clear(),
              sense()
          ]

        go_to_coordinates_block(Next) / coordinates_node(Next, X, Y) >> \
        [
              #show_line("Vado al nodo ", Next, " con coordinate (",X,",",Y,")"),
              go(X, Y, -90),
              +target(X, Y),
              +current_node(Next)
        ]

        #empty(color)
        empty(C) / (container(C, Total, content) & eq(Total, 0)) >> [show_line("Il contenitore ", C, " è già vuoto")]
        empty(C) / container(C, Total, content) >> [-container(C, Total, content), +container(C, 0, []), show_line("Contenitore ", C, " svuotato")]

ag = main()
ag.start()
ag.assert_belief(geometry(0.015, 0.01)) #W, Gap

#Grafo
PHIDIAS.assert_belief(link(1, 2, 0.01))
PHIDIAS.assert_belief(link(2, 3, 0.01))
PHIDIAS.assert_belief(link(3, 4, 0.01))
PHIDIAS.assert_belief(link(4, 5, 0.01))
PHIDIAS.assert_belief(link(1, 6, 0.8))
PHIDIAS.assert_belief(link(2, 6, 0.8))
PHIDIAS.assert_belief(link(3, 6, 0.8))
PHIDIAS.assert_belief(link(4, 6, 0.8))
PHIDIAS.assert_belief(link(5, 6, 0.8))
PHIDIAS.assert_belief(link(6, 7, 0.65))
PHIDIAS.assert_belief(link(7, 8, 0.8))
PHIDIAS.assert_belief(link(7, 9, 0.8))
PHIDIAS.assert_belief(link(7, 10, 0.8))
PHIDIAS.assert_belief(link(7, 16, 0.65))
PHIDIAS.assert_belief(link(16, 11, 0.8))
PHIDIAS.assert_belief(link(16, 12, 0.8))
PHIDIAS.assert_belief(link(16, 13, 0.8))
PHIDIAS.assert_belief(link(16, 14, 0.8))
PHIDIAS.assert_belief(link(16, 15, 0.8))
PHIDIAS.assert_belief(link(11, 12, 0.01))
PHIDIAS.assert_belief(link(12, 13, 0.01))
PHIDIAS.assert_belief(link(13, 14, 0.01))
PHIDIAS.assert_belief(link(14, 15, 0.01))

PHIDIAS.assert_belief(link(2, 1, 0.01))
PHIDIAS.assert_belief(link(3, 2, 0.01))
PHIDIAS.assert_belief(link(4, 3, 0.01))
PHIDIAS.assert_belief(link(5, 4, 0.01))
PHIDIAS.assert_belief(link(6, 1, 0.8))
PHIDIAS.assert_belief(link(6, 2, 0.8))
PHIDIAS.assert_belief(link(6, 3, 0.8))
PHIDIAS.assert_belief(link(6, 4, 0.8))
PHIDIAS.assert_belief(link(6, 5, 0.8))
PHIDIAS.assert_belief(link(7, 6, 0.65))
PHIDIAS.assert_belief(link(8, 7, 0.8))
PHIDIAS.assert_belief(link(9, 7, 0.8))
PHIDIAS.assert_belief(link(10, 7, 0.8))
PHIDIAS.assert_belief(link(16, 7, 0.65))
PHIDIAS.assert_belief(link(11, 16, 0.8))
PHIDIAS.assert_belief(link(12, 16, 0.8))
PHIDIAS.assert_belief(link(13, 16, 0.8))
PHIDIAS.assert_belief(link(14, 16, 0.8))
PHIDIAS.assert_belief(link(15, 16, 0.8))
PHIDIAS.assert_belief(link(12, 11, 0.01))
PHIDIAS.assert_belief(link(13, 12, 0.01))
PHIDIAS.assert_belief(link(14, 13, 0.01))
PHIDIAS.assert_belief(link(15, 14, 0.01))


ag.assert_belief(coordinates_node(1, 0.01, -0.016))
ag.assert_belief(coordinates_node(2, 0.02, -0.016))
ag.assert_belief(coordinates_node(3, 0.03, -0.016))
PHIDIAS.assert_belief(coordinates_node(4, 0.04, -0.016))
PHIDIAS.assert_belief(coordinates_node(5, 0.05, -0.016))
PHIDIAS.assert_belief(coordinates_node(6, 0.03, 0.07))
PHIDIAS.assert_belief(coordinates_node(7, 0.095, 0.07))
PHIDIAS.assert_belief(coordinates_node(8, 0.075, -0.016))
PHIDIAS.assert_belief(coordinates_node(9, 0.095, -0.016))
PHIDIAS.assert_belief(coordinates_node(10, 0.115, -0.016))
PHIDIAS.assert_belief(coordinates_node(11, 0.14, -0.016))
PHIDIAS.assert_belief(coordinates_node(12, 0.15, -0.016))
PHIDIAS.assert_belief(coordinates_node(13, 0.16, -0.016))
PHIDIAS.assert_belief(coordinates_node(14, 0.17, -0.016))
PHIDIAS.assert_belief(coordinates_node(15, 0.18, -0.016))
PHIDIAS.assert_belief(coordinates_node(16, 0.16, 0.07))

PHIDIAS.assert_belief(container("red", 0, []))
PHIDIAS.assert_belief(container("green", 0, []))
PHIDIAS.assert_belief(container("blue", 0, []))

PHIDIAS.run_net(globals(), 'http')
PHIDIAS.shell(globals())
