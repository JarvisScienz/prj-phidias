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
class new_block(Belief): pass
class sense_distance(Belief): pass
class sense_color(Belief): pass

# beliefs sent by the robot
class target_got(Reactor): pass
class distance(Reactor): pass
class color(Reactor): pass
class pose(SingletonBelief): pass

class block(Belief): pass
class target(SingletonBelief): pass

class go(Procedure): pass
class generate(Procedure): pass
class sense(Procedure): pass
class scan(Procedure): pass
class _scan(Procedure): pass
class _scan_next(Procedure): pass
class goto_block(Procedure): pass

class link(Belief): pass
class coordinates_block(Belief): pass
class find_min_path(Procedure): pass
class path(Procedure): pass
class select_min(Procedure): pass
class show_min(Procedure): pass
class go_to_coordinates_block(Procedure): pass
class follow_min_path(Procedure): pass
class selected(SingletonBelief): pass
class runtime_path(SingletonBelief): pass

def_vars('X','Y','A','_A','D', 'W', 'Gap', 'C', 'N', 'Src', 'Dest', 'Next', 'Cost', 'P', 'Total', 'CurrentMin', 'CurrentMinCost', "index", 'length', 'MinPath')

# ---------------------------------------------------------------------
# Agent 'main'
# ---------------------------------------------------------------------
class main(Agent):
    def main(self):
        # commands
        go(X,Y,A) >> [ +go_to(X,Y,A)[{'to': 'robot@127.0.0.1:6566'}] ]
        generate() >> [ +new_block()[{'to': 'robot@127.0.0.1:6566'}] ]
        sense() >> [ +sense_distance()[{'to': 'robot@127.0.0.1:6566'}],
                     +sense_color()[{'to': 'robot@127.0.0.1:6566'}] ]

        # strategy
        generate(0) >> [show_line("Blocchi posizionati")]
        generate(N) / gt(N, 6) >> [ show_line("Il valore massimo di blocchi da generare è 6") ]
        generate(N) >> [ generate(), "N = N - 1", generate(N) ]

        scan() / geometry(W,Gap) >> [ "X = W/2 + W + Gap", 
                                      go(X, 0.02, -90), 
                                      +target(X, 0.02) ]

        +target_got()[{'from': _A}] / (target(X, Y) & runtime_path(MinPath)) >> \
          [
              show_line('Reached Position (', X,",",Y,")"),
              #sense()
              #show_line("Index: ", index),
              #"index = index + 1",
#              go_to_coordinates_block(Next),
              follow_min_path(MinPath)
          ]

        +distance(D)[{'from':_A}] / (target(X, Y) & lt(D,0.02)) >> [ show_line("Block found in position ", X),
                                                                      +block(X) ]

        +color(C)[{'from':_A}] / (target(X, Y) & block(X)) >> [ show_line("Color ", C, " sampled in position ", X),
                                                                -block(X),
                                                                +block(X, C),
                                                                _scan_next() ]
        +color(C)[{'from':_A}] >> [ show_line("color(C)"), _scan_next() ]
        +color()[{'from':_A}] >> [ show_line("color()"), _scan_next() ]

        _scan_next() / (target(X, Y) & gt(X,0.2)) >> \
          [
              show_line('end')
          ]
        _scan_next() / (target(X, Y) & geometry(W, Gap) & runtime_path(path)) >> \
          [
              #show_line("X value: ", X),
              #show_line("W value: ", W),
              #show_line("Gap value: ", Gap),
              "X = X + W + Gap", 
              +target(X, Y),
              go(X, Y, -90)
          ]

        goto_block(C) / block(X,C) >> [ go(X, 0.02, -90),
                                        +target(X, 0.02) ]

        find_min_path(Src, Dest) >> \
        [
           path([], 0, Src, Dest),
           show_min()
        ]

        #Step finale. Nodo src e nodo dest coincidono.
        path(P, Total, Dest, Dest) >> \
        [ 
              "P.append(Dest)", 
              show_line("Trovato nuovo minimo. Path: ", P, " - Costo: ", Total),
              +selected(P, Total)
        ]
        #Tramite ['all'] possiamo prendere in considerazioni tutte le possibilità di esecuzione, effettua la chiamata su tutti i link disponibili
        path(P, Total, Src,  Dest)['all'] / link(Src,Next,Cost) >> \
          [
              "P = P.copy()", #Usare copy perché sennò per tutta l'esecuzione, verrà utilizzato lo stesso array e il contenuto sarà errato. Minimum Cost Path ['A', 'B', 'C', 'A', 'D', 'B', 'C', 'A', 'E', 'D', 'B', 'C'], cost 5
              "P.append(Src)",
              "Total = Total + Cost",
              select_min(P, Total, Next, Dest)
          ]

        select_min(P, Total, Next, Dest) / (selected(CurrentMin, CurrentMinCost) & gt(Total, CurrentMinCost)) >> \
          [
              show_line("Scartato path: ", P, " ", Next, ", - Costo: ", Total)
          ]

        select_min(P, Total, Next, Dest) >> \
          [
              path(P, Total, Next, Dest)
          ]

        show_min() / selected(CurrentMin, CurrentMinCost)  >> \
          [
              show_line("Path costo minimo: ", CurrentMin, ". Costo: ", CurrentMinCost),
              +runtime_path(CurrentMin),
              "length = len(CurrentMin)",
              show_line("Lunghezza array: ", length),
              follow_min_path(CurrentMin)
          ]
      
        follow_min_path(CurrentMin) / (runtime_path(MinPath) & gt(length, 0)) >> \
        [
              "Next = CurrentMin[0]",
              "MinPath.pop(0)",
              +runtime_path(MinPath),
              "length = len(MinPath)",
show_line("Lunghezza array aggiornato: ", length),
              #"index = index + 1",
              go_to_coordinates_block(Next),
              #follow_min_path(CurrentMin, index)
        ]
  
        follow_min_path(MinPath) >> \
        [
              -runtime_path(MinPath)
        ]

        go_to_coordinates_block(Next) / coordinates_block(Next, X, Y) >> \
        [
              show_line("Vado al nodo ", Next, " con coordinate (",X,",",Y,")"),
              +target(X, Y),
              go(X, Y, -90)
        ]

ag = main()
ag.start()
ag.assert_belief(geometry(0.015, 0.01)) #W, Gap
#Grafo
#PHIDIAS.assert_belief(link('A','B',2))
#PHIDIAS.assert_belief(link('B','C',3))
PHIDIAS.assert_belief(link('A', 'B', 3))
PHIDIAS.assert_belief(link('A', 'C', 2))
PHIDIAS.assert_belief(link('A', 'D', 3))

PHIDIAS.assert_belief(link('B', 'C', 1))

PHIDIAS.assert_belief(link('C', 'E', 1))

PHIDIAS.assert_belief(link('D', 'C', 1))
PHIDIAS.assert_belief(link('D', 'E', 3))
PHIDIAS.assert_belief(coordinates_block('A', 0.04, 0.02))
PHIDIAS.assert_belief(coordinates_block('B', 0.14, -0.01))
PHIDIAS.assert_belief(coordinates_block('C', 0.08, 0.02))
PHIDIAS.assert_belief(coordinates_block('D', 0.12, 0.025))
PHIDIAS.assert_belief(coordinates_block('E', 0.1, 0.00))

PHIDIAS.run_net(globals(), 'http')
PHIDIAS.shell(globals())

