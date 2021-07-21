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

class block(Belief): pass
class target(SingletonBelief): pass

class go(Procedure): pass
class generate(Procedure): pass
class sense(Procedure): pass
class scan(Procedure): pass
class _scan(Procedure): pass
class _scan_next(Procedure): pass
class goto_block(Procedure): pass
class reset(Procedure): pass
class capture(Procedure): pass
class release(Procedure): pass

class link(Belief): pass
class coordinates_block(Belief): pass
class present_in_path(Belief): pass
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

def_vars('X','Y','A','_A','D', 'W', 'Gap', 'C', 'N', 'Src', 'Dest', 'Next', 'Cost', 'P', 'Total', 'CurrentMin', 'CurrentMinCost', "index", 'length', 'MinPath', 'Node', 'i', 'AuxLen', 'WriteNode')

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
        reset() >> [ +reset_vectors()[{'to': 'robot@127.0.0.1:6566'}]]
        capture() >> [+capture_block()[{'to': 'robot@127.0.0.1:6566'}]]
        release() >> [+release_block()[{'to': 'robot@127.0.0.1:6566'}]]

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
        
        +remove(X, C)[{'from':_A}] / block(X, C) >> [-block(X, C)]
        
        +remove(X, C)[{'from':_A}] >> [show_line("The block (", X, " ", C, ") was not found in the knowledge base")]

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
              show_line("Scartato path: ", P, " ", Next, ", - Costo: ", Total)
          ]
          
        select_min(P, Total, Next, Dest) / present_in_path(Next) >> \
          [
              show_line("Individuato e scartato ciclo su nodo ", Next, ". Path: ", P)
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
          
        set_path(P, AuxLen) >> [set_path(P, AuxLen, 0)]
        set_path(P, AuxLen, i) / geq(i, AuxLen) >> [show_line("Path set")]
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
  
        follow_min_path(MinPath) / selected(CurrentMin, CurrentMinCost) >> \
          [
              -selected(CurrentMin, CurrentMinCost),
              -runtime_path(MinPath),
              clear(),
              +plot() [{'to': 'robot@127.0.0.1:6566'}]
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
PHIDIAS.assert_belief(link('1', '2', 0.01))
PHIDIAS.assert_belief(link('2', '3', 0.01))
PHIDIAS.assert_belief(link('3', '4', 0.01))
PHIDIAS.assert_belief(link('4', '5', 0.01))
PHIDIAS.assert_belief(link('1', '6', 0.8))
PHIDIAS.assert_belief(link('2', '6', 0.8))
PHIDIAS.assert_belief(link('3', '6', 0.8))
PHIDIAS.assert_belief(link('4', '6', 0.8))
PHIDIAS.assert_belief(link('5', '6', 0.8))
PHIDIAS.assert_belief(link('6', '7', 0.65))
PHIDIAS.assert_belief(link('7', '8', 0.8))
PHIDIAS.assert_belief(link('7', '9', 0.8))
PHIDIAS.assert_belief(link('7', '10', 0.8))
PHIDIAS.assert_belief(link('7', '16', 0.65))
PHIDIAS.assert_belief(link('16', '11', 0.8))
PHIDIAS.assert_belief(link('16', '12', 0.8))
PHIDIAS.assert_belief(link('16', '13', 0.8))
PHIDIAS.assert_belief(link('16', '14', 0.8))
PHIDIAS.assert_belief(link('16', '15', 0.8))
PHIDIAS.assert_belief(link('11', '12', 0.01))
PHIDIAS.assert_belief(link('12', '13', 0.01))
PHIDIAS.assert_belief(link('13', '14', 0.01))
PHIDIAS.assert_belief(link('14', '15', 0.01))

PHIDIAS.assert_belief(link('2', '1', 0.01))
PHIDIAS.assert_belief(link('3', '2', 0.01))
PHIDIAS.assert_belief(link('4', '3', 0.01))
PHIDIAS.assert_belief(link('5', '4', 0.01))
PHIDIAS.assert_belief(link('6', '1', 0.8))
PHIDIAS.assert_belief(link('6', '2', 0.8))
PHIDIAS.assert_belief(link('6', '3', 0.8))
PHIDIAS.assert_belief(link('6', '4', 0.8))
PHIDIAS.assert_belief(link('6', '5', 0.8))
PHIDIAS.assert_belief(link('7', '6', 0.65))
PHIDIAS.assert_belief(link('8', '7', 0.8))
PHIDIAS.assert_belief(link('9', '7', 0.8))
PHIDIAS.assert_belief(link('10', '7', 0.8))
PHIDIAS.assert_belief(link('16', '7', 0.65))
PHIDIAS.assert_belief(link('11', '16', 0.8))
PHIDIAS.assert_belief(link('12', '16', 0.8))
PHIDIAS.assert_belief(link('13', '16', 0.8))
PHIDIAS.assert_belief(link('14', '16', 0.8))
PHIDIAS.assert_belief(link('15', '16', 0.8))
PHIDIAS.assert_belief(link('12', '11', 0.01))
PHIDIAS.assert_belief(link('13', '12', 0.01))
PHIDIAS.assert_belief(link('14', '13', 0.01))
PHIDIAS.assert_belief(link('15', '14', 0.01))


PHIDIAS.assert_belief(coordinates_block('1', 0.01, -0.016))
PHIDIAS.assert_belief(coordinates_block('2', 0.02, -0.016))
PHIDIAS.assert_belief(coordinates_block('3', 0.03, -0.016))
PHIDIAS.assert_belief(coordinates_block('4', 0.04, -0.016))
PHIDIAS.assert_belief(coordinates_block('5', 0.05, -0.016))
PHIDIAS.assert_belief(coordinates_block('6', 0.03, 0.07))
PHIDIAS.assert_belief(coordinates_block('7', 0.095, 0.07))
PHIDIAS.assert_belief(coordinates_block('8', 0.075, -0.016))
PHIDIAS.assert_belief(coordinates_block('9', 0.095, -0.016))
PHIDIAS.assert_belief(coordinates_block('10', 0.115, -0.016))
PHIDIAS.assert_belief(coordinates_block('11', 0.14, -0.016))
PHIDIAS.assert_belief(coordinates_block('12', 0.15, -0.016))
PHIDIAS.assert_belief(coordinates_block('13', 0.16, -0.016))
PHIDIAS.assert_belief(coordinates_block('14', 0.17, -0.016))
PHIDIAS.assert_belief(coordinates_block('15', 0.18, -0.016))
PHIDIAS.assert_belief(coordinates_block('16', 0.16, 0.07))

PHIDIAS.run_net(globals(), 'http')
PHIDIAS.shell(globals())

