import pygame, sys
import random, math
from pygame import Rect
import math
from bfs import BestFirstSearch
import time
import random



class Ship(object):
    no_action, left_thrust, right_thrust, down_thrust= range(4)
    actions= [no_action, left_thrust, right_thrust, down_thrust]
    LATERAL_THRUST=0.2
    VERTICAL_THRUST=0.3

    def __cmp__(self, other):
        '''is needed for bestFirstSearch'''
        return 0 if norm(self.x, self.y, other.x, other.y)< 1 else 1
        
    def __init__(self,x=100,y=100, size=10):
        if isinstance(x, Ship):
            #clone
            self.x, self.y, self.size, self.vx, self.vy, self.ax, self.ay= x.x,x.y,x.size,x.vx,x.vy,x.ax,x.ay
        else:
            self.x, self.y= x,y
            self.vx, self.vy, self.ax, self.ay= 0.0,0.0,0.0,0.0
            self.size= size
        self.last_action= Ship.no_action
    
    def collides(self, rect_list):
        r= self.get_rect()
        return r.collidelist(rect_list)!=-1
        
    def execute(self,action):
        self.last_action= action
        if action == Ship.left_thrust:
            self.ax, self.ay= Ship.LATERAL_THRUST, 0
        elif action == Ship.right_thrust:
            self.ax, self.ay= -Ship.LATERAL_THRUST, 0
        elif action == Ship.down_thrust:
            self.ax, self.ay= 0, -Ship.VERTICAL_THRUST
        elif action == Ship.no_action:
            self.ax, self.ay= 0,0
        else:
            raise Exception("No such action")

        self.ay+= GRAVITY
        
        self.vx+= self.ax
        self.vy+= self.ay
        self.x+=self.vx
        self.y+=self.vy

        if self.collides(obstacles):
            self.x,self.y=10000, 100000

    def get_rect(self):
        size= self.size
        hs= size/2
        return Rect(self.x-hs,self.y-hs, size, size)
        
    def draw(self, screen):
        r= self.get_rect()
        hs=self.size/2
        pygame.draw.rect(screen, (255,255,255), r)
        if self.last_action==Ship.left_thrust:
            pygame.draw.rect(screen, (255,0,0), Rect(self.x-self.size, self.y-hs, hs, self.size))
        if self.last_action==Ship.right_thrust:
            pygame.draw.rect(screen, (255,0,0), Rect(self.x+hs, self.y-hs, hs, self.size))
        if self.last_action==Ship.down_thrust:
            pygame.draw.rect(screen, (255,0,0), Rect(self.x-hs, self.y+hs, self.size, hs))
        

class Obstacle(Rect):
    @staticmethod
    def generate_obstacle(width, height, size):
        import random
        x,y= random.random()*width, random.random()*height
        return Obstacle(x,y, size, size)
    def draw(self, screen):
        pygame.draw.rect(screen, (100,100,100), self)

class PathPlan():
    def __init__(self, block_number, block_size, actions):
        assert block_number==len(actions)>0
        self.actions= actions
        self._steps_number_total= block_number * block_size
        self.block_size, self.block_number= block_size, block_number
        self.reset()

    def reset(self):
        self._current_block, self._current_block_position= 0,0
        self._steps_number_left= self._steps_number_total

    def finished(self):
        return self._steps_number_left<=0
    def started(self):
        return self._steps_number_left!=self._steps_number_total
    def steps_done(self):
        return self._steps_number_total- self._steps_number_left
    def steps_left(self):
        return self._steps_number_left

    def _step(self):
        assert not self.finished()
        self._steps_number_left-=1
        self._current_block_position+=1
        if self._current_block_position>=self.block_size:
            self._current_block+=1
            self._current_block_position=0

    def step_ship(self, ship):
        action= self.actions[self._current_block]
        self._step()
        ship.execute(action)

    def _calculate_positions(self, ship):
        '''for drawing. iterates (a copy of the) ship through the
        plan, saving each x,y position. '''
        assert not self.started()
        #copy the ship, so as not to modify the original
        ship= Ship(ship) 
        positions= [[] for i in xrange(self.block_number)]
        while not self.finished():
            positions[self._current_block].append( (ship.x, ship.y) )
            self.step_ship(ship)
        self.reset()
        return positions
        
    def draw(self, ship, screen=None):
        '''For a given ship, draws it executing the plan.
        You can only call this method if the plan has not started
        executing OR this method was called uppon plan start with the
        same ship (in which case no recalculation will be performed and
        the result will be the same as the last call's)'''
        assert ship is not None
        if not hasattr(self, '_last_drawn_ship'):
            self._last_drawn_ship= None
            
        if not (ship is self._last_drawn_ship):
            self._last_drawn_ship= ship
            self._positions= self._calculate_positions(ship)

        if screen is not None:
            r,g,b= 255, 0, 127
            for block in self._positions:
                if len(block)>1:
                    pygame.draw.lines(screen, (r,g,b), False, block)
                    r= (r+ 67)%255
                    g= (g+101)%255
                    b= (b+ 31)%255

    def __repr__(self):
        return "%i/%i, %i/%i, %s" % (self._current_block, self.block_number-1, self._current_block_position, self.block_size-1, str(self.actions))




class ShipPathPlanner():
    def __init__(self, ship, objective):
        '''objective function is a function that should return a tuple (x,y)'''
        self.ship= Ship()
        self.objective= objective
        self.plan= None

    def generate_plan(self, start, goal, block_size, depth):
        '''create a new path plan'''
        my_ship_edge= lambda ship, edge: ship_edge(ship, edge, block_size)
    
        bfs= BestFirstSearch(start, goal, heuristic_score, ship_edges, my_ship_edge)
        score, finish_state= bfs.execute(depth)
        actions= BestFirstSearch.reconstruct_path(finish_state)
        if actions==[]: actions=[Ship.no_action]

        block_number= len(actions)
        plan= PathPlan(block_number, block_size, actions)
        #add a score attribute for later comparing
        plan.last_state_heuristic= score
        return plan

    def update_objective(self,objective):
        if self.objective!= objective:
            self.plan=None  #invalidate current plan
            self.objective=objective
        
    def shotgun_plan_generator(self):
        start= self.ship
        goal= Ship(*self.objective)
        block_size= heuristic_block_size(start, goal)
        depth= heuristic_depth(start, goal)

        plan_list= [self.generate_plan(start, goal, block_size, depth)]
        return plan_list

    def choose_best(self, list_of_plans):
        #sorts the plans. the last should be the better
        best= max(list_of_plans, key= heuristic_plan_score)
        return best

    def set_plan(self, plan):
        plan.draw(self.ship, screen=None)
        self.plan= plan

    def step(self):
        '''steps the ship, using the best plan available'''
        if self.plan==None or self.plan.finished():
            plans= self.shotgun_plan_generator()
            best= self.choose_best(plans)
            self.set_plan(best)

        if self.plan.steps_done()%10==0:
            #each 10 steps, recalculate a plan, compare to current, switch if best
            plans= self.shotgun_plan_generator() + [self.plan]
            best= self.choose_best(plans)
            if best!=self.plan:
                self.set_plan(best)
            
        self.plan.step_ship(self.ship)

    def draw(self, screen, draw_plan=True):
        self.ship.draw(screen)
        if draw_plan:
            self.plan.draw(self.ship, screen)







# AUX FUNCTIONS--------------------------------------/
def norm(x1,y1,x2,y2):
    dx, dy= x1-x2, y1-y2
    return math.sqrt(dx*dx+dy*dy)

def ship_edges(ship):
    '''the possible edges of a ship'''
    return Ship.actions

def ship_edge(ship_obj, edge_id, block_size):
    '''returns the node after following edge_id'''
    copy= Ship(ship_obj)
    for i in xrange(block_size):
        copy.execute(edge_id)
    return copy
#----------------------------------------/


# HEURISTICS-----------------------------------/
def heuristic_block_size(ship1, ship2):
    x1,y1,x2,y2= ship1.x, ship1.y, ship2.x, ship2.y
    '''the search is more efficient if actions are coarse if the target is too far, etc'''
    distance=           norm(x1,y1,x2,y2)
    d2= 6* distance ** 0.2
    block_size= 1+ int( d2)
    return block_size

def heuristic_depth(ship1, ship2):
    block_size= heuristic_block_size(ship1, ship2)
    #h= int(26 / (block_size**0.5 +3))
    h=5
    return h

def heuristic_score(ship_start, ship_goal):
    '''the BFS heuristic. bigger is worse'''
    x1,y1,x2,y2= ship_start.x, ship_start.y, ship_goal.x, ship_goal.y
    vx,vy= ship_start.vx, ship_start.vy
    distance= norm(x1,y1,x2,y2)
    speed= norm(0,0,vx,vy)
    return distance + 5*speed

def heuristic_plan_score(plan):
    h= plan.last_state_heuristic
    time= plan.steps_left()
    r= (-h)
    return r

#-------------------------------/




GRAVITY= 0.1
width, height= 800, 600
screen = pygame.display.set_mode((width, height))
clock= pygame.time.Clock()


path_planner= ShipPathPlanner(Ship(), (0,0) )
obstacles= [Obstacle.generate_obstacle(width, height,50) for i in xrange(20)]

while True:
    clock.tick(50)
    event = pygame.event.poll()
    if event.type == pygame.QUIT:
        break

    #   import ipdb; ipdb.set_trace()
    path_planner.update_objective( pygame.mouse.get_pos() )
    path_planner.step()
    screen.fill( (0, 0, 0) )
    path_planner.draw(screen, draw_plan=True)
   
    [o.draw(screen) for o in obstacles]
    pygame.display.flip()
