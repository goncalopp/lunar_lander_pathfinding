from Queue import PriorityQueue

class BestFirstSearch(object):
    '''edge_list_function takes as argument a node and returns a list 
    of edge identifiers for that node.
    edge_function takes as argument node1 and a edge identifier, and
    returns node2 (node2 being accessible from node1 by edge_identifier)'''
    def __init__(self, initial_node, desired_node, heuristic_function, edge_list_function, edge_function):
        self.start, self.goal= initial_node, desired_node
        self.h, self.el, self.e= heuristic_function, edge_list_function, edge_function
        self.queue= PriorityQueue()
        self.start.bfs_depth=0
        self.queue.put(self.score([self.start])[0])

    def score(self, list_of_states):
        h= self.h
        goal= self.goal
        scores= [(h(s, goal), s) for s in list_of_states]
        return scores

    def construct_state(self,current_state, edge_id):
        newstate= self.e(current_state, edge_id)
        newstate.bfs_parent= current_state
        newstate.bfs_parent_edge= edge_id
        newstate.bfs_depth= current_state.bfs_depth+1
        return newstate

    def execute(self, depth_limit=3):
        h, el, e= self.h, self.el, self.e

        minimum_found= self.score([self.start])[0]
        
        while not self.queue.empty():
            score, state= self.queue.get()
            if score< minimum_found[0]:
                minimum_found= (score, state)
                
            if state==self.goal:
                return score, state
            if state.bfs_depth>depth_limit:
                #return minimum_found
                return score, state
            edges= el(state)
            newstates= [self.construct_state(state,edge_id) for edge_id in edges]
            scores_states= self.score(newstates)
            for x in scores_states:
                self.queue.put(x)

    @staticmethod
    def reconstruct_path(state):
        '''traces from goal to start, returns ordered list of choices
        (edge_id s)'''
        path=[]
        while hasattr(state, 'bfs_parent'):
            path.append(state.bfs_parent_edge)
            state= state.bfs_parent
        path.reverse()
        return path
        
