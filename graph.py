#import pydot_ng as pydot
import networkx as nx
import matplotlib.pyplot as plt

class NodeNotInGraph(Exception):
    def __init__(self, node):
        self.node = node

    def __str__(self):
        return "Node %s not in graph." % str(self.node)


class Edge(object):
    def __init__(self, source, target, weight=1.0):
        self.source = source
        self.target = target
        self.weight = weight

    def __hash__(self):
        return hash("%s_%s_%f" % (self.source, self.target, self.weight))

    def __eq__(self, other):
        return self.source == other.source and self.target == other.target \
            and self.weight == other.weight
    def __repr__(self):
        return "Edge(%r,%r,%r)" % (self.source, self.target, self.weight)


class Graph(object):
    def __init__(self, node_label_fn=None):
        self._nodes = set()
        self._edges = dict()
        self.node_label_fn = node_label_fn if node_label_fn else lambda x: x
        self.node_positions = dict()

    def __contains__(self, node):
        return node in self._nodes

    def add_node(self, node):
        """Adds a node to the graph."""
        self._nodes.add(node)
    
    def add_edge(self, node1, node2, weight=1.0, bidirectional=True):
        """Adds an edge between node1 and node2. Adds the nodes to the graph first
        if they don't exist."""
        self.add_node(node1)
        self.add_node(node2)
        node1_edges = self._edges.get(node1, set())
        node1_edges.add(Edge(node1, node2, weight))
        self._edges[node1] = node1_edges
        if bidirectional:
                node2_edges = self._edges.get(node2, set())
                node2_edges.add(Edge(node2, node1, weight))
                self._edges[node2] = node2_edges

    def set_node_positions(self, positions):
        self.node_positions = positions

    def set_node_pos(self, node, pos):
        """Sets the (x,y) pos of the node, if it exists in the graph."""
        if not node in self:
            raise NodeNotInGraph(node)
        self.node_positions[node] = pos

    def get_node_pos(self, node):
        if not node in self:
            raise NodeNotInGraph(node)
        return self.node_positions[node]

    def node_edges(self, node):
        if not node in self:
            raise NodeNotInGraph(node)
        return self._edges.get(node, set())

    def draw(self, highlight_edges=None):
        nxg = nx.DiGraph()
        edges = [(e.source, e.target, {'weight':e.weight, 'inv_weight':1.0/e.weight}) for node_set in self._edges.values() for e in node_set]
        nxg.add_edges_from(edges)
        if len(self.node_positions) < len(self._nodes):
            # Calculate positions for nodes whose pos is not specified.
            pos = nx.spring_layout(nxg, weight='inv_weight', pos=self.node_positions, fixed=self.node_positions.keys() if self.node_positions else None)
        else:
            pos = self.node_positions

        f = plt.figure(figsize=(12,12))
        plt.gca().set_aspect('equal', adjustable='box')
        nx.draw_networkx_nodes(nxg, pos, node_color='w')
        nx.draw_networkx_edges(nxg, pos, edges)
        nx.draw_networkx_labels(nxg, pos)
        edge_labels=dict([((u,v,),"%s" % d['weight'])
                 for u,v,d in nxg.edges(data=True)])
        nx.draw_networkx_edge_labels(nxg, pos, edge_labels=edge_labels)


        if highlight_edges:
            nx.draw_networkx_edges(nxg, pos, highlight_edges, edge_color='r')
        
        plt.axis('off')
        plt.show()

    def draw_edges(self, edges):
        # print edges
        nx.draw_networkx_edges(nxg, pos, edges, edge_color='r')
        reduced_labels = {(u,v): edge_labels[(u,v)] for u,v,_ in edges}
        nx.draw_networkx_edge_labels(nxg, pos, edge_labels=reduced_labels, font_color='r')
        
        reduced_nodes = set([u for u,_,_ in edges])
        reduced_nodes.update([v for _,v,_ in edges])
        # nx.draw_networkx_nodes(nxg, pos, nodelist=reduced_nodes,  node_color='r')
        red_labels = {n:n for n in reduced_nodes}
        print red_labels
        nx.draw_networkx_labels(nxg, pos, labels=red_labels, font_color='r')

    def highlight_edges(self, edges):
        nx.draw_networkx_edges(nxg, pos, edges, edge_color='r')
        reduced_labels = {(u,v): edge_labels[(u,v)] for u,v,_ in edges}
        nx.draw_networkx_edge_labels(nxg, pos, edge_labels=reduced_labels, font_color='r')
        
        reduced_nodes = set([u for u,_,_ in edges])
        reduced_nodes.update([v for _,v,_ in edges])
        # nx.draw_networkx_nodes(nxg, pos, nodelist=reduced_nodes,  node_color='r')
        red_labels = {n:n for n in reduced_nodes}
        print red_labels
        nx.draw_networkx_labels(nxg, pos, labels=red_labels, font_color='r')
    '''
    def _create_dot_graph(self):
        dot_graph = pydot.Dot(graph_type='digraph', concentrate=True, rankdir="LR")
        dot_graph.set_node_defaults(shape='rect', fontsize=12)
        for n in self._nodes:
            node_name = self.node_label_fn(n)
            node = pydot.Node(shape="ellipse", name=node_name)
            if n in self.node_positions:
                node.set_pos("%d,%d!" % (self.node_positions[n][0], self.node_positions[n][1]))
            dot_graph.add_node(node)
        for src_node, edges in self._edges.items():
            for e in edges:
                dot_graph.add_edge(pydot.Edge(self.node_label_fn(src_node), self.node_label_fn(e.target), label=e.weight if e.weight!=1.0 else ""))
        return dot_graph

    def _repr_svg_(self):
        return self._create_dot_graph().create_svg()
        '''
