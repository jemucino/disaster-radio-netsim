# Based on networkx example at https://plot.ly/python/network-graphs/
# import plotly.plotly as py
import plotly.offline as py
from plotly.graph_objs import *
import networkx as nx

# initialize plotly for offline mode
py.init_notebook_mode(connected=True)


class State():
    STATES = ['rx','delay','tx','idle','blocked']

    def __init__(self, time_step=1):
        self.state = 'idle'
        self.time_step = time_step

        # keep track of how many nodes are trying to talk to us
        self.active_message_count = 0

        # keep message history to prevent re-transmission
        self.msg_history = set()

        # keep track of timers
        self.delay_timer = 0
        self.tx_timer = 0

    def new_message(self):
        # increment the active message count every time anode tries to talk to us
        self.active_message_count += 1

    def tx_begin(self, msg, neighbors):
        if self.state == 'idle':
            for neighbor in neighbors:
                neighbor.new_message()
            self.msg_history.add(msg['id'])
            self.tx_timer = 2 # TODO: this value should be calculated based on TBD
            self.state = 'tx'

    def activate(self, msg, neighbors):
        if self.state == 'idle' and self.active_message_count == 1:
            self.delay_timer = 2 # TODO: this value should be calculated based on TBD
            self.tx_timer = 2 # TODO: this value should be calculated based on TBD
            self.state = 'rx'
        elif self.state == 'rx' and self.active_message_count > 1:
            # if multiple nodes are talking to us, the messages are lost
            self.state = 'fault'
        elif self.state == 'rx' and self.active_message_count == 0:
            if msg['id'] not in self.msg_history:
                self.msg_history.add(msg['id'])
                self.state = 'delay'
            else:
                self.state = 'idle'
        elif self.state == 'fault' and self.active_message_count == 0:
            self.state = 'idle'
        elif self.state == 'delay':
            if self.delay_timer > 0:
                self.delay_timer -= self.time_step
            else:
                self.state = 'tx'
        elif self.state == 'tx':
            if self.tx_timer > 0:
                for neighbor in neighbors:
                    neighbor.new_message()
                self.tx_timer -= self.time_step
            else:
                self.state = 'idle'

        # reset the active message count for the next time step
        self.active_message_count = 0


class NetSim():

    def __init__(self):
        # Initialize simulation
        # TODO: make config file or dict
        self.time_step = 0.1
        self.max_steps = 1000
        self.starting_node = 0
        self.starting_message = {'id': 0}
        self.message_length = 2 # seconds TODO: randomize or calculate
        self.delay_factor = 2 # seconds TODO: randomize or calculate
        self.randomize = True

        # Create the graph
        self.network, self.edge_trace, self.node_trace = create_graph(self.time_step)
        self.network_size = 100 # TODO: this should be extracted from network

        # Initialize time
        self.time = 0

        # Begin simulation
        self.run_sim()

    def run_sim(self):
        neighbors = [self.network.node[node]['state'] for node in nx.all_neighbors(self.network,self.starting_node)]
        self.network.node[self.starting_node]['state'].tx_begin(self.starting_message, neighbors)

        for ii in range(self.max_steps):
            self.step_sim()

            if ii % 100 == 0:
                plot_graph(self.edge_trace, self.node_trace)

        print("The simulation ended at t = ".format(self.time))
        plot_graph(self.edge_trace, self.node_trace)

    def step_sim(self):
        for node in self.network:
            neighbors = [self.network.node[node_]['state'] for node_ in nx.all_neighbors(self.network,node)]
            self.network.node[node]['state'].activate(self.starting_message, neighbors)

        self.__increment_time()

    def __increment_time(self):
        self.time += self.time_step


def create_graph(time_step):
    # Store position as node attribute data for random_geometric_graph
    G=nx.random_geometric_graph(100,0.125)
    pos=nx.get_node_attributes(G,'pos')

    # Find node near center (0.5, 0.5)
    dmin=1
    ncenter=0
    for n in pos:
        x,y=pos[n]
        d=(x-0.5)**2+(y-0.5)**2
        if d<dmin:
            ncenter=n
            dmin=d

    # # Compute the shortest path lengths from source to all reachable nodes
    # p=nx.single_source_shortest_path_length(G,ncenter)

    # Add edges as disconnected lines in a single trace
    edge_trace = Scatter(
        x=[],
        y=[],
        line=Line(width=0.5,color='#888'),
        hoverinfo='none',
        mode='lines')

    for edge in G.edges():
        x0, y0 = G.node[edge[0]]['pos']
        x1, y1 = G.node[edge[1]]['pos']
        edge_trace['x'] += [x0, x1, None]
        edge_trace['y'] += [y0, y1, None]

    # Add nodes as a scatter trace
    node_trace = Scatter(
        x=[],
        y=[],
        text=[],
        mode='markers',
        hoverinfo='text',
        marker=Marker(
            showscale=True,
            # colorscale options
            # 'Greys' | 'Greens' | 'Bluered' | 'Hot' | 'Picnic' | 'Portland' |
            # Jet' | 'RdBu' | 'Blackbody' | 'Earth' | 'Electric' | 'YIOrRd' | 'YIGnBu'
            colorscale='YIGnBu',
            reversescale=True,
            color=[],
            size=10,
            colorbar=dict(
                thickness=15,
                title='Node Connections',
                xanchor='left',
                titleside='right'
            ),
            line=dict(width=2)))

    for node in G.nodes():
        x, y = G.node[node]['pos']
        node_trace['x'].append(x)
        node_trace['y'].append(y)
        # Add state attribute to each node
        G.node[node]['state'] = State(time_step)

    # # Color node points by the number of connections.
    # for node, adjacencies in enumerate(G.adjacency_list()):
    #     node_trace['marker']['color'].append(len(adjacencies))
    #     node_info = '# of connections: '+str(len(adjacencies))
    #     node_trace['text'].append(node_info)

    return G, edge_trace, node_trace


def augment_nodes(G, node_trace):
    # Color node points by the number of connections.
    for _, adjacencies in enumerate(G.adjacency_list()):
        node_trace['marker']['color'].append(len(adjacencies))
        node_info = '# of connections: '+str(len(adjacencies))
        node_trace['text'].append(node_info)


def plot_graph(edge_trace, node_trace):
    # Create the graph
    fig = Figure(data=Data([edge_trace, node_trace]),
                 layout=Layout(
                    title='<br>Network graph made with Python',
                    titlefont=dict(size=16),
                    showlegend=False,
                    hovermode='closest',
                    margin=dict(b=20,l=5,r=5,t=40),
                    annotations=[ dict(
                        text="Python code: <a href='https://plot.ly/ipython-notebooks/network-graphs/'> https://plot.ly/ipython-notebooks/network-graphs/</a>",
                        showarrow=False,
                        xref="paper", yref="paper",
                        x=0.005, y=-0.002 ) ],
                    xaxis=XAxis(showgrid=False, zeroline=False, showticklabels=False),
                    yaxis=YAxis(showgrid=False, zeroline=False, showticklabels=False)))

    py.iplot(fig, filename='networkx')

if __name__ == '__main__':
    sim = NetSim()
