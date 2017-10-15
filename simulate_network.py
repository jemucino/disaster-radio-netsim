# Based on networkx example at https://plot.ly/python/network-graphs/
from random import uniform

# import plotly.plotly as py
import plotly.offline as py
from plotly.graph_objs import *
import networkx as nx

# initialize plotly for offline mode
py.init_notebook_mode(connected=True)


class Node():
    STATES = ['rx','delay','tx','idle','fault']

    def __init__(self, config):
        self.config = config

        self.state = 'idle'
        self.delay_factor = 1

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

    def clear_messages(self):
        # reset the active message count before the next time step
        self.active_message_count = 0

    def tx_begin(self, msg, neighbors):
        if self.state == 'idle':
            self.msg_history.add(msg['id'])
            self.tx_timer = msg['length']
            self.state = 'tx'

    def transmit(self, msg, neighbors, time_step=1):
        if self.state == 'tx' or (self.state == 'delay' and self.delay_timer <= 0):
            self.__set_delay_factor()
            if self.tx_timer > 0:
                for neighbor in neighbors:
                    neighbor.new_message()
                self.tx_timer -= time_step
                self.state = 'tx'
            else:
                self.state = 'idle'
        elif self.state =='delay':
            self.delay_timer -= time_step

    def activate(self, msg, neighbors, time_step=1):
        next_state = None
        while next_state != self.state:
            if next_state in self.STATES:
                self.state = next_state

            if self.state == 'idle':
                if self.active_message_count > 1:
                    # if multiple nodes are talking to us, the messages are lost
                    next_state = 'fault'
                elif self.active_message_count == 1:
                    next_state = 'rx'
                else:
                    next_state = 'idle'
            elif self.state == 'rx':
                if self.active_message_count > 1:
                    # if multiple nodes are talking to us, the messages are lost
                    next_state = 'fault'
                elif self.active_message_count == 0: # this is a problem if consecutive messages but no overlap
                    if msg['id'] not in self.msg_history:
                        self.tx_timer = msg['length']
                        self.__set_delay_factor()
                        self.delay_timer = self.delay_factor*msg['length'] - time_step
                        self.msg_history.add(msg['id'])
                        next_state = 'delay'
                    else:
                        next_state = 'idle'
                else:
                    next_state = 'rx'
            elif self.state == 'fault':
                if self.active_message_count == 0: # this is a problem if consecutive messages but no overlap
                    next_state = 'idle'
                else:
                    next_state = 'fault'
            elif self.state == 'delay' or self.state =='tx':
                break
            else:
                # raise ValueError('At least one node is in an unexpected state.')
                print('A node was in an unexpected state: {}'.format(self.state))

        return self.state

    def __set_delay_factor(self):
        if self.config['delay_factor_randomize']:
            self.delay_factor = uniform(self.config['delay_factor_min'], self.config['delay_factor_max'])
        else:
            self.delay_factor = self.config['delay_factor_constant']


class NetSim():

    def __init__(self, config):
        # Initialize simulation
        # TODO: make config file or dict
        self.time_step = config['time_step']
        self.max_steps = config['max_steps']
        self.message = config['messages'][0]

        # if config['delay_factor_randomize']:
        #     self.delay_factor = uniform(config['delay_factor_min'], config['delay_factor_max'])
        # else:
        #     self.delay_factor = config['delay_factor_constant']

        # Create the graph
        self.network, self.starting_node = create_graph(self.time_step, config)

        # # Map nodes to connected neighbors
        # self.neighbors = {node: [neighbor for neighbor in nx.all_neighbors(self.network,node)] for node in self.network.nodes()}

        # Initialize time
        self.time = 0

        # Instantiate a visualization object for the simulation
        self.vis = Visualizer()

    def run_sim(self, decimation=1):
        # Begin tranmission on starting node
        neighbors = [self.network.node[node]['node_obj'] for node in nx.all_neighbors(self.network,self.starting_node)]
        self.network.node[self.starting_node]['node_obj'].tx_begin(self.message, neighbors)

        # Only need to define the edges once
        self.edge_trace = draw_edges(self.network)

        # Simulate the network
        for ii in range(self.max_steps):
            active_node_count = self.__step_sim()

            # If no nodes are active, terminate
            if active_node_count == 0:
                break

            # Draw the nodes every decimation number of steps (useful for long runs)
            if ii % decimation == 0:
                self.node_trace = draw_nodes(self.network)
                self.vis.add_data(self.edge_trace, self.node_trace, {'step_no': ii})

        metrics = self.analyze_network(ii, self.message)

        # Draw the nodes at the end of the simulation
        self.node_trace = draw_nodes(self.network)
        self.vis.add_data(self.edge_trace, self.node_trace, metrics)

    def __step_sim(self):
        active_node_count = 0

        # Iterate over all nodes and transmit
        for node in self.network.nodes():
            neighbors = [self.network.node[node_]['node_obj'] for node_ in nx.all_neighbors(self.network,node)]
            self.network.node[node]['node_obj'].transmit(self.message, neighbors, self.time_step)

        # Iterate over all nodes and perform transitions
        for node in self.network.nodes():
            neighbors = [self.network.node[node_]['node_obj'] for node_ in nx.all_neighbors(self.network,node)]
            state = self.network.node[node]['node_obj'].activate(self.message, neighbors, self.time_step)

            if state in ['delay','tx']:
                active_node_count += 1

        # Cleanup for next iteration
        for node in self.network.nodes():
            self.network.node[node]['node_obj'].clear_messages()

        # Move the simulation time forward
        self.__increment_time()

        return active_node_count

    def analyze_network(self, step_no, msg):
        nodes_reached = 0
        for node in self.network.nodes():
            if msg['id'] in self.network.node[node]['node_obj'].msg_history:
                nodes_reached += 1

        metrics = {'step_no': step_no,
                   'nodes_reached': nodes_reached,
                   'percent_reached': nodes_reached/config['network_size']*100,
                   }

        return metrics

    def __increment_time(self):
        self.time += self.time_step


def create_graph(time_step, config):
    size = config['network_size']
    max_distance = config['max_distance']
    x0, y0 = config['network_center']

    # Store position as node attribute data for random_geometric_graph
    G=nx.random_geometric_graph(size, max_distance)
    pos=nx.get_node_attributes(G,'pos')

    # Find node near center (x0, y0)
    dmin=1
    ncenter=0
    for n in pos:
        x,y=pos[n]
        d=(x-x0)**2+(y-y0)**2
        if d<dmin:
            ncenter=n
            dmin=d

    # Add Node object to each node
    node_config = config['node_config']
    for node in G.nodes():
        G.node[node]['node_obj'] = Node(node_config)

    return G, ncenter


def draw_edges(G):
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

    return edge_trace


def draw_nodes(G):
    # Add nodes as a scatter trace
    node_trace = Scatter(
        x=[],
        y=[],
        text=[],
        mode='markers',
        hoverinfo='text',
        marker=Marker(
            # showscale=True,
            # # colorscale options
            # # 'Greys' | 'Greens' | 'Bluered' | 'Hot' | 'Picnic' | 'Portland' |
            # # Jet' | 'RdBu' | 'Blackbody' | 'Earth' | 'Electric' | 'YIOrRd' | 'YIGnBu'
            # colorscale='YIOrRd',
            # reversescale=True,
            color=[],
            size=10,
            # colorbar=dict(
            #     thickness=15,
            #     title='Node Connections',
            #     xanchor='left',
            #     titleside='right'
            # ),
            line=dict(width=2)))

    for node in G.nodes():
        x, y = G.node[node]['pos']
        node_trace['x'].append(x)
        node_trace['y'].append(y)

    color = {'rx':'#66ff66','delay':'#ffff66','tx':'#3399ff','idle':'#ffffff','fault':'#ff3333'}

    # Color node points by the number of connections.
    for node in G:
        node_trace['marker']['color'].append(color[G.node[node]['node_obj'].state])
        node_info = 'State: {}'.format(G.node[node]['node_obj'].state)
        node_trace['text'].append(node_info)

    return node_trace


class Visualizer():

    def __init__(self):
        self.data = []

    def add_data(self, edge_trace, node_trace, metrics):
        title = '<br>Message propagation through network<br>(Step No. {})'.format(metrics['step_no'])


        if 'nodes_reached' in metrics and 'percent_reached' in metrics:
            text="The message reached {} nodes ({}% of the network)".format(metrics['nodes_reached'], metrics['percent_reached'])
        else:
            text="Running..."

        annotations = Annotations([Annotation(text=text,
                                              #  text="Python code: <a href='https://plot.ly/ipython-notebooks/network-graphs/'> https://plot.ly/ipython-notebooks/network-graphs/</a>",
                                              showarrow=False,
                                              xref='paper', yref='paper',
                                              x=0.005, y=-0.002
                                              )
                                  ])

        self.data.append({'data': Data([edge_trace, node_trace]),
                          'layout': Layout(title=title, annotations=annotations),
                          })

    def view_stills(self):
        for datum in self.data:
            # Create the graph
            fig = Figure(data=datum['data'],
                         layout=Layout(title='<br>Network graph made with Python',
                                       titlefont=dict(size=16),
                                       showlegend=False,
                                       hovermode='closest',
                                       margin=dict(b=20,l=5,r=5,t=40),
                                       annotations=[dict(text="Running...",
                                        #    text="Python code: <a href='https://plot.ly/ipython-notebooks/network-graphs/'> https://plot.ly/ipython-notebooks/network-graphs/</a>",
                                           showarrow=False,
                                           xref="paper", yref="paper",
                                           x=0.005, y=-0.002 )],
                                       xaxis=XAxis(showgrid=False, zeroline=False, showticklabels=False),
                                       yaxis=YAxis(showgrid=False, zeroline=False, showticklabels=False),
                                       ),
                            )

            py.iplot(fig, filename='networkx')

    def play_movie(self):
        # Create the graph
        fig = Figure(data=self.data[0]['data'],
                     layout=Layout(title='<br>Network graph made with Python',
                                   titlefont=dict(size=16),
                                   showlegend=False,
                                   hovermode='closest',
                                   margin=dict(b=20,l=5,r=5,t=40),
                                   annotations=[dict(text="Running...",
                                    #    text="Python code: <a href='https://plot.ly/ipython-notebooks/network-graphs/'> https://plot.ly/ipython-notebooks/network-graphs/</a>",
                                       showarrow=False,
                                       xref="paper", yref="paper",
                                       x=0.005, y=-0.002 )],
                                   updatemenus= [{'type': 'buttons',
                                                  'buttons': [{'label': 'Play','method': 'animate','args': [None]}]
                                                  }],
                                   xaxis=XAxis(showgrid=False, zeroline=False, showticklabels=False),
                                   yaxis=YAxis(showgrid=False, zeroline=False, showticklabels=False),
                                   ),
                        frames=self.data,
                        )

        py.plot(fig, filename='networkx')


if __name__ == '__main__':
    config = {'network_size': 400,
              'max_distance': 0.08,
              'network_center': (0.5,0.5),
              'time_step': 0.1,
              'max_steps': 1000,
              'messages': [{'id': 0, 'origin': 'center', 'length': 0.2}],  # length in seconds
              'node_config': {'delay_factor_constant': 1,
                              'delay_factor_randomize': True,
                              'delay_factor_min': 0.2,
                              'delay_factor_max': 5.0,
                              },
              }

    sim = NetSim(config)
    sim.run_sim()
    sim.vis.play_movie()
