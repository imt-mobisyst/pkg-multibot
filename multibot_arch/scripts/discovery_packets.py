#!/usr/bin/python3 discovery_server

# Script based on this file : https://docs.ros.org/en/iron/_downloads/617a6849c029c43a931e24db314cb224/discovery_packets.py

"""Script to count the number of rtps/zenoh packages in a tcpdump capture, and see their evolution."""
import os
import subprocess
import pandas as pd
import numpy as np

import matplotlib.pyplot as plt

# Path definition
current = os.path.dirname(os.path.abspath(__file__))
inputFolder  = os.path.join(current, "data")
outputFolder = os.path.join(current, "results")

nbSeconds = 30

# Plot functions
def plot_packets(data):
    """Plot a graph with the result obtained."""
    ax = data.plot.bar(x='discovery_architecture', y='discovery_packets', rot=0)
    ax.bar_label(ax.containers[0])
    
    plt.title('Packet traffic during discovery')
    plt.xlabel('Discovery Architecture')
    plt.ylabel(f'Number of packets in {nbSeconds}s')
    plt.legend().remove()
    # plt.show()

    path = os.path.join(outputFolder, 'discovery_packets.png')
    plt.savefig(path)
    plt.close()

    return path



def plot_packets_evolution(times, methods):
    """Plot a graph with the evolution of the packet traffic obtained."""
    for i in range(len(data)):

        # Define the time ranges
        ranges = np.arange(np.floor(np.min(times[i])), np.ceil(np.max(times[i])), 0.1)  # Create ranges with step size of 0.1
        # Count the number of values in each range
        counts, _ = np.histogram(times[i], ranges)
        
        method = methods[i]

        plt.plot(ranges[:-1], counts,  marker='o', linestyle='-', linewidth=1, markersize=2, label=method)

    
    plt.title('Packet traffic evolution over time')
    plt.xlabel('Time (s)')
    plt.ylabel('Number of packets')
    # plt.show()

    path = os.path.join(outputFolder, 'evolution.png')
    plt.savefig(path)
    plt.close()

    return path

# Data extraction functions
def createFilter(method):
    """Create a filter for the tshark cli command"""
    filter = 'rtps && ip.src != 127.0.0.1 && ip.dst != 127.0.0.1'

    if method == 'Zenoh':
        filter = f'({filter}) || zenoh'
    
    return filter


def extractPacketData(filename, filter):
    """Get the times and total number of discovery packets"""

    # Filter tcpdump output
    command = f'tshark -r {filename} -Y "{filter}"'

    res = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE)
    lines = res.communicate()[0].decode().split('\n')

    # Clean output
    cleanedLines = [l.strip().split() for l in lines if l.strip() != ""]
    filteredLines = [l for l in cleanedLines if float(l[1]) < nbSeconds]

    # Extract data

    ## Timestamps
    times = [float(l[1]) for l in filteredLines if len(l)] 

    ## Nb packets
    nbPackets = len(filteredLines)

    ## Nb subnet packets

    return nbPackets, times




if __name__ == '__main__':

    # Create output folder
    if not os.path.isdir(outputFolder):
        os.makedirs(outputFolder)

    # Load tcpdump files
    files = {
        'Domain ID': 'domain_id.pcapng',
        'Zenoh': 'zenoh.pcapng'
        # 'Namespaces': 'namespaces.pcapng',
        # 'DDS Discovery Server': 'discovery_server.pcapng',
        # 'DDS partitions': 'partitions.pcapng',
    }

    data = pd.DataFrame()
    methods = []
    nbPackets = []
    packetEvolution = []

    # Process data
    for method, f in files.items():

        f = os.path.join(inputFolder, f)
        if not os.path.isfile(f):
            print('Cannot find file {}'.format(f))
            continue

        print(f'Processing {f}')
        methods.append(method)

        filter = createFilter(method)

        # Extract data from tcpdump
        n, times = extractPacketData(f, filter)

        nbPackets.append(n)
        packetEvolution.append(times)

    data = pd.DataFrame(
        {
            'discovery_architecture': methods,
            'discovery_packets': nbPackets
        }
    )


    # Plot data

    ## Comparison of the number of packets during discovery
    comparison_path = plot_packets(data)

    ## Evolution of the number of packets over time
    evolution_path = plot_packets_evolution(packetEvolution, methods)

    print(f'Files created in :\n - {comparison_path}\n - {evolution_path}')
