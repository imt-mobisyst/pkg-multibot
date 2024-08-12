#!/usr/bin/python3 discovery_server

# Script based on this file : https://docs.ros.org/en/iron/_downloads/617a6849c029c43a931e24db314cb224/discovery_packets.py

"""Script to count number of rtps packages in a tcpdump capture."""
import os
import subprocess
import pandas as pd
import numpy as np

import matplotlib.pyplot as plt

# Path definition
current = os.path.dirname(os.path.abspath(__file__))
inputFolder  = os.path.join(current, "data")
outputFolder = os.path.join(current, "results")

# Plot functions
def plot_packets(data):
    """Plot a graph with the result obtained."""
    ax = data.plot.bar(x='discovery_architecture', y='discovery_packets', rot=0)
    plt.title('Packet traffic during discovery')
    plt.xlabel('Discovery Architecture')
    plt.ylabel('Number of packets')
    plt.legend().remove()
    # plt.show()

    path = os.path.join(outputFolder, 'discovery_packets.png')
    plt.savefig(path)
    plt.close()

    return path



def plot_packets_evolution(data, methods):
    """Plot a graph with the evolution of the packet traffic obtained."""
    for i in range(len(data)):
        ranges, counts = data[i]
        method = methods[i]

        plt.plot(ranges[:-1], counts,  marker='o', linestyle='-', linewidth=1, markersize=2, label=method)

    
    plt.title('Packet traffic evolution over time')
    plt.xlabel('Time')
    plt.ylabel('Number of packets')
    # plt.show()

    path = os.path.join(outputFolder, 'evolution.png')
    plt.savefig(path)
    plt.close()

    return path

# Data extraction functions
def createFilter(method):
    if method == 'Zenoh':
        return '"Zenoh"'
    
    return '"rtps"'

def count_packets(filename, filter):
    """Count the discovery packets"""

    command = f'tshark -r {filename} -Y {filter} | wc -l'

    res = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE)
    n_packets = int(res.communicate()[0].decode().rstrip())
    return n_packets


def packet_evolution(filename, filter):
    """Get the times of the discovery packets"""

    command = f'tshark -r {filename} -Y {filter}'

    res = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE)
    lines = res.communicate()[0].decode().split('\n')

    cleanedLines = [" ".join(l.split()) for l in lines if l.strip() != ""]

    times = [float(l.strip().split(' ')[1]) for l in cleanedLines]
    times = [t for t in times if t < 30]

    
    # Define the time (time ranges)
    ranges = np.arange(np.floor(np.min(times)), np.ceil(np.max(times)), 0.1)  # Create ranges with step size of 0.1

    # Count the number of values in each range
    counts, _ = np.histogram(times, ranges)

    return ranges, counts




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
    nbPackets = []
    packetEvolution = []

    # Process data
    for method, f in files.items():

        f = os.path.join(inputFolder, f)
        if not os.path.isfile(f):
            print('Cannot find file {}'.format(f))
            continue

        print(f'Processing {f}')


        filter = createFilter(method)

        # Extract data from tcpdump
        nbPackets.append(count_packets(f, filter))
        packetEvolution.append(packet_evolution(f, filter))



    methods = list(files.keys())

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
