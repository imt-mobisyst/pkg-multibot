#!/usr/bin/python3 discovery_server

# Script based on this file : https://docs.ros.org/en/iron/_downloads/617a6849c029c43a931e24db314cb224/discovery_packets.py

"""Script to count number of rtps packages in a tcpdump capture."""
import os
import subprocess
import pandas as pd
import numpy as np

import matplotlib.pyplot as plt

current = os.path.dirname(os.path.abspath(__file__))
inputFolder  = os.path.join(current, "data")
outputFolder = os.path.join(current, "results")


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


def count_packets(filename):
    """Count the discovery packets filtering them by Entity id."""
    print('Processing {}'.format(filename))
    command = 'tshark -r {} -Y'.format(filename)
    command += ' "rtps'
    # command += ' && ((rtps.sm.rdEntityId == 0x000002c2) ||'
    # command +=     ' (rtps.sm.rdEntityId == 0x000002c7) ||'
    # command +=     ' (rtps.sm.rdEntityId == 0x000003c2) ||'
    # command +=     ' (rtps.sm.rdEntityId == 0x000003c7) ||'
    # command +=     ' (rtps.sm.rdEntityId == 0x000004c2) ||'
    # command +=     ' (rtps.sm.rdEntityId == 0x000004c7) ||'
    # command +=     ' (rtps.sm.rdEntityId == 0x000100c2) ||'
    # command +=     ' (rtps.sm.rdEntityId == 0x000100c7) ||'
    # command +=     ' (rtps.sm.rdEntityId == 0x000200C2) ||'
    # command +=     ' (rtps.sm.rdEntityId == 0x000200C7) ||'
    # command +=     ' (rtps.sm.rdEntityId == 0x000201C3) ||'
    # command +=     ' (rtps.sm.rdEntityId == 0x000201C4))'
    command += '" | wc -l'

    res = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE)
    n_packets = int(res.communicate()[0].decode().rstrip())
    return n_packets


def packet_evolution(filename):

    command = 'tshark -r {} -Y'.format(filename)
    command += ' "rtps'
    # command += ' && ((rtps.sm.rdEntityId == 0x000002c2) ||'
    # command +=     ' (rtps.sm.rdEntityId == 0x000002c7) ||'
    # command +=     ' (rtps.sm.rdEntityId == 0x000003c2) ||'
    # command +=     ' (rtps.sm.rdEntityId == 0x000003c7) ||'
    # command +=     ' (rtps.sm.rdEntityId == 0x000004c2) ||'
    # command +=     ' (rtps.sm.rdEntityId == 0x000004c7) ||'
    # command +=     ' (rtps.sm.rdEntityId == 0x000100c2) ||'
    # command +=     ' (rtps.sm.rdEntityId == 0x000100c7) ||'
    # command +=     ' (rtps.sm.rdEntityId == 0x000200C2) ||'
    # command +=     ' (rtps.sm.rdEntityId == 0x000200C7) ||'
    # command +=     ' (rtps.sm.rdEntityId == 0x000201C3) ||'
    # command +=     ' (rtps.sm.rdEntityId == 0x000201C4))'

    res = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE)

    lines = res.communicate()[0].split('\n')

    cleanedLines = [" ".join(l.split()) for l in lines if l.strip() != ""]

    times = [float(l.strip().split(' ')[1]) for l in cleanedLines]
    
    # Define the time (time ranges)
    ranges = np.arange(np.floor(np.min(times)), np.ceil(np.max(times)), 0.1)  # Create ranges with step size of 0.1

    # Count the number of values in each range
    counts, _ = np.histogram(times, ranges)

    return ranges, counts




if __name__ == '__main__':

    # Load tcpdump files
    files = [
        'namespaces.pcapng',
        'domain_id.pcapng',
        'discovery_server.pcapng',
        'partitions.pcapng'
    ]

    data = pd.DataFrame()
    methods = []
    nbPackets = []
    packetEvolution = []

    # Process data
    for f in files:
        f = os.path.join(inputFolder, f)
        if not os.path.isfile(f):
            print('Cannot find file {}'.format(f))
            continue

        method = 'Namespaces'
        if 'server' in f:
            method = 'DDS Discovery Server'
        elif 'domain' in f:
            method = 'Domain ID'
        elif 'partition' in f:
            method = 'DDS partitions'

        methods.append(method)
        nbPackets.append(count_packets(f))

        packetEvolution.append(packet_evolution(f))

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
