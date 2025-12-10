# Constella Swarm

Constella Swarm is a system for maintaining formation flight of multiple drones in GPS-denied environments. It leverages relative positioning among drones to achieve formation control based on swarm intelligence.

## Usage

Constella Swarm presents a Python Package that implements the core functionalities required for swarm formation flight, and initially designed to simulate the behavior of a swarm of drones.

To use the package, install it via pip:

```bash
pip install -e .
```

Then, you can import the package in Python scripts:

```python
import constellaswram
```

## Examples

An example script demonstrating the usage of the Constella Swarm package can be found in the `examples` directory. To run the example, execute:

```bash
cd examples
python simulation_4_drones.py
```

This script simulates a swarm of 4 drones maintaining formation flight, showcasing the usage of the Constella Swarm package.

A [rerun.io](https://rerun.io/) visualization of the simulation will also be launched automatically in this example.
