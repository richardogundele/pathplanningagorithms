# A* Path Planning Algorithm

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

A Python implementation of the Search algorithm for path planning in a grid-based environment. This project allows you to find the optimal path from a starting point to a goal point while avoiding obstacles.

## Table of Contents

- [Introduction](#introduction)
- [Features](#features)
- [Getting Started](#getting-started)
  - [Prerequisites](#prerequisites)
  - [Installation](#installation)
- [Usage](#usage)
  - [Configuring Parameters](#configuring-parameters)
- [Results](#results)
- [Contributing](#contributing)
- [License](#license)

## Introduction

Path planning is a fundamental problem in robotics and autonomous systems. 

## Features

- Depth First search algorithm implementation.
- Breadth First search algorithm implementation.
- A* search algorithm implementation.
- Uniform Cost Search algorithm implementation.
- Customizable motion type (4n or 8n) for path generation.
- Choice of heuristic calculation (manhattan or euclidean) for estimating costs.
- Dynamic obstacle support for real-world simulations.
- Memory and execution time measurement.
- Interactive visualization of the path planning process using Matplotlib.

## Getting Started

### Prerequisites

- Python 3.x
- Matplotlib (for visualization)
- NumPy (for array manipulation)
- pandas (for reading configuration data)
- (Optional) tracemalloc (for memory tracking)

### Installation

1. Clone the repository:

   ```bash
   git clone https://github.com/yourusername/path-planning.git

pip install matplotlib numpy pandas tracemalloc

