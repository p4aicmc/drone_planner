# KML Map generator 

[![Python 3.10+](https://img.shields.io/badge/python-3.10%2B-blue.svg)](https://www.python.org/downloads/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Key Features

### Core Capabilities
- 🗺️ KML File Processing with Advanced Parsing
- 📦 Map Package Generation (JSON)

### Technical Highlights
- Qt5-based Modern GUI Framework
- Cross-Platform Compatibility

## Installation

### Prerequisites
- Python 3.10 or newer
- lxml, PyQt5

### Installing

```bash
pip install PyQt5 lxml
```

### Running the Application

Once you have installed the required libraries, you can run the application using the following command:

```bash
python3 mission_planner.py
```

## Usage

### Basic Workflow
1. Launch application: `mission-planner`
2. Navigate to KML file using explorer
3. Save the json file to the desired destination

## Development

### Architecture Overview
```
src/
├── kml_parser/       # KML processing core
└── mission_planner/  # Main application logic
```
