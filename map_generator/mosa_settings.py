import json

import requests
from PyQt5 import QtCore, QtGui, QtWidgets


class MosaSettingsWindow(QtWidgets.QDialog):
    def __init__(self, parent=None, initial_settings=None):
        super().__init__(parent)

        self.setWindowTitle("Configurações MOSA")
        self.setMinimumSize(600, 400)

        # Initialize data structures
        self.mosa_data = None  # Will hold data fetched from server

        # Fetch MOSA configurations from server
        self.fetch_mosa_configurations()

        # Initialize UI components
        self.init_ui()

        # Set initial settings
        if initial_settings:
            self.set_initial_settings(initial_settings)

    def fetch_mosa_configurations(self):
        try:
            response = requests.get(
                "https://patronrouteplanner.zapto.org/api/v2/get_mosa"
            )
            response.raise_for_status()
            self.mosa_data = response.json()
        except requests.RequestException as e:
            QtWidgets.QMessageBox.critical(
                self,
                "Erro",
                f"Não foi possível obter as configurações MOSA do servidor:\n{str(e)}",
            )
            # Use default data if server request fails
            self.mosa_data = {
                "Version": "0.1.0",
                "MissionTypes": ["Area survey", "Corridor survey"],
                "TargetAltitudes": ["ASL", "HOME", "AGL", "AGL+"],
                "VehicleTypes": ["Copter", "Plane", "VTOL"],
                "Aircrafts": [],
                "MOSAs": [],
                "Optimizations": [],
            }

    def init_ui(self):
        layout = QtWidgets.QVBoxLayout()

        # Aircraft selection
        aircraft_layout = QtWidgets.QHBoxLayout()
        aircraft_label = QtWidgets.QLabel("Aeronave:")
        self.aircraft_combo = QtWidgets.QComboBox()
        for aircraft in self.mosa_data.get("Aircrafts", []):
            self.aircraft_combo.addItem(
                f"{aircraft['Name']} ({aircraft['Type']})", aircraft
            )
        self.aircraft_combo.currentIndexChanged.connect(self.display_aircraft_image)
        aircraft_layout.addWidget(aircraft_label)
        aircraft_layout.addWidget(self.aircraft_combo)

        # MOSA selection
        mosa_layout = QtWidgets.QHBoxLayout()
        mosa_label = QtWidgets.QLabel("MOSA:")
        self.mosa_combo = QtWidgets.QComboBox()
        for mosa in self.mosa_data.get("MOSAs", []):
            self.mosa_combo.addItem(mosa["Name"], mosa)
        self.mosa_combo.currentIndexChanged.connect(self.update_advanced_configurations)
        mosa_layout.addWidget(mosa_label)
        mosa_layout.addWidget(self.mosa_combo)

        # Optimization selection
        optimization_layout = QtWidgets.QHBoxLayout()
        optimization_label = QtWidgets.QLabel("Otimização:")
        self.optimization_combo = QtWidgets.QComboBox()
        for opt in self.mosa_data.get("Optimizations", []):
            opt_name_pt = {
                "Low": "Baixa",
                "Middle": "Média",
                "High": "Alta",
            }.get(opt["Name"], opt["Name"])
            self.optimization_combo.addItem(opt_name_pt, opt)
        optimization_layout.addWidget(optimization_label)
        optimization_layout.addWidget(self.optimization_combo)

        # Target Altitude selection
        target_altitude_layout = QtWidgets.QHBoxLayout()
        target_altitude_label = QtWidgets.QLabel("Altitude Alvo:")
        self.target_altitude_combo = QtWidgets.QComboBox()
        for altitude in self.mosa_data.get("TargetAltitudes", []):
            self.target_altitude_combo.addItem(altitude)
        target_altitude_layout.addWidget(target_altitude_label)
        target_altitude_layout.addWidget(self.target_altitude_combo)

        # Flight Altitude
        flight_altitude_layout = QtWidgets.QHBoxLayout()
        flight_altitude_label = QtWidgets.QLabel("Altitude de Voo:")
        self.flight_altitude_spin = QtWidgets.QSpinBox()
        self.flight_altitude_spin.setRange(100, 500)
        self.flight_altitude_spin.setValue(100)
        flight_altitude_layout.addWidget(flight_altitude_label)
        flight_altitude_layout.addWidget(self.flight_altitude_spin)

        # Line Distance
        line_distance_layout = QtWidgets.QHBoxLayout()
        line_distance_label = QtWidgets.QLabel("Distância entre Linhas:")
        self.line_distance_spin = QtWidgets.QSpinBox()
        self.line_distance_spin.setRange(10, 100)
        self.line_distance_spin.setValue(20)
        line_distance_layout.addWidget(line_distance_label)
        line_distance_layout.addWidget(self.line_distance_spin)

        # Waypoints Distance
        waypoints_distance_layout = QtWidgets.QHBoxLayout()
        waypoints_distance_label = QtWidgets.QLabel("Distância entre Waypoints:")
        self.waypoints_distance_spin = QtWidgets.QSpinBox()
        self.waypoints_distance_spin.setRange(5, 300)
        self.waypoints_distance_spin.setValue(40)
        waypoints_distance_layout.addWidget(waypoints_distance_label)
        waypoints_distance_layout.addWidget(self.waypoints_distance_spin)

        # Aircraft Image
        self.aircraft_image_label = QtWidgets.QLabel()
        self.aircraft_image_label.setAlignment(QtCore.Qt.AlignCenter)
        self.aircraft_image_label.setFixedSize(200, 150)
        self.display_aircraft_image()

        # Save and Cancel buttons
        buttons_layout = QtWidgets.QHBoxLayout()
        self.save_button = QtWidgets.QPushButton("Salvar Configurações")
        self.save_button.clicked.connect(self.save_advanced_configurations)
        cancel_button = QtWidgets.QPushButton("Cancelar")
        cancel_button.clicked.connect(self.close)
        buttons_layout.addWidget(self.save_button)
        buttons_layout.addWidget(cancel_button)

        # Add all layouts to the main layout
        layout.addLayout(aircraft_layout)
        layout.addWidget(self.aircraft_image_label)
        layout.addLayout(mosa_layout)
        layout.addLayout(optimization_layout)
        layout.addLayout(target_altitude_layout)
        layout.addLayout(flight_altitude_layout)
        layout.addLayout(line_distance_layout)
        layout.addLayout(waypoints_distance_layout)
        layout.addLayout(buttons_layout)

        self.setLayout(layout)

        # Initialize configurations
        self.update_advanced_configurations()

    def set_initial_settings(self, initial_settings):
        # Set Aircraft selection
        selected_aircraft_name = initial_settings.get("Aircraft", "VT-14")
        for i in range(self.aircraft_combo.count()):
            aircraft = self.aircraft_combo.itemData(i)
            if aircraft["Name"] == selected_aircraft_name:
                self.aircraft_combo.setCurrentIndex(i)
                break

        # Set Optimization selection
        selected_optimization_name = initial_settings.get("Optimization", "Middle")
        for i in range(self.optimization_combo.count()):
            opt = self.optimization_combo.itemData(i)
            if opt["Name"] == selected_optimization_name:
                self.optimization_combo.setCurrentIndex(i)
                break

        # Set MOSA selection
        selected_mosa_name = initial_settings.get("MOSA", "MOSA1")
        for i in range(self.mosa_combo.count()):
            mosa = self.mosa_combo.itemData(i)
            if mosa["Name"] == selected_mosa_name:
                self.mosa_combo.setCurrentIndex(i)
                break

    def display_aircraft_image(self):
        current_aircraft = self.aircraft_combo.currentData()
        if current_aircraft and current_aircraft.get("Image"):
            image_url = current_aircraft["Image"]
            image = self.load_image_from_url(image_url)
            if image:
                pixmap = QtGui.QPixmap.fromImage(image)
                scaled_pixmap = pixmap.scaled(
                    self.aircraft_image_label.size(),
                    QtCore.Qt.KeepAspectRatio,
                    QtCore.Qt.SmoothTransformation,
                )
                self.aircraft_image_label.setPixmap(scaled_pixmap)
            else:
                self.aircraft_image_label.clear()
        else:
            self.aircraft_image_label.clear()

    def load_image_from_url(self, url):
        try:
            from urllib.request import urlopen

            from PyQt5.QtGui import QImage

            with urlopen(url) as response:
                data = response.read()
                image = QImage()
                image.loadFromData(data)
                return image
        except Exception as e:
            print(f"Error loading image: {e}")
            return None

    def update_advanced_configurations(self):
        current_mosa = self.mosa_combo.currentData()
        if current_mosa:
            self.target_altitude_combo.setCurrentText(current_mosa["TargetAltitude"])
            self.flight_altitude_spin.setValue(current_mosa["FlightAltitude"])
            self.line_distance_spin.setValue(current_mosa["LineDist"])
            self.waypoints_distance_spin.setValue(current_mosa["WaypointsDist"])

    def save_advanced_configurations(self):
        # Get values from UI
        selected_aircraft = self.aircraft_combo.currentData()
        selected_mosa = self.mosa_combo.currentData()
        selected_optimization = self.optimization_combo.currentData()

        self.new_mosa_data = {
            "Aircraft": selected_aircraft["Name"] if selected_aircraft else "VT-14",
            "MOSA": selected_mosa["Name"] if selected_mosa else "MOSA1",
            "Optimization": (
                selected_optimization["Name"] if selected_optimization else "Middle"
            ),
        }

        if selected_aircraft and selected_mosa and selected_optimization:
            # Update the MOSA configurations with the values from UI
            selected_mosa["TargetAltitude"] = self.target_altitude_combo.currentText()
            selected_mosa["FlightAltitude"] = self.flight_altitude_spin.value()
            selected_mosa["LineDist"] = self.line_distance_spin.value()
            selected_mosa["WaypointsDist"] = self.waypoints_distance_spin.value()

            # Prepare data to send to the server
            updated_mosa_data = self.mosa_data
            # Update the MOSA in the list
            for mosa in updated_mosa_data.get("MOSAs", []):
                if mosa["Name"] == selected_mosa["Name"]:
                    mosa.update(selected_mosa)
                    break

            try:
                # Send updated MOSA configurations to the server
                headers = {"Content-Type": "application/json"}
                response = requests.post(
                    "https://patronrouteplanner.zapto.org/api/v2/set_mosa",
                    headers=headers,
                    data=json.dumps(updated_mosa_data),
                )
                response.raise_for_status()
            except requests.RequestException as e:
                QtWidgets.QMessageBox.critical(
                    self,
                    "Erro",
                    f"Não foi possível atualizar as configurações MOSA no servidor:\n{str(e)}",
                )
                return

            # Show a success message
            QtWidgets.QMessageBox.information(
                self, "Sucesso", "Configurações salvas com sucesso!"
            )

            self.accept()
        else:
            QtWidgets.QMessageBox.warning(
                self, "Aviso", "Por favor, preencha todas as configurações."
            )

    def get_mosa_data(self):
        # Return the selected settings
        return self.new_mosa_data
