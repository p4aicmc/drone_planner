import json
import os
import shutil
import sys
import tempfile
import time
import zipfile
from shapely.geometry import Polygon

import requests
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QSettings

from kml_parser import parse_kml_file
from mosa_settings import MosaSettingsWindow


class MissionPlannerMainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.settings = QSettings("GrapeHawk", "Planejador de Missões")
        self._initialize_app_state()
        self._setup_ui()

    def _initialize_app_state(self):
        """Initialize application state and default values"""
        self.user_directory = self.settings.value(
            "user_directory", os.path.expanduser("~")
        )
        self.mosa_settings_data = None
        self.selected_aircraft = "VT-14"
        self.selected_optimization = "Middle"
        self.selected_file_path = None
        self.current_directory = self.user_directory

    def _setup_ui(self):
        """Set up main window UI components"""
        self.setWindowTitle("Planejador de Missões")
        self.setGeometry(300, 300, 800, 600)

        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        self.main_layout = QtWidgets.QVBoxLayout(central_widget)

        self._create_toolbar()
        self._setup_file_explorer()
        self._create_generate_button()

    def _create_toolbar(self):
        """Create the main toolbar with settings action"""
        toolbar = self.addToolBar("Toolbar")
        toolbar.setMovable(False)

        settings_action = QtWidgets.QAction(
            QtGui.QIcon("assets/icons/gear-solid.svg"), "Configurações", self
        )
        settings_action.triggered.connect(self.open_mosa_settings)
        toolbar.addAction(settings_action)
        toolbar.addSeparator()

    def _setup_file_explorer(self):
        """Set up the file system model and tree view"""
        self.model = QtWidgets.QFileSystemModel()
        self.model.setRootPath(self.user_directory)
        self.model.setNameFilters(["*.kml"])
        self.model.setNameFilterDisables(True)
        self.model.setFilter(
            QtCore.QDir.AllDirs | QtCore.QDir.NoDotAndDotDot | QtCore.QDir.Files
        )

        self.tree = QtWidgets.QTreeView()
        self.tree.setModel(self.model)
        self.tree.setRootIndex(self.model.index(self.user_directory))
        self.tree.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)

        # Configure tree view columns and signals
        for i in range(1, 4):
            self.tree.setColumnHidden(i, True)
        self.tree.selectionModel().selectionChanged.connect(
            self._handle_selection_change
        )
        self.tree.doubleClicked.connect(self._handle_directory_change)
        self.tree.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
        self.tree.customContextMenuRequested.connect(self._show_context_menu)

        self._setup_navigation_controls()

    def _setup_navigation_controls(self):
        """Set up navigation controls including breadcrumb and up button"""
        nav_layout = QtWidgets.QHBoxLayout()

        # Parent directory button
        self.up_button = QtWidgets.QPushButton()
        self.up_button.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_ArrowUp))
        self.up_button.clicked.connect(self._navigate_to_parent)
        nav_layout.addWidget(self.up_button)

        # Breadcrumb navigation
        self.breadcrumb_layout = QtWidgets.QHBoxLayout()
        nav_layout.addLayout(self.breadcrumb_layout)
        nav_layout.addStretch()

        self.main_layout.addLayout(nav_layout)
        self.main_layout.addWidget(self.tree)
        self._update_breadcrumb()

    def _create_generate_button(self):
        """Create and configure the mission generation button"""
        self.generate_button = QtWidgets.QPushButton("Gerar Missão")
        self.generate_button.setEnabled(False)
        self.generate_button.clicked.connect(self._start_mission_generation)
        self.main_layout.addWidget(self.generate_button)

    # Navigation and UI handlers
    def _handle_directory_change(self, index):
        """Handle directory navigation through double-click events"""
        file_path = self.model.filePath(index)
        if os.path.isdir(file_path):
            self.current_directory = file_path
            self.tree.setRootIndex(self.model.index(self.current_directory))
            self._update_ui_state()

    def _navigate_to_parent(self):
        """Navigate to parent directory"""
        parent_dir = os.path.dirname(self.current_directory)
        if os.path.isdir(parent_dir):
            self.current_directory = parent_dir
            self.tree.setRootIndex(self.model.index(self.current_directory))
            self._update_ui_state()

    def _update_ui_state(self):
        """Update UI elements after navigation changes"""
        self._update_breadcrumb()
        self.up_button.setEnabled(self.current_directory != self.model.rootPath())

    def _update_breadcrumb(self):
        """Update breadcrumb navigation display"""
        # Clear existing breadcrumb
        while self.breadcrumb_layout.count():
            item = self.breadcrumb_layout.takeAt(0)
            if widget := item.widget():
                widget.deleteLater()

        path_components = self.current_directory.split(os.sep)
        cumulative_path = ""

        for i, component in enumerate(path_components):
            if not component:
                continue

            cumulative_path = (
                os.path.join(cumulative_path, component) if i > 0 else component
            )
            if os.name == "nt" and i == 0:
                cumulative_path += os.sep

            button = QtWidgets.QPushButton(component)
            button.setFlat(True)
            button.setStyleSheet(
                "QPushButton { text-decoration: underline; color: white; }"
            )
            button.clicked.connect(
                lambda _, p=cumulative_path: self._navigate_to_path(p)
            )
            self.breadcrumb_layout.addWidget(button)

            if i < len(path_components) - 1:
                self.breadcrumb_layout.addWidget(QtWidgets.QLabel(">"))

    def _navigate_to_path(self, path):
        """Navigate to specified directory path"""
        if os.path.isdir(path):
            self.current_directory = path
            self.tree.setRootIndex(self.model.index(self.current_directory))
            self._update_ui_state()

    def _handle_selection_change(self, selected, deselected):
        """Handle file selection changes in the tree view"""
        indexes = self.tree.selectionModel().selectedIndexes()
        self.selected_file_path = None

        if indexes:
            file_path = self.model.filePath(indexes[0])
            if os.path.isfile(file_path) and file_path.lower().endswith(".kml"):
                self.selected_file_path = file_path

        self.generate_button.setEnabled(bool(self.selected_file_path))

    def _show_context_menu(self, position):
        """Show context menu for KML files"""
        indexes = self.tree.selectedIndexes()
        if indexes and self.selected_file_path:
            menu = QtWidgets.QMenu()
            generate_action = menu.addAction("Gerar Missão")
            generate_action.triggered.connect(lambda: self._start_mission_generation())
            menu.exec_(self.tree.viewport().mapToGlobal(position))

    # Mission generation workflow
    def _start_mission_generation(self):
        """Start mission generation process"""
        if self.selected_file_path:
            self._generate_mission(self.selected_file_path)

    def _generate_mission(self, file_path):
        """Handle mission generation workflow"""
        save_path = self._get_save_path(os.path.basename(file_path))
        if not save_path:
            return

        progress_dialog = self._create_progress_dialog()
        QtCore.QTimer.singleShot(
            100, lambda: self._process_mission(file_path, save_path, progress_dialog)
        )

    def _get_save_path(self, filename):
        """Get save path from file dialog"""
        default_name = os.path.splitext(filename)[0] + ".zip"
        save_path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "Salvar Missão",
            os.path.join(os.path.expanduser("~"), default_name),
            "ZIP Files (*.zip);;All Files (*)",
        )
        return save_path if save_path.endswith(".zip") else f"{save_path}.zip"

    def _create_progress_dialog(self):
        """Create and show progress dialog"""
        dialog = QtWidgets.QProgressDialog("Gerando missão...", "Cancelar", 0, 0, self)
        dialog.setWindowTitle("Gerando Missão")
        dialog.setWindowModality(QtCore.Qt.WindowModal)
        dialog.show()
        return dialog

    def _process_mission(self, file_path, save_path, progress_dialog):
        """Main mission processing pipeline"""
        try:
            
            mission_data = self._prepare_mission_data(file_path)
            harpia_data = self._create_harpia_json(mission_data)
            path = save_path[0:save_path.rfind('/')]
            print(f'\n{harpia_data}')
            with open(path+"/map.json", "w") as outfile:
                outfile.write(harpia_data)

            #mission_key = self._send_mission_request(mission_data)
            #final_data = self._poll_mission_ready(mission_key)
            
            #self._package_mission_files(file_path, save_path, mission_key, final_data)
            progress_dialog.close()
            self._show_success_message(save_path)
        except Exception as e:
            progress_dialog.close()
            self._handle_mission_error(os.path.basename(file_path), str(e))

    def _prepare_mission_data(self, file_path):
        """Prepare mission data from KML and MOSA settings"""
        kml_data = parse_kml_file(file_path)
        return {
            "Aircraft": self.selected_aircraft,
            "MOSA": (
                self.mosa_settings_data.get("MOSA", "MOSA1")
                if self.mosa_settings_data
                else "MOSA1"
            ),
            "Optimization": self.selected_optimization,
            **kml_data,
        }

    def _send_mission_request(self, mission_data):
        """Send mission data to server and return mission key"""
        response = requests.post(
            "https://patronrouteplanner.zapto.org/api/v2/calculate",
            headers={"Content-Type": "application/json"},
            data=json.dumps(mission_data, ensure_ascii=False).encode("utf-8"),
        )
        response.raise_for_status()
        return response.json().get("key")

    def _poll_mission_ready(self, mission_key):
        """Poll server until mission is ready"""
        for _ in range(150):
            time.sleep(2)
            response = requests.get(
                f"https://patronrouteplanner.zapto.org/api/v2/get_response/{mission_key}"
            )
            if response.status_code == 200:
                return response.json()
            if response.status_code in (400, 500):
                raise Exception(f"Server error: {response.text}")
        raise Exception("Mission generation timeout")

    def _package_mission_files(self, file_path, save_path, mission_key, final_data):
        """Download and package mission files into ZIP"""
        with tempfile.TemporaryDirectory() as temp_dir:
            files = self._download_mission_files(
                temp_dir, mission_key, os.path.basename(file_path)
            )
            if final_data:
                files.append(self._save_final_data(temp_dir, final_data))
            self._create_mission_zip(save_path, files)

    def _download_mission_files(self, temp_dir, mission_key, base_name):
        """Download mission files from server"""
        base_name = os.path.splitext(base_name)[0]
        files = []

        endpoints = [
            (f"download_waypoint/{mission_key}", f"{base_name}.plan"),
            (f"download_kml/{mission_key}", f"{base_name}.kml"),
        ]

        for endpoint, filename in endpoints:
            response = requests.get(
                f"https://patronrouteplanner.zapto.org/api/v2/{endpoint}"
            )
            response.raise_for_status()
            file_path = os.path.join(temp_dir, filename)
            with open(file_path, "wb") as f:
                f.write(response.content)
            files.append(file_path)

        return files

    def _save_final_data(self, temp_dir, final_data):
        """Save final mission data to JSON file"""
        json_path = os.path.join(temp_dir, "mission_data.json")
        with open(json_path, "w") as f:
            json.dump(final_data, f, indent=2)
        return json_path

    def _create_mission_zip(self, save_path, files):
        """Create ZIP archive from mission files"""
        with zipfile.ZipFile(save_path, "w") as zipf:
            for file_path in files:
                zipf.write(file_path, arcname=os.path.basename(file_path))

    # UI feedback methods
    def _show_success_message(self, save_path):
        QtWidgets.QMessageBox.information(
            self, "Sucesso", f"Missão gerada e salva com sucesso em:\n{save_path}"
        )

    def _handle_mission_error(self, filename, error):
        QtWidgets.QMessageBox.critical(
            self, "Erro", f"Erro ao processar {filename}:\n{error}"
        )

    # MOSA settings management
    def open_mosa_settings(self):
        """Handle MOSA settings window"""
        initial_settings = {
            "Aircraft": self.selected_aircraft,
            "Optimization": self.selected_optimization,
            "MOSA": (
                self.mosa_settings_data.get("MOSA", "MOSA1")
                if self.mosa_settings_data
                else "MOSA1"
            ),
        }

        settings_window = MosaSettingsWindow(self, initial_settings)
        if settings_window.exec_() == QtWidgets.QDialog.Accepted:
            self._update_mosa_settings(settings_window.get_mosa_data())

    def _update_mosa_settings(self, new_settings):
        """Update application state with new MOSA settings"""
        self.mosa_settings_data = new_settings
        self.selected_aircraft = new_settings.get("Aircraft", "VT-14")
        self.selected_optimization = new_settings.get("Optimization", "Middle")
        self._reload_directory_settings()

    def _reload_directory_settings(self):
        """Reload directory settings from persistent storage"""
        self.user_directory = self.settings.value(
            "user_directory", os.path.expanduser("~")
        )
        self.model.setRootPath(self.user_directory)
        self.tree.setRootIndex(self.model.index(self.user_directory))
        self.current_directory = self.user_directory
        self._update_ui_state()

    # Harpia additions
    def _create_harpia_json(self, mission_data):
        harpiaFormat = dict()
        harpiaFormat['id'] = 0
        harpiaFormat['name'] = 'Generated with Patron Mission Generator'
        harpiaFormat['geo_home'] = []
        harpiaFormat['bases'] = []
        harpiaFormat['roi'] = []
        harpiaFormat['nfz'] = []
        for i in range(len(mission_data['home']['polygons'])):
            geo_points = list(map(self._coords_map ,mission_data['home']['polygons'][i]['coords']))
            poly = Polygon(list(map(self._coords_poly,mission_data['home']['polygons'][i]['coords'])))
            harpiaFormat['bases'].append({'id': i, 'name': mission_data['home']['polygons'][i]['name'], 'geo_points': geo_points[0:-1], 'center' : [poly.centroid.x, poly.centroid.y, 0] })
            harpiaFormat['geo_home'] = [poly.centroid.x, poly.centroid.y, 0]

        for i in range(len(mission_data['ROIs']['polygons'])):
            geo_points = list(map(self._coords_map ,mission_data['ROIs']['polygons'][i]['coords']))
            poly = Polygon(list(map(self._coords_poly,mission_data['ROIs']['polygons'][i]['coords'])))
            harpiaFormat['roi'].append({'id': i, 'name': mission_data['ROIs']['polygons'][i]['name'], 'geo_points': geo_points[0:-1], 'center' : [poly.centroid.x, poly.centroid.y, 0] })

        for i in range(len(mission_data['NFZs']['polygons'])):
            geo_points = list(map(self._coords_map ,mission_data['NFZs']['polygons'][i]['coords']))
            poly = Polygon(list(map(self._coords_poly,mission_data['NFZs']['polygons'][i]['coords'])))
            harpiaFormat['nfz'].append({'id': i, 'name': mission_data['NFZs']['polygons'][i]['name'], 'geo_points': geo_points[0:-1], 'center' : [poly.centroid.x, poly.centroid.y, 0] })
        
        json_object = json.dumps(harpiaFormat, indent=4)
        return json_object

    def _coords_map(self, coord):
        return [coord['lng'], coord['lat'], 0]
    
    def _coords_poly(self, coord):
        return (coord['lng'], coord['lat'])




def main():
    app = QtWidgets.QApplication(sys.argv)
    window = MissionPlannerMainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
