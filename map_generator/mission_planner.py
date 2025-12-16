import json
import os
import sys
from shapely.geometry import Polygon

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QSettings

from kml_parser import parse_kml_file

class MissionPlannerMainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.settings = QSettings("Mission Planner", "Mission Planner")
        self._initialize_app_state()
        self._setup_ui()

    def _initialize_app_state(self):
        """Initialize application state and default values"""
        self.user_directory = self.settings.value(
            "user_directory", os.path.expanduser("~")
        )
        self.mosa_settings_data = None
        self.selected_file_path = None
        self.current_directory = self.user_directory

    def _setup_ui(self):
        """Set up main window UI components"""
        self.setWindowTitle("Mission Planner")
        self.setGeometry(300, 300, 500, 200)

        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        self.main_layout = QtWidgets.QVBoxLayout(central_widget)

        self._create_file_selection_ui()
        self._create_generate_button()

    def _create_file_selection_ui(self):
        """Create UI for file selection using native file dialog"""
        # File selection layout
        file_layout = QtWidgets.QHBoxLayout()
        
        # Label to show selected file
        self.file_label = QtWidgets.QLabel("No file selected")
        self.file_label.setStyleSheet("padding: 10px; border: 1px solid #555; border-radius: 5px;")
        file_layout.addWidget(self.file_label, stretch=1)
        
        # Button to open file dialog
        self.open_button = QtWidgets.QPushButton("Select KML")
        self.open_button.clicked.connect(self._open_kml_file_dialog)
        file_layout.addWidget(self.open_button)
        
        self.main_layout.addLayout(file_layout)

    def _open_kml_file_dialog(self):
        """Open native file dialog to select a KML file"""
        file_path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Select a KML file",
            self.current_directory,
            "KML Files (*.kml);;All Files (*)",
            options=QtWidgets.QFileDialog.DontUseNativeDialog
        )
        if file_path and file_path.lower().endswith(".kml"):
            self.selected_file_path = file_path
            self.current_directory = os.path.dirname(file_path)
            self.file_label.setText(os.path.basename(file_path))
            self.generate_button.setEnabled(True)
        elif file_path:
            self._show_error("Invalid File", "Please select a .kml file")

    def _create_generate_button(self):
        """Create and configure the mission generation button"""
        self.generate_button = QtWidgets.QPushButton("Generate Map")
        self.generate_button.setEnabled(False)
        self.generate_button.clicked.connect(self._start_mission_generation)
        self.main_layout.addWidget(self.generate_button)

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
        default_name = os.path.splitext(filename)[0] + ".json"
        save_path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "Save map",
            os.path.join(os.path.expanduser("~"), default_name),
            "JSON Files (*.json);;All Files (*)",
            options=QtWidgets.QFileDialog.DontUseNativeDialog
        )
        return save_path if save_path.endswith(".json") else f"{save_path}.json"

    def _create_progress_dialog(self):
        """Create and show progress dialog"""
        dialog = QtWidgets.QProgressDialog("Generating map...", "Cancel", 0, 0, self)
        dialog.setWindowTitle("Generating Mission")
        dialog.setWindowModality(QtCore.Qt.WindowModal)
        dialog.show()
        return dialog

    def _process_mission(self, file_path, save_path, progress_dialog):
        """Main mission processing pipeline"""
        try:
            mission_data = self._prepare_mission_data(file_path)
            harpia_data = self._create_harpia_json(mission_data)
            
            # Save JSON file with the user-selected path
            with open(save_path, "w") as outfile:
                outfile.write(harpia_data)

            progress_dialog.close()
            self._show_success_message(save_path)
        except Exception as e:
            progress_dialog.close()
            self._handle_mission_error(os.path.basename(file_path), str(e))

    def _prepare_mission_data(self, file_path):
        """Prepare mission data from KML and MOSA settings"""
        kml_data = parse_kml_file(file_path)
        return {
            **kml_data,
        }

    # UI feedback methods
    def _show_success_message(self, save_path):
        QtWidgets.QMessageBox.information(
            self, "Success", f"Map generated in:\n{save_path}"
        )

    def _handle_mission_error(self, filename, error):
        QtWidgets.QMessageBox.critical(
            self, "Error", f"Error processing {filename}:\n{error}"
        )

    def _show_error(self, title, message):
        """Show error dialog"""
        QtWidgets.QMessageBox.critical(self, title, message)

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
