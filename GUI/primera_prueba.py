import os
os.environ["QT_QPA_PLATFORM"] = "xcb"  # Solución para Wayland

from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QPushButton, QLineEdit, QLabel, QFrame
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPainter, QColor, QBrush, QFont
from PyQt5.QtWidgets import QSizePolicy

class SwitchButton(QWidget):
    """ Switch visual estilo ON/OFF con etiquetas 'Sí' y 'No' """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(70, 30)
        self.checked = False

    def toggle(self):
        """ Cambia el estado del switch """
        self.checked = not self.checked
        self.update()
        parent = self.parentWidget()
        if parent and hasattr(parent, 'toggle_variac'):
            parent.toggle_variac(self.checked)

    def paintEvent(self, event):
        """ Dibuja el switch """
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        bg_color = QColor("#0078D7") if self.checked else QColor("#B0B0B0")
        painter.setBrush(QBrush(bg_color))
        painter.setPen(Qt.NoPen)
        painter.drawRoundedRect(0, 0, self.width(), self.height(), 15, 15)

        painter.setPen(Qt.white)
        painter.setFont(QFont("Arial", 10, QFont.Bold))
        if self.checked:
            painter.drawText(10, 20, "Sí")
        else:
            painter.drawText(self.width() - 30, 20, "No")

        circle_x = self.width() - 30 if self.checked else 5
        painter.setBrush(QBrush(Qt.white))
        painter.drawEllipse(circle_x, 5, 20, 20)

    def mousePressEvent(self, event):
        """ Detecta el clic y cambia el estado """
        self.toggle()

class ElectrospinningUI(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Interfaz Electrospinning")
        self.setFixedSize(550, 400)  # La ventana no cambiará de tamaño

        main_layout = QVBoxLayout()

        self.title_label = QLabel("Interfaz Electrospinning")
        self.title_label.setAlignment(Qt.AlignCenter)
        self.title_label.setStyleSheet("background-color: #0078D7; color: white; font-size: 20px; font-weight: bold; padding: 0px; border-radius: 10px;")
        self.title_label.setFixedSize(528,40)  

        self.sample_frame = QFrame()
        self.sample_frame.setFrameShape(QFrame.StyledPanel)
        self.sample_frame.setStyleSheet("background-color: #EAEAEA; border: 2px solid #B0B0B0; border-radius: 5px; padding: 1px;")
        sample_layout = QHBoxLayout(self.sample_frame)

        self.sample_label = QLabel("Tipo de Muestra")
        self.sample_label.setStyleSheet("font-size: 14px; font-weight: bold;")
        self.sample_label.setFixedSize(490, 30) 

        self.switch_label = QLabel("¿Se usará Colector?")
        self.switch_label.setFixedSize(150, 30)  

        self.switch_button = SwitchButton(self)
        self.switch_button.mousePressEvent = lambda event: self.toggle_variac(not self.switch_button.checked)  # Conectar correctamente

        sample_layout.addWidget(self.sample_label)
        sample_layout.addStretch()
        sample_layout.addWidget(self.switch_label)
        sample_layout.addWidget(self.switch_button)

        self.params_frame = QFrame()
        self.params_frame.setFrameShape(QFrame.StyledPanel)
        self.params_frame.setStyleSheet("background-color: #EAEAEA; border: 2px solid #B0B0B0; border-radius: 5px; padding: 5px;")
        params_layout = QVBoxLayout(self.params_frame)
        self.params_label = QLabel("Parámetros")
        self.params_label.setAlignment(Qt.AlignCenter)
        self.params_label.setStyleSheet("font-size: 14px; font-weight: bold;")
        self.params_label.setFixedSize(500,30)  # Tamaño fijo para que no cambie

        # Evitar que los marcos de los títulos cambien de tamaño dinámicamente
        self.params_frame.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.sample_frame.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)

        params_layout.addWidget(self.params_label)
        grid_layout = QGridLayout()
        
        # Variables internas
        self.parameters = {"syringe": None, "voltage": None, "variac": None}

        def send_value(field):
            """ Actualiza el color del texto y almacena el valor al presionar enviar. """
            value = field.text()
            field.setStyleSheet("color: red; " + self.get_text_input_style())
            return value

        def reset_text_color(field):
            """ Restaura el color del texto si se edita después de enviarlo. """
            field.setStyleSheet(self.get_text_input_style())

        # Jeringa/Bomba
        self.syringe_label = QLabel("Jeringa/Bomba")
        self.syringe_input = QLineEdit()
        self.syringe_input.setPlaceholderText("0.0")
        self.syringe_input.setStyleSheet(self.get_text_input_style())
        self.syringe_unit = QLabel("mm/s")
        self.syringe_button = QPushButton("Enviar")
        self.syringe_button.setStyleSheet(self.get_button_style())

        self.syringe_input.textEdited.connect(lambda: reset_text_color(self.syringe_input))
        self.syringe_button.clicked.connect(lambda: self.update_parameter("syringe", send_value(self.syringe_input)))

        grid_layout.addWidget(self.syringe_label, 0, 0)
        grid_layout.addWidget(self.syringe_input, 0, 1)
        grid_layout.addWidget(self.syringe_unit, 0, 2)
        grid_layout.addWidget(self.syringe_button, 0, 3)

        # Fuente de alto voltaje
        self.voltage_label = QLabel("Fuente de alto Voltaje")
        self.voltage_input = QLineEdit()
        self.voltage_input.setPlaceholderText("0.0")
        self.voltage_input.setStyleSheet(self.get_text_input_style())
        self.voltage_unit = QLabel("KV")
        self.voltage_button = QPushButton("Enviar")
        self.voltage_button.setStyleSheet(self.get_button_style())

        self.voltage_input.textEdited.connect(lambda: reset_text_color(self.voltage_input))
        self.voltage_button.clicked.connect(lambda: self.update_parameter("voltage", send_value(self.voltage_input)))

        grid_layout.addWidget(self.voltage_label, 1, 0)
        grid_layout.addWidget(self.voltage_input, 1, 1)
        grid_layout.addWidget(self.voltage_unit, 1, 2)
        grid_layout.addWidget(self.voltage_button, 1, 3)

        # Variador Trifásico
        self.variac_label = QLabel("Variador trifásico")
        self.variac_input = QLineEdit()
        self.variac_input.setPlaceholderText("0.0")
        self.variac_input.setStyleSheet(self.get_text_input_style())
        self.variac_unit = QLabel("rpm")
        self.variac_button = QPushButton("Enviar")
        self.variac_button.setStyleSheet(self.get_button_style())

        self.variac_input.textEdited.connect(lambda: reset_text_color(self.variac_input))
        self.variac_button.clicked.connect(lambda: self.update_parameter("variac", send_value(self.variac_input)))

        # Agregar cada elemento individualmente al grid_layout para que esté alineado
        grid_layout.addWidget(self.variac_label, 2, 0)
        grid_layout.addWidget(self.variac_input, 2, 1)
        grid_layout.addWidget(self.variac_unit, 2, 2)
        grid_layout.addWidget(self.variac_button, 2, 3)

        # Inicialmente oculto
        self.variac_label.setVisible(False)
        self.variac_input.setVisible(False)
        self.variac_unit.setVisible(False)
        self.variac_button.setVisible(False)

        # Botón enviar todo
        self.submit_button = QPushButton("Enviar todo")
        self.submit_button.setStyleSheet(self.get_button_style())
        self.submit_button.clicked.connect(self.send_all)

        main_layout.addWidget(self.title_label)
        main_layout.addWidget(self.sample_frame)
        main_layout.addWidget(self.params_frame)
        main_layout.addLayout(grid_layout)
        main_layout.addWidget(self.submit_button)

        self.setLayout(main_layout)
        self.setStyleSheet("background-color: #f5f5f5;")

    def get_button_style(self):
        return """
            QPushButton {
                background-color: #0078D7;
                color: white;
                border-radius: 10px;
                padding: 8px;
                font-size: 14px;
            }
            QPushButton:hover {
                background-color: #005A9E;
            }
        """

    def get_text_input_style(self):
        return """
            QLineEdit {
                border: 2px solid #0078D7;
                border-radius: 5px;
                padding: 5px;
                font-size: 14px;
                color: black;
            }
        """
    
    def toggle_variac(self, state):
        """ Muestra u oculta el Variador Trifásico dependiendo del estado del switch y actualiza la variable interna. """
        self.variac_label.setVisible(state)
        self.variac_input.setVisible(state)
        self.variac_unit.setVisible(state)
        self.variac_button.setVisible(state)

        # Actualizar la variable interna
        self.parameters["colector"] = state  
        print(f"colector actualizado a {state}")  # Imprimir en consola

        # Asegurar que el estado del switch se actualiza correctamente
        self.switch_button.checked = state  
        self.switch_button.update()  # Redibuja el switch para reflejar el cambio

    def update_parameter(self, key, value):
        """ Actualiza el valor del parámetro solo cuando se presiona enviar. """
        if value:
            self.parameters[key] = value
            print(f"{key} actualizado a {value}")

    def send_all(self):
        """ Actualiza todos los valores visibles. """
        self.syringe_button.click()
        self.voltage_button.click()
        
        if self.variac_label.isVisible():  # Solo actualizar si el Variador Trifásico está visible
            self.variac_button.click()

if __name__ == "__main__":
    app = QApplication([])
    window = ElectrospinningUI()
    window.show()
    app.exec_()