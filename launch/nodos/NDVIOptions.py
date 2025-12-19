from .NodeOptions import NodeOptions
import tkinter as tk
from tkinter import ttk

class NDVIOptions(NodeOptions):
    def __init__(self):
        super().__init__()
        self.selected_model = tk.StringVar(value="Continuous")  # Modo seleccionado
        self.selected_model_name = tk.StringVar(value="cont")  # Nombre del modo a usar
        self.available_models = ["Continuous", "Discontinuous"]  # Lista de modelos disponibles

    def create_widgets(self, parent):
        # Etiqueta para seleccionar el modelo
        ttk.Label(parent, text="Select Mode", font=("Arial", 12)).pack(anchor="center", pady=5)

        # Combobox para seleccionar el modelo
        model_combobox = ttk.Combobox(
            parent,
            textvariable=self.selected_model,
            values=self.available_models,
            state="readonly",
            width=20
        )
        model_combobox.pack(anchor="center", pady=10)
        model_combobox.bind("<<ComboboxSelected>>", self.on_model_selected)

    def on_model_selected(self, event):
        """Callback al seleccionar un modelo."""
        model = self.selected_model.get()
        if model == "Continuous":
            self.selected_model_name.set("cont")
        elif model == "Discontinuous":
            self.selected_model_name.set("disc")
        else:
            self.selected_model_name.set("None")
        print(f"Selected YOLO model: {model}, Mode Name: {self.selected_model_name.get()}")