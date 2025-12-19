class NodeOptions:
    """Clase base para opciones de configuración de un nodo."""
    def __init__(self, package=None, executable=None, node_name=None):
        self.package = package
        self.executable = executable
        self.node_name = node_name

    def create_widgets(self, parent):
        raise NotImplementedError("Este método debe ser implementado por subclases.")

