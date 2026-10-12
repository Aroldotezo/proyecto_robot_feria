from PySide6.QtCore import QObject, Signal

class WorkerSignals(QObject):
    finished = Signal(object) 
    failed = Signal(str)       
