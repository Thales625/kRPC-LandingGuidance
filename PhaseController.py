class PhaseController:
    def __init__(self, functions) -> None:
        self.phases_functions = functions
        self.phase_index = 0
        self.update_func()

    def update_func(self):
        self.func = self.phases_functions[self.phase_index]

    def next_phase(self):
        self.phase_index += 1
        self.update_func()
    
    def set_phase(self, phase_index):
        self.phase_index = phase_index
        self.update_func()

    def loop(self):
        self.func()