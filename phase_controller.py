class PhaseControl:
    def __init__(self, functions) -> None:
        self.phases_functions = functions # [(func, transition_func), ...]
        self.phase_index = 0
        self.phase_max_index = len(self.phases_functions)
        self.func = None
        self.transition_func = lambda: None
        self.update_func()

    def update_func(self):
        if self.phase_index >= self.phase_max_index: exit()

        self.func = self.phases_functions[self.phase_index][0]
        self.transition_func = self.phases_functions[self.phase_index][1]
        if self.transition_func is None: self.transition_func = lambda: None

    def next_phase(self):
        self.phase_index += 1
        self.transition_func()
        self.update_func()
    
    def set_phase(self, phase_index):
        self.phase_index = phase_index
        self.update_func()

    def loop(self):
        self.func()