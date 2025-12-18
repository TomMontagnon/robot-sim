from .abstract_adapter import AbstractAdapter


class UnicycleAdapter(AbstractAdapter):
    def map(self, uc):
        return uc  # déjà [v, w]

