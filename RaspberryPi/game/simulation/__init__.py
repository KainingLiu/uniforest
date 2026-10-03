"""Hardware-free trajectory experiments; output never enables competition motion."""

__all__ = ['SimulationSettings', 'build_catalog', 'run_case', 'run_catalog']


def __getattr__(name):
    if name == 'SimulationSettings':
        from .core import SimulationSettings
        return SimulationSettings
    if name in ('build_catalog', 'run_case', 'run_catalog'):
        from . import catalog
        return getattr(catalog, name)
    raise AttributeError(name)
