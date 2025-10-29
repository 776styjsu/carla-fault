"""Faults package utilities and dynamic registry."""

from __future__ import annotations

import importlib
import pkgutil
import re
from typing import Dict, List, Type

from .base_fault import BaseFault

__all__ = [
	"BaseFault",
	"create_fault",
	"get_fault_class",
	"list_available_faults",
]

_fault_registry: Dict[str, Type[BaseFault]] = {}
_canonical_names: Dict[Type[BaseFault], str] = {}
_DISCOVERED = False


def _camel_to_snake(name: str) -> str:
	first_pass = re.sub(r"(.)([A-Z][a-z]+)", r"\1_\2", name)
	second_pass = re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", first_pass)
	return second_pass.lower()


def _normalize_key(name: str) -> str:
	if not name:
		return ""
	name = name.replace('-', '_').replace(' ', '_')
	if not name.islower():
		name = _camel_to_snake(name)
	if name.endswith('_fault'):
		name = name[:-6]
	return name.strip('_').lower()


def _register_fault_class(cls: Type[BaseFault]) -> None:
	canonical = getattr(cls, "fault_name", None) or _normalize_key(cls.__name__)
	if canonical:
		canonical = _normalize_key(canonical)
		_canonical_names.setdefault(cls, canonical)

	candidates = {
		canonical,
		_normalize_key(cls.__name__),
		_normalize_key(cls.__name__.replace('Fault', '')),
		_normalize_key(cls.__module__.split('.')[-1]),
	}

	for key in candidates:
		if key:
			_fault_registry.setdefault(key, cls)


def _discover_faults() -> None:
	global _DISCOVERED
	if _DISCOVERED:
		return

	package_path = __path__  # type: ignore[name-defined]
	package_name = __name__

	for _, module_name, is_pkg in pkgutil.iter_modules(package_path):
		if is_pkg or module_name.startswith('_') or module_name == 'base_fault':
			continue
		module = importlib.import_module(f"{package_name}.{module_name}")
		for attr in module.__dict__.values():
			if isinstance(attr, type) and issubclass(attr, BaseFault) and attr is not BaseFault:
				_register_fault_class(attr)

	_DISCOVERED = True


def list_available_faults() -> List[str]:
	"""Return canonical fault names discovered in the package."""
	_discover_faults()
	return sorted({_canonical_names[cls] for cls in _canonical_names})


def get_fault_class(name: str) -> Type[BaseFault]:
	"""Resolve a fault name or dotted class path to a fault class."""
	_discover_faults()

	if not name:
		raise ValueError("Fault name must be provided")

	normalized = _normalize_key(name)

	if '.' in name and name not in _fault_registry:
		module_path, _, class_name = name.rpartition('.')
		module = importlib.import_module(module_path)
		attr = getattr(module, class_name)
		if isinstance(attr, type) and issubclass(attr, BaseFault):
			_register_fault_class(attr)
			return attr
		raise ValueError(f"{name} is not a BaseFault subclass")

	if normalized in _fault_registry:
		return _fault_registry[normalized]

	available = ', '.join(list_available_faults()) or 'none'
	raise ValueError(f"Unknown fault '{name}'. Available faults: {available}")


def create_fault(name: str, *args, **kwargs) -> BaseFault:
	"""Instantiate a fault by name."""
	cls = get_fault_class(name)
	return cls(*args, **kwargs)


# Optional: keep named exports for direct access when available
try:  # pragma: no cover - optional convenience import
	from .camera_blackout import CameraBlackoutFault  # type: ignore F401
except ImportError:  # pragma: no cover
	CameraBlackoutFault = None  # type: ignore

